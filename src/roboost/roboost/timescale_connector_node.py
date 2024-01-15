import os
import rclpy
from rclpy.node import Node
import yaml
import psycopg2
from psycopg2 import sql
from ament_index_python import get_package_share_directory


class TimescaleDBLogger(Node):
    def __init__(self):
        super().__init__("timescale_logger")
        self.read_config()
        self.establish_db_connection()
        self.create_subscriptions()

    def read_config(self):
        package_share_directory = get_package_share_directory("roboost")
        conf_file = os.path.join(package_share_directory, "config", "timescale.yaml")
        with open(conf_file, "r") as file:
            config = yaml.safe_load(file)
        self.db_config = config["db_config"]
        self.topics = config["topics"]

    def establish_db_connection(self):
        self.conn = psycopg2.connect(
            host=self.db_config["host"],
            port=self.db_config["port"],
            dbname=self.db_config["dbname"],
            user=self.db_config["user"],
            password=self.db_config["password"],
        )
        self.cursor = self.conn.cursor()

    def create_subscriptions(self):
        for topic, topic_config in self.topics.items():
            module_name, class_name = topic_config["msg_type"].rsplit(".", 1)
            module = __import__(module_name, fromlist=[class_name])
            msg_class = getattr(module, class_name)

            # Create the table for the topic if it doesn't exist
            table_name = topic.replace("/", "_").lstrip("_")
            if "table_structure" in topic_config:
                self.create_table_if_not_exists(
                    table_name, topic_config["table_structure"]
                )

            self.create_subscription(
                msg_class,
                topic,
                lambda msg, topic=topic: self.topic_callback(msg, topic),
                10,
            )

    def topic_callback(self, msg, topic):
        table_name = topic.replace("/", "_").lstrip("_")
        table_structure = self.topics[topic]["table_structure"]

        # Extracting data from the message based on table structure
        msg_data = []
        for column in table_structure:
            column_name = column["column_name"]
            if hasattr(msg, column_name):
                value = getattr(msg, column_name)
                # Here you might need to convert the value based on its type
                # For example, if it's a complex type, you might need to serialize it
                msg_data.append(value)
            else:
                self.get_logger().warn(
                    f"Column '{column_name}' not found in message for topic '{topic}'"
                )

        # Only proceed if msg_data matches the number of columns (excluding 'id' and 'time')
        if len(msg_data) == len(table_structure):
            self.insert_into_table(table_name, msg_data)
        else:
            self.get_logger().error(
                f"Mismatch in number of data fields for topic '{topic}'"
            )

    def insert_into_table(self, table_name, msg_data):
        # Constructing the column names and placeholders for SQL
        columns = [col["column_name"] for col in self.topics[topic]["table_structure"]]
        placeholders = ", ".join(["%s"] * len(columns))
        columns_sql = ", ".join(columns)

        self.cursor.execute(
            sql.SQL(
                f"""
            INSERT INTO {table_name} ({columns_sql}) VALUES ({placeholders})
        """
            ),
            msg_data,
        )
        self.conn.commit()

    def create_table_if_not_exists(self, table_name, table_structure):
        columns_sql = ", ".join(
            [f"{col['column_name']} {col['column_type']}" for col in table_structure]
        )
        self.cursor.execute(
            sql.SQL(
                f"""
            CREATE TABLE IF NOT EXISTS {table_name} (
                id SERIAL PRIMARY KEY,
                {columns_sql},
                time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """
            )
        )
        self.conn.commit()

    def insert_into_table(self, table_name, msg_data):
        self.cursor.execute(
            sql.SQL(
                f"""
        INSERT INTO {table_name} (data) VALUES (%s)
        """
            ),
            [msg_data],
        )
        self.conn.commit()

    def close_db_connection(self):
        self.cursor.close()
        self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = TimescaleDBLogger()
    rclpy.spin(node)

    node.close_db_connection()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "main":
    main()
