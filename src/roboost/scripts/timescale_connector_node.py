#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
import yaml
import psycopg2
from psycopg2 import sql
from ament_index_python import get_package_share_directory


# TODO timestamp from the message itself, 1 timestamp from the database
class TimescaleDBLogger(Node):
    def __init__(self):
        super().__init__("timescale_connector")
        self.read_config()
        self.establish_db_connection()
        self.create_subscriptions()

    def read_config(self):
        """
        Read the configuration file.
        :return: None
        """
        package_share_directory = get_package_share_directory("roboost")
        conf_file = os.path.join(package_share_directory, "config", "timescale.yaml")
        with open(conf_file, "r") as file:
            config = yaml.safe_load(file)
        self.db_config = config["db_config"]
        self.topics = config["topics"]

    def establish_db_connection(self):
        """
        Establish a connection to the database.
        :return: None
        """
        self.conn = psycopg2.connect(
            host=self.db_config["host"],
            port=self.db_config["port"],
            dbname=self.db_config["dbname"],
            user=self.db_config["user"],
            password=self.db_config["password"],
        )
        self.cursor = self.conn.cursor()

    def create_subscriptions(self):
        """
        Create subscriptions to the topics specified in the configuration file.
        :return: None
        """
        for topic, topic_config in self.topics.items():
            module_name, class_name = topic_config["msg_type"].rsplit(".", 1)
            module = __import__(module_name, fromlist=[class_name])
            msg_class = getattr(module, class_name)

            # Create the table for the topic if it doesn't exist
            table_name = topic.replace("/", "_").lstrip("_")
            if "msg_structure" in topic_config:
                self.create_table_if_not_exists(
                    table_name, topic_config["msg_structure"]
                )

            self.create_subscription(
                msg_class,
                topic,
                lambda msg, topic=topic: self.topic_callback(msg, topic),
                10,
            )

    def topic_callback(self, msg, topic):
        """
        Callback function for the subscribed topics.
        :param msg: Message received.
        :param topic: ROS topic name.
        :return: None
        """
        table_name = topic.replace("/", "_").lstrip("_")
        table_structure = self.topics[topic]["msg_structure"]

        msg_data = []
        for column in table_structure:
            column_name = column["msg_value"]
            attr_parts = column_name.split("/")
            value = msg
            for part in attr_parts:
                if hasattr(value, part):
                    value = getattr(value, part)
                else:
                    self.get_logger().warn(
                        f"Attribute '{part}' not found in message for topic '{topic}'"
                    )
                    value = None
                    break
            if value is not None:
                if isinstance(value, list) or isinstance(value, np.ndarray):
                    value = list_to_db_format_function(value)
                msg_data.append(value)

        if len(msg_data) == len(table_structure):
            self.insert_into_table(table_name, topic, msg_data)
        else:
            self.get_logger().warn(
                f"Message for topic '{topic}' does not contain all the required attributes. Message is:\n{msg_data}\nTable structure is:\n{table_structure}"
            )

    def create_table_if_not_exists(self, table_name, table_structure):
        """
        Create a table in the database if it doesn't exist.
        :param table_name: Name of the table to be created.
        :param table_structure: A list of dictionaries containing the column name and type.
        :return: None
        """
        columns_sql = ", ".join(
            [
                f"{col['msg_value'].replace('/', '_')} {col['column_type']}"
                for col in table_structure
            ]
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

    def insert_into_table(self, table_name, topic, msg_data):
        """
        Insert a message into a table.
        :param table_name: Name of the table to insert into.
        :param topic: ROS topic name.
        :param msg_data: Data to be inserted into the table.
        :return: None
        """
        # Get the column names from the configuration and replace dots with underscores
        columns = [
            col["msg_value"].replace("/", "_")
            for col in self.topics[topic]["msg_structure"]
        ]

        # Construct the column names and placeholders for SQL
        columns_sql = ", ".join(columns)
        placeholders = ", ".join(["%s"] * len(columns))

        # Execute the INSERT statement
        self.cursor.execute(
            sql.SQL(
                f"INSERT INTO {table_name} ({columns_sql}) VALUES ({placeholders})"
            ),
            msg_data,
        )
        self.conn.commit()

    def close_db_connection(self):
        self.cursor.close()
        self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = TimescaleDBLogger()
    print("Spinning timescale logger node")
    rclpy.spin(node)
    node.close_db_connection()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def list_to_db_format_function(value):
    """
    Converts a Python list or numpy array to a PostgreSQL array format.
    :param value: List or numpy array to be converted.
    :return: A string representing the PostgreSQL array.
    """
    if isinstance(value, np.ndarray):
        value = value.tolist()

    return "{" + ",".join([str(element) for element in value]) + "}"
