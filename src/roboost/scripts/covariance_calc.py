import os
import psycopg2
import numpy as np
import pandas as pd
import yaml


def load_db_config(config_path):
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)
    return config["db_config"]


def get_odometry_data(conn):
    odometry_query = "SELECT pose_pose_position_x AS x, pose_pose_position_y AS y, pose_pose_orientation_z AS yaw, twist_twist_linear_x AS linear_velocity_x, twist_twist_linear_y AS linear_velocity_y, twist_twist_angular_z AS angular_velocity_z FROM odom;"
    odometry_data = pd.read_sql_query(odometry_query, conn)
    return odometry_data


def calculate_covariance(odometry_data):
    relevant_data = odometry_data[
        [
            "x",
            "y",
            "yaw",
            "linear_velocity_x",
            "linear_velocity_y",
            "angular_velocity_z",
        ]
    ]
    cov_matrix = np.cov(relevant_data, rowvar=False)
    return cov_matrix


def main():
    script_dir = os.path.dirname(__file__)
    config_path = os.path.join(script_dir, "../config/timescale.yaml")
    db_params = load_db_config(config_path)
    conn = psycopg2.connect(**db_params)
    odometry_data = get_odometry_data(conn)
    cov_matrix = calculate_covariance(odometry_data)
    print("Covariance Matrix:\n", cov_matrix)


if __name__ == "__main__":
    main()
