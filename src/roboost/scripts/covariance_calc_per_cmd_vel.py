import os
import psycopg2
import numpy as np
import pandas as pd
import yaml


def load_db_config(config_path):
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)
    return config["db_config"]


def get_data(conn):
    odometry_query = "SELECT time, pose_pose_position_x AS x, pose_pose_position_y AS y, pose_pose_orientation_z AS yaw, twist_twist_linear_x AS linear_velocity_x, twist_twist_linear_y AS linear_velocity_y, twist_twist_angular_z AS angular_velocity_z FROM odom;"
    odometry_data = pd.read_sql_query(odometry_query, conn)

    cmd_vel_query = "SELECT time, linear_x, linear_y, angular_z FROM cmd_vel;"
    cmd_vel_data = pd.read_sql_query(cmd_vel_query, conn)

    return odometry_data, cmd_vel_data


def sync_data(odometry_data, cmd_vel_data):
    odometry_data["time"] = pd.to_datetime(odometry_data["time"])
    cmd_vel_data["time"] = pd.to_datetime(cmd_vel_data["time"])

    cmd_vel_data_sorted = cmd_vel_data.sort_values(by="time")
    synced_data = []

    for _, odometry_row in odometry_data.iterrows():
        valid_cmds = cmd_vel_data_sorted[
            cmd_vel_data_sorted["time"] <= odometry_row["time"]
        ]
        latest_cmd_vel = (
            valid_cmds.iloc[-1]
            if not valid_cmds.empty
            else pd.Series(
                [np.nan] * len(cmd_vel_data.columns), index=cmd_vel_data.columns
            )
        )
        synced_row = odometry_row.to_dict()
        synced_row.update(latest_cmd_vel.to_dict())
        synced_data.append(synced_row)

    return pd.DataFrame(synced_data)


def calculate_covariance(synced_data):
    cmd_vel_configurations = synced_data[
        ["linear_x", "linear_y", "angular_z"]
    ].drop_duplicates()

    for _, config in cmd_vel_configurations.iterrows():
        config_data = synced_data[
            (synced_data["linear_x"] == config["linear_x"])
            & (synced_data["linear_y"] == config["linear_y"])
            & (synced_data["angular_z"] == config["angular_z"])
        ]
        cov_matrix = np.cov(
            config_data[
                [
                    "x",
                    "y",
                    "yaw",
                    "linear_velocity_x",
                    "linear_velocity_y",
                    "angular_velocity_z",
                ]
            ],
            rowvar=False,
        )
        print(
            f"Covariance Matrix for cmd_vel [{config['linear_x']}, {config['linear_y']}, {config['angular_z']}]:\n",
            cov_matrix,
            "\n",
        )


def main():
    script_dir = os.path.dirname(__file__)
    config_path = os.path.join(script_dir, "../config/timescale.yaml")
    db_params = load_db_config(config_path)
    conn = psycopg2.connect(**db_params)
    odometry_data, cmd_vel_data = get_data(conn)
    synced_data = sync_data(odometry_data, cmd_vel_data)
    calculate_covariance(synced_data)


if __name__ == "__main__":
    main()
