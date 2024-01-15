import os
from glob import glob
from setuptools import setup, find_packages

package_name = "roboost"


# Function to create the proper data_files structure
def package_files(directory):
    paths = []
    for path, directories, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(
                (
                    os.path.join("share", package_name, path),
                    [os.path.join(path, filename)],
                )
            )
    return paths


# List of all the directories to include
directories = ["launch", "robot_descriptions", "config", "meshes", "audio"]

# Creating the data files list
data_files = [
    (
        os.path.join("share", "ament_index", "resource_index", "packages"),
        [os.path.join("resource", package_name)],
    ),
    (os.path.join("share", package_name), ["package.xml"]),
]
for directory in directories:
    data_files.extend(package_files(directory))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="geibinger",
    maintainer_email="friedl.jak@gmail.com",
    description="Package for broadcasting tf transformations for the Roboost Mecanum robot",  # TODO
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_to_base = roboost.odom_to_base_node:main",
            "camera_publisher = roboost.camera_publisher_node:main",
            "audio_player = roboost.audio_player_node:main",
            "timescale_connector = roboost.timescale_connector_node:main",
        ],
    },
)
