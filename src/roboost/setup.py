import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = "roboost"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.py")),
        (os.path.join("share", package_name), glob("urdf/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="geibinger",
    maintainer_email="friedl.jak@gmail.com",
    description="Package for broadcasting tf transformations for the Roboost Mecanum robot",  # TODO
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["odom_to_base_node = roboost.odom_to_base_node:main"],
    },
)
