from setuptools import find_packages, setup

package_name = "robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/bringup.launch.py",
            "launch/microros_agent.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marco",
    maintainer_email="30201929+goldjunge91@users.noreply.github.com",
    description="Bringup for my_steel robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [],
    },
)
