from setuptools import setup

package_name = "warebot_map_merger"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/map_merger.launch.py"
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osama",
    maintainer_email="osama@example.com",
    description="Centralized multi-robot map merger with MQTT output.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "map_merger = warebot_map_merger.map_merger_node:main",
        ],
    },
)
