from glob import glob

from setuptools import setup

package_name = "glosa_speed_advisor"

setup(
    name=package_name,
    version="0.4.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anil",
    maintainer_email="anillselcuk1@gmail.com",
    description="GLOSA speed advisor for Autoware (fixed-time signals)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "glosa_node = glosa_speed_advisor.glosa_node:main",
            "fake_spat_node = glosa_speed_advisor.fake_spat_node:main",
            "stop_line_helper = glosa_speed_advisor.stop_line_helper:main",
            "list_traffic_lights = glosa_speed_advisor.lanelet_osm:main",
        ],
    },
)
