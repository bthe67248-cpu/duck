from glob import glob
from setuptools import setup

package_name = "duck_nav"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
        ("share/" + package_name + "/models", glob("models/*.onnx")),
        ("share/" + package_name + "/mjcf", glob("mjcf/*.xml") + glob("mjcf/*.json")),
        ("share/" + package_name + "/mjcf/assets", glob("mjcf/assets/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "world_sim = duck_nav.world_sim:main",
        "slam_node = duck_nav.slam_node:main",
        "yolo_node = duck_nav.yolo_node:main",
        "nav_node = duck_nav.nav_node:main",
        "score_node = duck_nav.score_node:main",
    ]},
)
