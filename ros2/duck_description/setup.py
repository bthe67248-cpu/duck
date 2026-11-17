from glob import glob
from setuptools import setup

package_name = "duck_description"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/meshes", glob("meshes/*")),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
        ("share/" + package_name + "/mjcf", glob("mjcf/*.xml")),
        ("share/" + package_name + "/mjcf/assets", glob("mjcf/assets/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["duck_mujoco_sim = duck_description.duck_mujoco_sim:main"]},
)
