import os
from glob import glob

from setuptools import setup


package_name = "black_stand"


def data_files_from(directory: str) -> list[tuple[str, list[str]]]:
    """将 launch 和 description 下的所有文件安装至包 share 目录。"""
    files_by_destination: dict[str, list[str]] = {}
    for path in glob(os.path.join(directory, "**", "*"), recursive=True):
        if not os.path.isfile(path):
            continue
        destination = os.path.join("share", package_name, os.path.dirname(path))
        files_by_destination.setdefault(destination, []).append(path)
    return list(files_by_destination.items())


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        *data_files_from("launch"),
        *data_files_from("description"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wujuncheng",
    maintainer_email="wujuncheng@todo.todo",
    description="MuJoCo 四足机器人站立仿真与控制节点。",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "stand_mujoco_sim = black_stand.stand_mujoco_sim_node:main",
            "stand_controller = black_stand.stand_controller_node:main",
        ],
    },
)
