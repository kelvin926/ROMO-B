from glob import glob
from setuptools import find_packages, setup


package_name = "romo_b_autoware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jang HyunSeo",
    maintainer_email="futile-nutmegs0d@icloud.com",
    description="Autoware Universe integration for ROMO-B.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kinematic_bridge = romo_b_autoware.kinematic_bridge:main",
            "localization_interface = romo_b_autoware.localization_interface:main",
            "object_tracker = romo_b_autoware.object_tracker:main",
            "occupancy_grid_republisher = romo_b_autoware.occupancy_grid_republisher:main",
            "speed_limit_guard = romo_b_autoware.speed_limit_guard:main",
            "trajectory_follower = romo_b_autoware.trajectory_follower:main",
            "vector_map_startup_guard = romo_b_autoware.vector_map_startup_guard:main",
            "vehicle_interface = romo_b_autoware.vehicle_interface:main",
        ],
    },
)
