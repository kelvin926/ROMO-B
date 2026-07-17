from setuptools import find_packages, setup

package_name = "romo_b_waypoints"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Jang HyunSeo",
    maintainer_email="futile-nutmegs0d@icloud.com",
    description="ROMO-B continuous waypoint manager",
    license="Proprietary",
    entry_points={"console_scripts": ["waypoint_manager = romo_b_waypoints.node:main"]},
)
