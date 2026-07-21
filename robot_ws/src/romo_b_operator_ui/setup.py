from glob import glob
from setuptools import find_packages, setup


package_name = "romo_b_operator_ui"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/web_dist",
            glob(package_name + "/web_dist/*.*"),
        ),
        (
            "share/" + package_name + "/web_dist/assets",
            glob(package_name + "/web_dist/assets/*"),
        ),
    ],
    install_requires=["setuptools", "Flask"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Jang HyunSeo",
    maintainer_email="futile-nutmegs0d@icloud.com",
    description="ROMO-B ROS 2 operator console",
    license="Proprietary",
    entry_points={"console_scripts": ["operator_ui = romo_b_operator_ui.server:main"]},
)
