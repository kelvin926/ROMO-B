from setuptools import find_packages, setup

package_name = "romo_b_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/simulation.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Jang HyunSeo",
    maintainer_email="futile-nutmegs0d@icloud.com",
    description="PTY-based ROMO-B PCU simulator",
    license="Proprietary",
    entry_points={"console_scripts": ["pcu_simulator = romo_b_sim.pcu_simulator:main"]},
)
