from setuptools import setup


package_name = "px4_python_sitl_ros2_example"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/px4_lockstep_ros2.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "pymavlink"],
    zip_safe=True,
    maintainer="PX4 Python SITL Contributors",
    maintainer_email="noreply@example.com",
    description="ROS2 lockstep integration example for px4-python-sitl",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "px4_lockstep_ros2_node = px4_python_sitl_ros2_example.lockstep_ros2_node:main",
        ],
    },
)
