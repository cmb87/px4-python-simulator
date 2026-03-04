from setuptools import setup


package_name = "webrtc_fpv_ros2_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/webrtc_fpv_bridge.launch.py"]),
    ],
    install_requires=["setuptools", "aiohttp", "aiortc", "numpy"],
    zip_safe=True,
    maintainer="PX4 Python SITL Contributors",
    maintainer_email="noreply@example.com",
    description="WebRTC FPV receiver publishing ROS2 image topics",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "webrtc_fpv_bridge_node = webrtc_fpv_ros2_bridge.webrtc_image_bridge_node:main",
        ],
    },
)
