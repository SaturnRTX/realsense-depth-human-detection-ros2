from setuptools import setup

package_name = 'intel_depth_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/realsense_yolo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Intel RealSense D456 + YOLOv11 human detection and distance estimation (ROS2 python)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'realsense_node = intel_depth_detection.realsense_node:main'
        ],
    },
)
