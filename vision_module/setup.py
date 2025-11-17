from setuptools import setup

package_name = 'vision_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'opencv-python',
        'torch',
        'torchvision',
        'numpy',
        'rclpy',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS 2 package for object detection using YOLO.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_module.vision_node:main',
        ],
    },
)