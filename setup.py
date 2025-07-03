from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'nursing_assistance_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('nursing_assistance_robot/model/*.pb')),
        (os.path.join('share', package_name, 'model'), glob('nursing_assistance_robot/model/*.pbtxt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moon',
    maintainer_email='mjw3723@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_node = nursing_assistance_robot.nav_node:main',
            'yolo_detect_depth = nursing_assistance_robot.yolo_detect_depth:main',
            'request_node = nursing_assistance_robot.request_node:main',
            'response_node = nursing_assistance_robot.response_node:main',
            'depth_node = nursing_assistance_robot.depth_node:main',
            'fake_node = nursing_assistance_robot.fake_node:main',
            'music_node = nursing_assistance_robot.music_node:main',
            'main_controller = nursing_assistance_robot.main_controller:main',
            'fake_patrol_node = nursing_assistance_robot.fake_patrol_node:main',
            'service_node = nursing_assistance_robot.service_node:main',
            'fake_node2 = nursing_assistance_robot.fake_node2:main',
            'nav_node2 = nursing_assistance_robot.nav_node2:main',
            'nav_node3 = nursing_assistance_robot.nav_node3:main',
            'cloud = nursing_assistance_robot.cloud:main',
            'cloud_sub = nursing_assistance_robot.cloud_sub:main',
            'vital = nursing_assistance_robot.vital_check_node2:main',
            'yolo_detect_node1 = nursing_assistance_robot.yolo_detect_node:main',
        ],
    },
)
