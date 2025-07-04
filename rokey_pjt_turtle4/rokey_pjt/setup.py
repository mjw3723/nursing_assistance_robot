from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/srv', glob('srv/*.srv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools',
                      'rokey_interfaces'],
    zip_safe=True,
    maintainer='hongha',
    maintainer_email='hongha0704@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '2_1_capture_image = rokey_pjt.2_1_capture_image:main',
            'yolo_detect = rokey_pjt.yolo_detect:main',
            'depth_checker = rokey_pjt.depth_checker:main',
            'depth_checker_click = rokey_pjt.depth_checker_click:main',
            'yolo_depth_checker = rokey_pjt.yolo_depth_checker:main',
            'tf_trans = rokey_pjt.4_tb4_tf_transform:main',
            'object_xyz_marker = rokey_pjt.object_xyz_marker:main',
            'tf = rokey_pjt.tf:main',
            'vision = rokey_pjt.vision:main',
            'object_xyz_marker_service = rokey_pjt.object_xyz_marker_service:main',
            'service_test = rokey_pjt.service_test:main',
            'yolo_once_client = rokey_pjt.yolo_once_client:main',
            'yolo_once = rokey_pjt.yolo_once:main',
            'robot4_vision = rokey_pjt.robot4_vision:main',
            '1_webcam_publisher = rokey_pjt.1_webcam_publisher:main',
            'main_controller = rokey_pjt.main_controller:main',
            'main_controller_server_test = rokey_pjt.main_controller_server_test:main',
            'main_controller_client_test = rokey_pjt.main_controller_client_test:main',
            'robot4_navigation = rokey_pjt.robot4_navigation:main',
            '0629test = rokey_pjt.0629test:main',
            'fake_node = rokey_pjt.fake_node:main',
        ],
    },
)
