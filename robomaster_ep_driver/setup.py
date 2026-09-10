import glob
import os

from setuptools import setup

package_name = 'robomaster_ep_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davidliu',
    maintainer_email='you@example.com',
    description='RoboMaster EP 真机驱动 (ros2_control 兼容接口 + 官方 SDK 封装)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ep_arm_driver = robomaster_ep_driver.ep_arm_driver:main',
            'calibrate_real = robomaster_ep_driver.calibrate_real:main',
        ],
    },
)
