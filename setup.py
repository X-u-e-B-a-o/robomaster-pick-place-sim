from setuptools import setup, find_packages
from glob import glob

package_name = 'robomaster_pick_place_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'pick_place_moveit = robomaster_pick_place_sim.pick_place_moveit:main',
            'calibrate_pose = robomaster_pick_place_sim.calibrate_pose:main',
            'test_joints = robomaster_pick_place_sim.test_joints:main',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LpTriX',
    maintainer_email='LpTriX@users.noreply.github.com',
    description='RoboMaster EP pick and place Gazebo simulation',
    license='MIT',
)
