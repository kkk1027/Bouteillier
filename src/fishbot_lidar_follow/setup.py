from glob import glob

from setuptools import find_packages, setup

package_name = 'fishbot_lidar_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kbf',
    maintainer_email='kbf@example.com',
    description='单线激光雷达目标跟随节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_follow=fishbot_lidar_follow.lidar_follow:main',
            'lidar_follow_sim=fishbot_lidar_follow.sim_node:main',
        ],
    },
)
