from setuptools import find_packages, setup

package_name = 'fishbot_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share',package_name,'action'),glob('action/*action'))
    ],
    install_requires=['setuptools',
                      'rclpy',  # ROS 2 Python 客户端库
                      'geometry_msgs',  # 几何消息类型
                      'std_msgs',  # 标准消息类型
                      'std_srvs',  # 标准服务类型
                      'nav2_msgs',  # Nav2 消息类型
                      'nav2_common',  # Nav2 公共库
                        ],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='87068644+fishros@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "init_robot_pose=fishbot_application.init_robot_pose:main",
            "get_robot_pose=fishbot_application.get_robot_pose:main",
            "nav_to_pose=fishbot_application.nav_to_pose:main",
            "waypoint_follower=fishbot_application.waypoint_follower:main",

        ],
    },
)
