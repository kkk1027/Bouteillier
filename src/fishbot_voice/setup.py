from glob import glob
import sys

from setuptools import find_packages, setup

package_name = 'fishbot_voice'

# 当前 Python 版本目录名，用于把包源码直接装进 site-packages
_py_site = 'lib/python%d.%d/site-packages' % sys.version_info[:2]

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
        # ★ 用 data_files 而不是 scripts= 安装启动器。
        #   setuptools 的 develop 模式会把 scripts= 改写成
        #   EASY-INSTALL-DEV-SCRIPT，里面依然有
        #       __import__('pkg_resources').require('fishbot-voice==0.1.0')
        #   仍然依赖元数据 —— 等于没绕开问题。
        #   data_files 则是原样拷贝，真正不碰元数据。
        ('lib/' + package_name, ['scripts/run_voice_control_node.py']),
        # ★★ 关键：把包源码直接装到 site-packages 下。
        #   colcon 的 ament_python + --symlink-install 会用 setuptools
        #   develop 模式，只在 site-packages 放一个 egg-link，真正的包
        #   目录靠 easy-install.pth 加进 sys.path。
        #   但 Python **只对真正的 site-packages 目录处理 .pth 文件**；
        #   通过 PYTHONPATH 加入的目录不会处理。而 ROS 正是用 PYTHONPATH
        #   指向 install/.../site-packages 的 —— 于是 `import fishbot_voice`
        #   在 `ros2 launch` 拉起的子进程里直接 ModuleNotFoundError。
        #   本项目实际踩过：终端里能 import、能启动，launch 里就一直崩。
        #   这里显式把 .py 装进 site-packages，形成真正可导入的包。
        ('%s/%s' % (_py_site, package_name),
         glob(package_name + '/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kbf',
    maintainer_email='kbf@example.com',
    description='语音控制：ASR-01 原始文本事件 -> 底盘运动',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_control_node = fishbot_voice.voice_control_node:main',
        ],
    },
)
