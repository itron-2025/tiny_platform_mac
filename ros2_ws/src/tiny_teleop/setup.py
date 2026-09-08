from glob import glob

from setuptools import setup

package_name = 'tiny_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itron',
    maintainer_email='itron2025@gmail.com',
    description='Pro Controller teleoperation for the tiny_platform_mac mecanum chassis.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_teleop = tiny_teleop.joy_teleop:main',
            'joy_probe  = tiny_teleop.joy_probe:main',
            'face_node  = tiny_teleop.face_node:main',
        ],
    },
)
