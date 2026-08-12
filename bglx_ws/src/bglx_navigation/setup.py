import os
from glob import glob
from setuptools import setup

package_name = 'bglx_navigation'

# DEST: bglx_ws/src/bglx_navigation/setup.py
# Track A change: register the cmd_vel_limiter console script.

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'bt'), glob('config/bt/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel',
    maintainer_email='samuel@bglxai.com',
    description='Nav2 bringup and params for the BGLX autonomous e-trike.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_limiter = bglx_navigation.cmd_vel_limiter:main',
        ],
    },
)