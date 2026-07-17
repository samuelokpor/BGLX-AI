import os
from glob import glob
from setuptools import setup

package_name = 'bglx_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel',
    maintainer_email='samuel@bglxai.com',
    description='BGLX autonomous cmd_vel loop: waypoint following with lidar obstacle stop.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = bglx_autonomy.waypoint_follower:main',
        ],
    },
)
