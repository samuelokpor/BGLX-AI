import os
from glob import glob
from setuptools import setup
package_name = 'bglx_exploration'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Samuel', maintainer_email='samuel@bglxai.com',
    description='Info-gain / A*-path-cost frontier exploration (Nav2 port of dimos PR #2830).',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'frontier_explorer = bglx_exploration.frontier_explorer:main',
    ]},
)
