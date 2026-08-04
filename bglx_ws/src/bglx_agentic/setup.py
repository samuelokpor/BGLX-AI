from setuptools import setup

package_name = 'bglx_agentic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Okpor',
    maintainer_email='samuel@bglx.ai',
    description='LLM agent layer for the BGLX AI delivery tricycle.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'recovery = bglx_agentic.recovery_node:main',
            'inspect = bglx_agentic.robot_tools:main',
            'record_landmark = bglx_agentic.robot_tools:record_landmark',
            'agent = bglx_agentic.agent_loop:main',
        ],
    },
)
