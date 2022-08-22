from setuptools import setup

package_name = 'era_5g_action_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='adrian.lendinez@outlook.com',
    description='ROS 2 Action client interface for 5g-era middleware',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ActionClientNode = era_5g_action_client.ActionClientNode:main'
        ],
    },
)
