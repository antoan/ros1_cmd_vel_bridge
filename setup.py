from setuptools import find_packages, setup

package_name = 'ros1_cmd_vel_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cmd_vel_forwarder.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='antoan.bekele@gmail.com',
    description='A ROS 2 node to forward cmd_vel messages to a ROS 1 system via rosbridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_forwarder = ros1_cmd_vel_bridge.cmd_vel_forwarder:main'
        ],
    },
)
