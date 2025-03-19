from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_kinova_transport'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Add CSV data files
        (os.path.join('share', package_name, 'data'), glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pojuilin',
    maintainer_email='pojuilin@uw.edu',
    description='Integration of Kinova Gen3 Lite arm with TurtleBot for pick and place and transport tasks',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_kinova_transport = turtlebot_kinova_transport.turtlebot_kinova_transport:main',
        ],
    },
)

