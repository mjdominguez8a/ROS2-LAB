from glob import glob
from setuptools import find_packages, setup

package_name = 'lab_starter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Course Instructor',
    maintainer_email='instructor@example.edu',
    description='Starter nodes for the browser-based ROS 2 student lab.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = lab_starter.talker:main',
            'listener = lab_starter.listener:main',
            'turtle_square = lab_starter.turtle_square:main',
            'turtle_circle = lab_starter.turtle_circle:main',
        ],
    },
)
