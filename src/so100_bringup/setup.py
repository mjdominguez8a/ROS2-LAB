from glob import glob
from setuptools import find_packages, setup

package_name = 'so100_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.srdf')),
        ('share/' + package_name + '/config/robots', glob('config/robots/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Course Instructor',
    maintainer_email='instructor@example.edu',
    description='SO-100 bringup (mock / Gazebo / real) and safety tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor = so100_bringup.safety_monitor:main',
            'estop = so100_bringup.estop:main',
            'calibrate = so100_bringup.calibrate:main',
        ],
    },
)
