from setuptools import find_packages, setup

package_name = 'so100_labs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Course Instructor',
    maintainer_email='instructor@example.edu',
    description='Student tasks for the SO-100 arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task1_joint_commands = so100_labs.task1_joint_commands:main',
            'task2_trajectories = so100_labs.task2_trajectories:main',
            'task3_forward_kinematics = so100_labs.task3_forward_kinematics:main',
            'task4_inverse_kinematics = so100_labs.task4_inverse_kinematics:main',
            'task5_cartesian = so100_labs.task5_cartesian:main',
            'task6_pid_control = so100_labs.task6_pid_control:main',
            'task6_pid_tune = so100_labs.task6_pid_control:tune',
            'task6_moveit_pid = so100_labs.moveit_pid:main',
            'servo_sag_sim = so100_labs.sag_sim:main',
            'bonus_moveit_goals = so100_labs.bonus_moveit_goals:main',
        ],
    },
)
