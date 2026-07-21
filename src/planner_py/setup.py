from setuptools import find_packages, setup

package_name = 'planner_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup Team',
    maintainer_email='jingyang040306@gmail.com',
    description=(
        'Freshman-editable Python global planner: a standalone action '
        'server implementing nav2_msgs/action/ComputePathToPose.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compute_path_to_pose_server = '
            'planner_py.compute_path_to_pose_server:main',
        ],
    },
)
