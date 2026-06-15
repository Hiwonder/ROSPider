import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_controller = app.lidar_controller:main',
            'line_following = app.line_following:main',
            'intelligent_kick = app.intelligent_kick:main',
            'hand_trajectory = app.hand_trajectory:main',
            'hand_gesture = app.hand_gesture:main',
            'self_balancing = app.self_balancing:main',
            'perform_actions = app.perform_actions:main',
            'object_tracking = app.object_tracking:main',

        ],
    },
)
