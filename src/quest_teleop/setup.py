import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'quest_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Aaron Reyes',
    maintainer_email='areyes@stevens.edu',
    description='Meta Quest 3 controller pose streaming (WebXR) and UR5 teleoperation',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'quest_bridge = quest_teleop.quest_bridge_node:main',
            'quest_teleop = quest_teleop.teleop_node:main',
            'fake_quest = quest_teleop.fake_quest:main',
            'pose_server = quest_teleop.pose_server:main',
        ],
    },
)
