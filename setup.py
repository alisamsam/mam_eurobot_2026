from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'mam_eurobot_2026'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        'mam_eurobot_2026',
        'mam_eurobot_2026.Move',
        'mam_eurobot_2026.navigation',
        'mam_eurobot_2026.Perception',
        'mam_eurobot_2026.Task_Planner'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Freddy Liendo',
    maintainer_email='liendomf@univ-smb.fr',
    description='MAM team ROBOT-UN – Eurobot 2026',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'py_test     = mam_eurobot_2026.Move.py_test:main',
            'key_ctrl    = mam_eurobot_2026.Move.key_ctrl:main',     
            'half_spin   = mam_eurobot_2026.Move.half_spin:main',
            'aruco_perception = mam_eurobot_2026.Perception.cam_view:main',
            'auto_move   = mam_eurobot_2026.Move.auto_move:main',
            'test_movement = mam_eurobot_2026.test_movement:main',
            'main_strategy = mam_eurobot_2026.Task_Planner.main_strategy:main',
            'gripper_controller = mam_eurobot_2026.Move.gripper_controller:main',
        ],
    },
)
