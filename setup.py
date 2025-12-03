from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'mam_eurobot_2026'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
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
            'py_test     = mam_eurobot_2026.py_test:main',
            'key_ctrl    = mam_eurobot_2026.key_ctrl:main',     
            'half_spin   = mam_eurobot_2026.half_spin:main',
            'open_cv_cam = mam_eurobot_2026.open_cv_cam:main',
            'auto_move   = mam_eurobot_2026.auto_move:main', 
        ],
    },
)
