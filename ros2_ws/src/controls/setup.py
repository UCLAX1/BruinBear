from setuptools import find_packages, setup
from glob import glob
import sys

package_name = 'controls'
sys.path.insert(0, "/home/sara/venv/lib/python3.10/site-packages")


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))  # Include all launch files
    ],
    install_requires=['setuptools',
                      'mediapipe',
                      ],
    python_requires='>=3.6',
    zip_safe=True,
    maintainer='akhilesh',
    maintainer_email='akhilesh.basetty@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj = controls.foot_traj_follower:main',
            'sim = controls.sim:main',
            'fsm = controls.gait_publisher:main'
        ],
    },
)
