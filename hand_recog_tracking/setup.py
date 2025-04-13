from setuptools import find_packages, setup

package_name = 'hand_recog_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'mediapipe',
                      ],
    python_requires='>=3.6',
    zip_safe=True,
    maintainer='sara',
    maintainer_email='sara@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = hand_recog_tracking.handTracking:main',
            'poslistener = hand_recog_tracking.handPosSub:main',
            'recoglistener = hand_recog_tracking.handRecogSub:main',
        ],
    },
)
