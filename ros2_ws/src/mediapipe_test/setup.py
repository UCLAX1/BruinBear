from setuptools import find_packages, setup

package_name = 'mediapipe_test'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'mediapipe', 'matplotlib',],
    zip_safe=True,
    maintainer='sara',
    maintainer_email='sara@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipeTest = mediapipe_test.mediapipeTest:main',
        ],
    },
)
