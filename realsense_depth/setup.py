from setuptools import find_packages, setup

package_name = 'realsense_depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyrealsense2'],
    zip_safe=True,
    maintainer='sara',
    maintainer_email='saturnvdt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense = realsense_depth.realsense:main'
        ],
    },
)
