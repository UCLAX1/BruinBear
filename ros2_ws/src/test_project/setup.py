from setuptools import find_packages, setup

package_name = 'test_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'talker = test_project.publisher_member_function:main',
            'listener = test_project.subscriber_member_function:main',
            'service = test_project.service_member_function:main',
            'client = test_project.client_member_function:main'
        ],
    },
)
