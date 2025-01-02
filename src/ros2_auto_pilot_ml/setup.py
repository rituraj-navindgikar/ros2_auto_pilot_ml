from setuptools import find_packages, setup
import os
package_name = 'ros2_auto_pilot_ml'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rituraj',
    maintainer_email='riturajn1200@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_recorder = ros2_auto_pilot_ml.data_recorder:main',
        ],
    },
)
