from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'f1tenth_imu'


setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.json')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='zaid al-qaisi',

    maintainer_email='zqaisi2004@gmail.com',

    description=(
        'BMI160 IMU driver for the '
        'UJ F1TENTH autonomous race car'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            (
                'bmi160_node = '
                'f1tenth_imu.bmi160_node:main'
            ),
        ],
    },
)
