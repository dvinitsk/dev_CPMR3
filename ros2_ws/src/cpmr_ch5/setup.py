from glob import glob
import os
from setuptools import setup

package_name = 'cpmr_ch5'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='CPMR 3rd Edition Chpater 5',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_target = cpmr_ch5.aruco_target:main',
        ],
    },
)
