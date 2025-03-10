from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stinger_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all folders.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishy',
    maintainer_email='seantfish@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera-node = stinger_bringup.camera:main',
            'gps-node = stinger_bringup.gps:main',
            'imu-node = stinger_bringup.imu:main',
            'motor-node = stinger_bringup.motor:main',
            'filter-node = stinger_bringup.filter_scan:main',
        ],
    },
)
