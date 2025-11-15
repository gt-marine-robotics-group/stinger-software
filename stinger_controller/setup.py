from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stinger_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='jeff300fang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_controller = stinger_controller.throttle_controller:main',
            'velocity_controller = stinger_controller.velocity_controller:main',
            'acceleration_controller = stinger_controller.acceleration_controller:main',
            'position_controller = stinger_controller.position_controller:main',
            'direct_thrust_controller = stinger_controller.direct_thrust_controller:main'
        ],
    },
)
