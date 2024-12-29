from setuptools import setup
import os
from glob import glob

package_name = 'turtlesim_fuel_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'src'), glob('src/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Turtlesim simulation with fuel management',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fuel_manager = src.fuel_manager:main',
            'fuel_visualizer = src.fuel_visualizer:main',
        ],
    },
)

