from setuptools import find_packages, setup
import os # <--- Added import
from glob import glob # <--- Added import

package_name = 'blueboat_sensors'

setup(
    name=package_name,
    version='0.0.1', # <--- Updated version slightly
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- This line installs launch files ---
   
                # --- This line installs launch files ---
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))) #<-- CORRECTED LINE
    ],
    # --- numpy IS NEEDED FOR LIDAR NODE ---
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='bb',
    maintainer_email='bb@todo.todo',
    description='Placeholder sensor nodes for BlueBoat',
    license='Apache License 2.0',
    tests_require=['pytest'],
    # --- THESE ENTRY POINTS ARE NEEDED TO RUN THE PYTHON NODES ---
    entry_points={
        'console_scripts': [
            'placeholder_imu_node = blueboat_sensors.placeholder_imu_node:main',
            'placeholder_gps_node = blueboat_sensors.placeholder_gps_node:main',
            'placeholder_lidar_node = blueboat_sensors.placeholder_lidar_node:main',
        ],
    },
)
