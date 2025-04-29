from setuptools import find_packages, setup
import os  # Ensure this is imported
from glob import glob # Ensure this is imported

package_name = 'blueboat_hardware_interface'

setup(
    name=package_name, version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
 
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bb',
    maintainer_email='bb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'base_controller_node = blueboat_hardware_interface.base_controller_node:main',
        ],
    },
)
