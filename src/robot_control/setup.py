import os
from glob import glob
from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Standard ROS2 files needed for installation
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[py|xml]'))),
        
        # NOTE: If you had configuration files (like YAML), they would be listed here too.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishrut',
    maintainer_email='vishrutkakadiya99@gmail.com',
    description='ROS2 Python node for executing the pick-and-place task logic using MoveIt2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    # Define your executable Python nodes here
    entry_points={
        'console_scripts': [
            # The format is: 'executable_name = package_name.python_file_name:main_function'
            'pick_and_place_node = robot_control.pick_and_place_node:main'
        ],
    },
)
