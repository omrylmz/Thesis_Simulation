import os # Operating system library
from glob import glob # Handles file path names
from setuptools import setup # Facilitates the building of packages

package_name = 'warehouse_robot_spawner'

# Path of the current directory
cur_directory_path = os.path.abspath(os.path.dirname(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Path to the launch file      
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),

        # Path to the world file
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*.world')),

        # Path to the warehouse sdf file
        (os.path.join('share', package_name,'models/small_warehouse/'), glob('./models/small_warehouse/*.sdf') + glob('./models/small_warehouse/*.config')),

        # Path to the mobile robot sdf file
        (os.path.join('share', package_name,'models/pr2/'), glob('./models/pr2/*.sdf') + glob('./models/pr2/*.config')),
        
        # Path to the world file (i.e. warehouse + global environment)
        (os.path.join('share', package_name,'models/'), glob('./worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='focalfossa',
    maintainer_email='focalfossa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'spawn_demo = warehouse_robot_spawner.spawn_demo:main'
        ],
    },
)