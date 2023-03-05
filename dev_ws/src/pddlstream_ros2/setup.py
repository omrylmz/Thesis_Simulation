from tabnanny import verbose
from setuptools import setup, find_packages
from pathlib import Path


package_name = 'pddlstream_ros2'
file_dir = Path(__file__).resolve().parent

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages("pddlstream"),
    package_dir={"": "pddlstream"},
    package_data={"": ["pddlstream/image/construction.png"]},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', ['launch/pddlstream_examples.launch.py']),
        ('share/' + package_name + '/launch', [str(p.relative_to(file_dir)) for p in file_dir.rglob("*.launch.py")]),
        ("lib/python3.10/site-packages/images", [str(p.relative_to(file_dir)) for p in file_dir.rglob("pddlstream/images/*.*")]),
        # ("lib/python3.10/site-packages/downward", [str(p.relative_to(file_dir)) for p in file_dir.rglob("pddlstream/downward/*")]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omer',
    maintainer_email='omeryilmaz991@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "blocksworld = examples.blocksworld.run_derived:main",
            "warehouse_application = warehouse_application.warehouse_application:main",
            "path_planner_test = warehouse_application.path_planner_test:main"
        ],
    },
)
