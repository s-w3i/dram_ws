from setuptools import find_packages, setup
from glob import glob

package_name = 'dram_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usern',
    maintainer_email='usern@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_viz = dram_viz.path_viz:main',
            'nav_graph_publisher = dram_viz.nav_graph_publisher:main',
            'nav_graph_visualizer = dram_viz.nav_graph_viz:main',
            'robot_visualizer = dram_viz.robot_viz:main'
        ],
    },
)
