from setuptools import find_packages, setup

package_name = 'dram_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'path_planner = dram_plan.path_planner:main',
            'path_planner_reserved = dram_plan.path_planner_reserved:main',
            'resource_allocator = dram_plan.resource_allocator:main',
            'command_to_fastapi_service = dram_plan.command_to_fastapi_service:main',
            'resource_manager = dram_plan.resource_manager:main',
        ],
    },
)
