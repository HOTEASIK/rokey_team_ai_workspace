from glob import glob
from setuptools import find_packages, setup

package_name = 'rokey1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ],
    package_data={'hmi': ['web/*']},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BL',
    maintainer_email='mail@mail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move = rokey1.move:main',
            'grip_test = rokey1.grip_test:main',
            'homeset = rokey1.homeset:main',
            'hmi_server = hmi.hmi_server:main',
            'wall_size = navigation.wall_size:main',
            'wall_navigation = navigation.wall_navigation:main',
            'table_navigation = navigation.table_navigation:main',
            'wall_modeling = navigation.wall_modeling:main',
            'wall_size_rviz = navigation.wall_size_rviz:main',
            'navigation_rviz_sim = navigation.rviz_sim_node:main',
            'tool_task_server = sanding.tool_task_action_server:main',
            'tool_task_client = sanding.tool_task_client:main',
            'sanding_server = sanding.sanding_server:main',
            'sanding_client = sanding.sanding_client:main',
            'initialize_device = putty_demo.initialize_device:main',
            'pickup_spatula = putty_demo.pickup_spatula:main',
            'scoop_putty = putty_demo.scoop_putty:main',
            'apply_putty_plane = putty_demo.apply_putty_plane:main',
            'return_spatula = putty_demo.return_spatula:main',
        ],
    },
)
