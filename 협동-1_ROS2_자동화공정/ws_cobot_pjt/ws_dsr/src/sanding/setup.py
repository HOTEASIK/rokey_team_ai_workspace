from setuptools import find_packages, setup

package_name = 'sanding'

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
    maintainer='BL',
    maintainer_email='mail@mail.com',
    description='M0609 + RG2 tool task (move -> pick tool -> work)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tool_task_server = sanding.tool_task_action_server:main',
            'tool_task_client = sanding.tool_task_client:main',
            'get_pose = sanding.get_pose:main',
        ],
    },
)
