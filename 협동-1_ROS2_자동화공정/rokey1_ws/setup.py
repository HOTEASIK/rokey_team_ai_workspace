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
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    # 웹 HMI 정적 파일은 hmi 패키지 데이터로 설치 (hmi_server 가 hmi/web 에서 서빙)
    # ※ data_files 로 넣으면 colcon --symlink-install 에서 새 파일 설치가 실패하는 버그가 있음
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
            'tape_locate = tape.tape_locate:main',
            'tape_peel = tape.tape_peel:main',
            # ===== HMI =====
            'hmi_server = hmi.hmi_server:main',
            # ===== navigation =====
            'wall_tap = navigation.wall_tap:main',
            'explore_server = navigation.explore_server:main',   # /process/explore (wall_tap → DB)
            # ===== putty_demo =====
            'initialize_device = putty_demo.initialize_device:main',
            'pickup_spatula = putty_demo.pickup_spatula:main',
            'scoop_putty = putty_demo.scoop_putty:main',
            'load_putty = putty_demo.load_putty:main',
            'apply_putty_plane = putty_demo.apply_putty_plane:main',
            'return_spatula = putty_demo.return_spatula:main',
            'putty_task_server = putty_demo.putty_task_server:main',
            'putty_task_client = putty_demo.putty_task_client:main',
            # ===== sanding =====
            'tool_task_server = sanding.sanding_server:main',     # HMI [샌딩 액션 서버] 버튼
            'sanding_server = sanding.sanding_server:main',
            'sanding_client = sanding.sanding_client:main',
            # ===== 공정 테스트 =====
            'fake_process = manager.fake_process:main',
        ],
    },
)
