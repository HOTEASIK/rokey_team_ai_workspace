# new_bringup.launch.py 새로운 파일의 launch 파일

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────
    # (virtual) ros2 launch m0609_rg2_bringup bringup.launch.py
    # (real)    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
    args = [
        DeclareLaunchArgument('name',       default_value = 'dsr01',          description = 'NAME_SPACE'     ),
        DeclareLaunchArgument('host',       default_value = '127.0.0.1',      description = 'ROBOT_IP'       ),
        DeclareLaunchArgument('port',       default_value = '12345',          description = 'ROBOT_PORT'     ),
        DeclareLaunchArgument('mode',       default_value = 'virtual',        description = 'OPERATION MODE' ),
        DeclareLaunchArgument('model',      default_value = 'm0609',          description = 'ROBOT_MODEL'    ),
        DeclareLaunchArgument('color',      default_value = 'white',          description = 'ROBOT_COLOR'    ),
        DeclareLaunchArgument('gui',        default_value = 'false',          description = 'Start RViz2'    ),
        DeclareLaunchArgument('gz',         default_value = 'false',          description = 'USE GAZEBO SIM' ),
        DeclareLaunchArgument('rt_host',    default_value = '192.168.1.50',   description = 'ROBOT_RT_IP'    ),
        DeclareLaunchArgument('remap_tf',   default_value = 'false',          description = 'REMAP TF'       ),
    ]

    is_real    = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'real'"])
    is_virtual = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])

    # ── [virtual] DRCF 에뮬레이터 (Docker) ───────────────────────────
    # virtual mode 시 localhost에서 DRCF 에뮬레이터를 실행
    # 종료 시 terminate_drcf()로 컨테이너 자동 정리
    run_emulator_node = Node(
        package    = 'dsr_bringup2',
        executable = 'run_emulator',
        namespace  = LaunchConfiguration('name'),
        parameters = [
            {"name":    LaunchConfiguration('name')  }, 
            {"rate":    100         },
            {"standby": 5000        },
            {"command": True        },
            {"host":    LaunchConfiguration('host')  },
            {"port":    LaunchConfiguration('port')  },
            {"mode":    LaunchConfiguration('mode')  },
            {"model":   LaunchConfiguration('model') },
            {"gripper": "none"      },
            {"mobile":  "none"      },
            {"rt_host":  LaunchConfiguration('rt_host') },

        ],
        condition=IfCondition(is_virtual),
        output='screen',
    )

    # ── [virtual] 이전 run 잔여 에뮬레이터 컨테이너 정리 ──────────────
    # run_drcf.sh의 중복 컨테이너 체크는 'docker ps -q'(running 상태만) 기반이라
    # Exited 상태로 남은 --rm 미정리 컨테이너를 놓친다. 그 경우 다음 bringup의
    # 'docker run --name dsr01_emulator'가 이름 충돌로 실패 → 에뮬레이터 미기동 →
    # ros2_control 하드웨어 init 실패로 연쇄. run_emulator 시작 전 동명 컨테이너를
    # 강제 제거해 launch를 idempotent하게 만든다.
    emulator_cleanup = ExecuteProcess(
        cmd=["bash", "-c", "docker rm -f dsr01_emulator 2>/dev/null || true"],
        condition=IfCondition(is_virtual),
        output="log",
    )

    start_emulator = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action = emulator_cleanup,
            on_exit       = [run_emulator_node],
        ),
    )

    # ── 커스텀 URDF (M0609 + RG2) ────────────────────────────────────
    xacro_file = os.path.join(
        get_package_share_directory("m0609_rg2_bringup"),
        "xacro",
        "my_m0609_rg2.urdf.xacro",
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("m0609_rg2_bringup"), "rviz", "default.rviz"
    )

    # ── [real] Doosan URDF (ros2_control 하드웨어 인터페이스용) ───────

    doosan_robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' name:=', LaunchConfiguration('name'),
        ' host:=', LaunchConfiguration('host'),
        ' port:=', LaunchConfiguration('port'),
        ' mode:=', LaunchConfiguration('mode'),
        ' model:=', LaunchConfiguration('model'),
        ' update_rate:=100',
    ])

    # ── ros2_control_node (virtual/real 공통) ─────────────────────────
    # virtual: run_emulator_node가 먼저 Docker DRCF를 띄우고,
    #          dsr_hw_interface2 재시도 루프(0.5s × 20회)로 연결
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=LaunchConfiguration('name'),
        parameters=[
            {'robot_description': ParameterValue(doosan_robot_description, value_type=str)},
            {'update_rate': 100},
            PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml']),
        ],
        output='both',
    )

    # ── joint_state_broadcaster (/dsr01/joint_states 퍼블리시) ────────
    joint_state_broadcaster_spawner = Node(
        package    = 'controller_manager',
        executable = 'spawner',
        namespace  = LaunchConfiguration('name'),
        arguments  = ['joint_state_broadcaster', '-c', 'controller_manager'],
    )

    # ── dsr_controller2 (motion service 등록) ─────────────────────────
    robot_controller_spawner = Node(
        package    = 'controller_manager',
        executable = 'spawner',
        namespace  = LaunchConfiguration('name'),
        arguments  = ['dsr_controller2', '-c', 'controller_manager'],
    )
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit       = [robot_controller_spawner],
        ),
    )

    # ── [virtual] GripperVirtualNode (/onrobot/sendCommand 서비스) ───
    is_virtual_gripper = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])
    gripper_virtual_node = Node(
        package    = 'm0609_rg2_bringup',
        executable = 'gripper_virtual_node.py',
        namespace  = LaunchConfiguration("name"),
        name       = 'gripper_virtual_node',

        condition  = IfCondition(is_virtual_gripper),
        output     = 'screen',
    )

    # ── [real] OnRobot RG2 드라이버 ──────────────────────────────────
    # /joint_states → /onrobot_joint_states 로 remap (joint_state_publisher와 충돌 방지)
    onrobot_driver = Node(
        package    = "onrobot_rg_control",
        executable = "OnRobotRGControllerServer",
        namespace  = LaunchConfiguration("name"),
        name       = "OnRobotRGControllerServer",
        output     = "screen",
        parameters = [
            {
                "/onrobot/control"     : "modbus",
                "/onrobot/ip"          : "192.168.1.1",
                "/onrobot/port"        : 502,
                "/onrobot/changer_addr": 65,
                "/onrobot/gripper"     : "rg2",
                "/onrobot/offset"      : 5,
            }
        ],
        remappings=[("/joint_states", "/onrobot_joint_states")],
        condition=IfCondition(is_real),
    )

    # ── [real] 그리퍼 너비 → rg2_finger_joint 변환 노드 ──────────────
    # OnRobotRGInput.ggwd → /gripper_joint_states (rg2_finger_joint)
    rg2_joint_state_republisher = Node(
        package    = 'm0609_rg2_bringup',
        executable = 'rg2_joint_state_republisher.py',
        namespace  = LaunchConfiguration("name"),
        name       = 'rg2_joint_state_republisher',

        condition  = IfCondition(is_real),
        output     = 'screen',
    )

    # ── joint_state_publisher (virtual/real 공통) ─────────────────────
    # dsr01/joint_states와 /gripper_joint_states 통합 토픽
    # virtual 환경에서는 /gripper_joint_states 없음(DRCF 문제) → gripper joint 0으로 채워짐

    joint_state_merger = Node(
        package="m0609_rg2_bringup",
        executable="joint_state_merger.py",
        namespace=LaunchConfiguration("name"),
        name="joint_state_merger",
        output="screen",
        parameters=[
            {"robot_joint_topic": ["/", LaunchConfiguration("name"), "/robot_joint_states"]}
        ],
    )

    # ── robot_state_pub_node ─────────────────────────────────────────
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        remappings=[("joint_states", "/joint_states")],
        parameters=[
            {
               "robot_description": ParameterValue(doosan_robot_description, value_type=str)
            }
        ],
    )

    # ── Static TF (world → base_link) ────────────────────────────────
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=LaunchConfiguration("name"),
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        namespace  = LaunchConfiguration('name'),
        name       = 'rviz2',
        output     = 'log',
        arguments  = ['-d', rviz_config_file],
    )

    return LaunchDescription(args + [
        emulator_cleanup,
        start_emulator,
        gripper_virtual_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_controller,
        onrobot_driver,
        rg2_joint_state_republisher,
        joint_state_merger,
        robot_state_pub_node,
        static_tf,
        rviz_node,
    ])
