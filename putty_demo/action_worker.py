"""액션 서버의 격리 실행부. ROS Action 서버와 기존 노드의 spin/shutdown을 분리한다.

직접 실행용이 아니다. 서버는 shell 없이 이 모듈을 자식 프로세스로 실행하며,
기존 노드가 끝나기 전에도 별도 ROS 노드에서 피드백과 취소를 처리한다.
"""

import importlib
import sys


MODULES = {
    'initialize': 'initialize_device',
    'pickup': 'pickup_spatula',
    'scoop': 'scoop_putty',
    'load': 'load_putty',
    'apply': 'apply_putty_plane',
    'return': 'return_spatula',
}


def main():
    task = sys.argv[1]
    module = importlib.import_module(f'putty_demo.{MODULES[task]}')
    sys.argv = [MODULES[task], *sys.argv[2:]]
    return module.main()


if __name__ == '__main__':
    raise SystemExit(main())
