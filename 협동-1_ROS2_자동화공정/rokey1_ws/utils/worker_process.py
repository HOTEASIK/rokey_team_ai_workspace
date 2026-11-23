"""작업 노드를 자식 프로세스로 격리 실행하고 출력/중단을 관리한다 (putty_task_server, explore_server 공통).
로봇 정지는 호출한 서버가 별도로 확인한다."""

import os
import queue
import signal
import subprocess
import threading


class WorkerProcess:
    def __init__(self, command, grasp_confirmed=False):
        self.lines = queue.Queue(maxsize=500)
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, encoding='utf-8', errors='replace', bufsize=1, start_new_session=True,
        )
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        # 실제 확인을 받은 Goal에 한해 기존 scoop의 파지 확인 입력을 전달한다.
        try:
            if grasp_confirmed:
                self.process.stdin.write('y\n')
                self.process.stdin.flush()
        except BrokenPipeError:
            pass
        finally:
            self.process.stdin.close()

    def _read(self):
        try:
            for line in self.process.stdout:
                line = line.rstrip()[-4000:]
                try:
                    self.lines.put_nowait(line)
                except queue.Full:
                    try:
                        self.lines.get_nowait()
                    except queue.Empty:
                        pass
                    self.lines.put_nowait(line)
        finally:
            self.process.stdout.close()

    def drain(self):
        lines = []
        while True:
            try:
                lines.append(self.lines.get_nowait())
            except queue.Empty:
                return lines

    def poll(self):
        return self.process.poll()

    def freeze(self):
        """취소 후 다음 단계 명령을 내지 못하게 먼저 작업 프로세스를 정지한다."""
        if self.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGSTOP)
            except ProcessLookupError:
                pass

    def terminate(self):
        """SIGINT로 finally 정리를 허용한 뒤, 응답이 없으면 프로세스만 강제 종료한다.

        이것만으로 물리 로봇이 정지한 것은 아니다. 호출자는 전후에 MoveStop과
        STANDBY를 확인하고, 필요 시 힘/컴플라이언스 해제도 확인해야 한다.
        """
        if self.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                os.killpg(self.process.pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=2.0)
        self.reader.join(timeout=1.0)

    def kill(self):
        """정리 동작 없이 즉시 종료 (SIGINT 로 끄면 노드가 홈 복귀 같은 동작을 할 수 있을 때)."""
        if self.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait(timeout=2.0)
        self.reader.join(timeout=1.0)
