"""가짜 DB (MySQL 대신 메모리 + JSON 파일). utils.db.DB 와 같은 메서드만 가진다.

ProcessRunner 가 DB 로 보내는 데이터(run / wall / gap / task_log)를 그대로 받아 리스트에 쌓는다.
  - 오프라인 테스트: test/test_full_process.py
  - sodvir 테스트 : hmi_server -p fake_db:=<json 경로>  → 쓸 때마다 그 파일에 저장 (실제 MySQL 은 안 건드림)
"""
import json


class FakeDB:
    def __init__(self, path=None):
        self.path = path              # 주면 쓸 때마다 JSON 저장
        self.runs, self.walls, self.gaps, self.task_log = [], [], [], []

    def _save(self):
        if self.path:
            self.dump(self.path)

    # ── run ──
    def abort_open_runs(self):
        return 0

    def start_run(self, task, gap_id, mode, purpose, material, note, code_version):
        self.runs.append({'run_id': len(self.runs) + 1, 'process': task, 'gap_id': gap_id, 'mode': mode,
                          'purpose': purpose, 'material': material, 'note': note, 'result': None})
        self._save()
        return len(self.runs)

    def end_run(self, run_id, result, data_check):
        self.runs[run_id - 1].update(result=result, data_check=data_check)
        self._save()

    def check_run(self, run_id, task, explored=True, need_gap=True):
        missing = []
        if not any(r['run_id'] == run_id for r in self.task_log):
            missing.append('task_log 없음')
        if task in ('ALL', 'EXPLORE') and explored and not any(w['run_id'] == run_id for w in self.walls):
            missing.append('wall/gap 없음')
        return ', '.join(missing) or 'OK'

    # ── 결과 ──
    def gap_points(self, gap_id):
        for g in self.gaps:
            if g['gap_id'] == gap_id:
                wall = next(w for w in self.walls if w['wall_id'] == g['wall_id'])
                return g['points'], wall['points']
        return None

    def latest_wall(self):
        if not self.walls:
            return None
        w = self.walls[-1]
        gaps = [(g['gap_id'], list(g['points'])) for g in self.gaps if g['wall_id'] == w['wall_id']]
        return w['wall_id'], w['run_id'], list(w['points']), gaps

    def insert_wall(self, run_id, w):
        self.walls.append({'wall_id': len(self.walls) + 1, 'run_id': run_id, 'points': list(w)})
        self._save()
        return len(self.walls)

    def insert_gap(self, wall_id, g):
        self.gaps.append({'gap_id': len(self.gaps) + 1, 'wall_id': wall_id, 'points': list(g)})
        self._save()
        return len(self.gaps)

    def log_step(self, run_id, task_code, gap_id, attempt, fb):
        self.task_log.append({'run_id': run_id, 'task_code': task_code, 'gap_id': gap_id, 'step': fb.step,
                              'success': bool(fb.step_success), 'error_code': fb.error_code or None,
                              'fail_reason': fb.fail_reason or '', 'params': fb.params_json or ''})
        self._save()

    def dump(self, path):
        with open(path, 'w') as f:
            json.dump({'run': self.runs, 'wall': self.walls, 'gap': self.gaps, 'task_log': self.task_log},
                      f, ensure_ascii=False, indent=2)
