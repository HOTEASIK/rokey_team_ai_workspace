#!/usr/bin/env python3
"""wall_tap 파라미터 조합 반복 실험 (Grid Search) + 결과 분석.

tactile_exploration_optimization_plan 4.1 (Grid Search) / 3.3 (다목적 평가 함수) 구현.
각 조합마다 `ros2 run rokey1 wall_tap` 을 repeat 번 실행하고, 결과 JSON 을 모아서
조합별로 치수 오차 / 반복 편차 / 탐색 시간 / 찍기 횟수 / 접촉 지연 / 최대 힘을 비교한다.

[평가 함수]  (조합 사이에서 min-max 정규화 후 가중합, 작을수록 좋음)
  J = w_dim · E_dim + w_rep · σ_dim + w_time · T + w_force · F_peak
    E_dim  : 높이/너비 절대 오차 평균 (정답 true_height/true_width 가 있을 때)
    σ_dim  : 반복 측정 표준편차 (높이/너비 평균) → 재현성
    T      : 탐색 시간 평균 [s]
    F_peak : 찍을 때 최대 힘 평균 [N] (과한 접촉 = 벽/툴 부담)
  실패한 실행이 있는 조합은 순위에서 뒤로 (실패율 표시).

[실행]
  # 에뮬레이터 (로직 검증용. 가상 벽은 이상적이라 힘/임계값 튜닝 결과는 실제와 다름)
  ros2 run rokey1 wall_tap_grid run --mode virtual --grid k_sigma=3,5 --grid step_z=10,20 --repeat 2

  # 실제 로봇 (실제 치수를 자로 재서 넣기)
  ros2 run rokey1 wall_tap_grid run --mode real --grid force=6,8,10 --grid k_sigma=3,5 --repeat 3 \
      --true-height 300 --true-width 500 --set start=350,0,200 --set z_limit=400

  # 이미 모은 결과만 다시 분석 (가중치 바꿔보기)
  ros2 run rokey1 wall_tap_grid analyze ~/wall_scan_data/grid_20261116_1530 --w-time 0.5

  --set 으로 모든 조합에 같은 값을, --grid 로 바꿔가며 실험할 값을 준다 (이름은 wall_tap 파라미터).
  튜닝용 실행과 최종 평가용 실행은 다른 폴더(다른 날/다른 repeat)로 나눠서 비교할 것 (과적합 방지).
"""
import argparse
import csv
import glob
import itertools
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime

import numpy as np


def parse_kv(items, multi):
    out = {}
    for it in items or []:
        if '=' not in it:
            raise SystemExit(f'"이름=값" 형식이어야 함: {it}')
        k, v = it.split('=', 1)
        out[k.strip()] = [x.strip() for x in v.split(',')] if multi else v.strip()
    return out


def tag_of(combo):
    return ','.join(f'{k}={v}' for k, v in combo.items()) or 'default'


# =============================================================================
# 실행
# =============================================================================
def cmd_run(a):
    grid = parse_kv(a.grid, multi=True)
    fixed = parse_kv(a.set, multi=False)
    keys = list(grid)
    combos = [dict(zip(keys, vals)) for vals in itertools.product(*[grid[k] for k in keys])] or [{}]
    out_dir = os.path.expanduser(a.out or f'~/wall_scan_data/grid_{datetime.now():%Y%m%d_%H%M}')
    os.makedirs(out_dir, exist_ok=True)
    total = len(combos) * a.repeat

    print(f'[grid] 조합 {len(combos)}개 × 반복 {a.repeat}회 = {total}회 실행 → {out_dir}')
    for c in combos:
        print(f'   - {tag_of(c)}')
    if a.mode == 'real':
        ans = input(f'[REAL] 실제 로봇이 {total}회 연속으로 바닥/벽을 찍습니다 (매 회 홈 복귀). '
                    '주변 확인 후 y 입력: ')
        if ans.strip().lower() != 'y':
            print('취소했습니다.')
            return

    with open(os.path.join(out_dir, 'grid.json'), 'w') as f:
        json.dump({'mode': a.mode, 'grid': grid, 'fixed': fixed, 'repeat': a.repeat,
                   'true_height': a.true_height, 'true_width': a.true_width}, f, indent=2)

    # 같은 조합을 연속으로 돌리지 않고 섞어서 돌림 (센서 드리프트/온도 영향이 한 조합에 몰리지 않게)
    runs = [(r, i, c) for r in range(a.repeat) for i, c in enumerate(combos)]
    n = 0
    t_all = time.time()
    try:
        for r, i, c in runs:
            n += 1
            tag = tag_of(c)
            out = os.path.join(out_dir, f'c{i:02d}_r{r:02d}.json')
            if os.path.exists(out) and not a.overwrite:
                print(f'[grid] ({n}/{total}) {tag} #{r} 이미 있음 → 건너뜀')
                continue
            params = {'mode': a.mode, 'confirm': 'false', 'out': out, 'tag': tag, **fixed, **c}
            if a.true_height:
                params['true_height'] = a.true_height
            if a.true_width:
                params['true_width'] = a.true_width
            cmd = ['ros2', 'run', 'rokey1', 'wall_tap', '--ros-args']
            for k, v in params.items():
                cmd += ['-p', f'{k}:={v}']
            print(f'\n[grid] ({n}/{total}) {tag} #{r}')
            if a.dry_run:
                print('   ' + ' '.join(cmd))
                continue
            t = time.time()
            ret = subprocess.run(cmd).returncode
            print(f'[grid] 종료 코드 {ret}, {time.time() - t:.0f}s')
            if ret != 0 and a.mode == 'real' and a.stop_on_error:
                print('[grid] real 모드 실행 실패 → 중단 (--no-stop-on-error 로 계속 가능)')
                break
    except KeyboardInterrupt:
        print('\n[grid] 사용자 중단')
    print(f'[grid] 전체 {time.time() - t_all:.0f}s')
    if not a.dry_run:
        analyze(out_dir, a)


# =============================================================================
# 분석
# =============================================================================
def _mean(v):
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.mean(v)) if v else None


def _norm(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return [0.0 for _ in vals]
    lo, hi = min(v), max(v)
    return [0.0 if x is None or hi - lo < 1e-12 else (x - lo) / (hi - lo) for x in vals]


def load_runs(out_dir):
    runs = []
    for path in sorted(glob.glob(os.path.join(out_dir, '*.json'))):
        if path.endswith('_trace.json') or os.path.basename(path) == 'grid.json':
            continue
        try:
            with open(path) as f:
                d = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            print(f'[분석] 읽기 실패 {path}: {e}')
            continue
        if 'metrics' in d:
            d['_file'] = os.path.basename(path)
            runs.append(d)
    return runs


def summarize(runs):
    groups = {}
    for d in runs:
        groups.setdefault(d.get('tag') or 'default', []).append(d)
    rows = []
    for tag, ds in groups.items():
        ok = [d for d in ds if d.get('status') == 'ok']
        h = [d['results']['height']['median'] for d in ok if 'height' in d.get('results', {})]
        w = [d['results']['width']['median'] for d in ok if 'width' in d.get('results', {})]
        mt = [d['metrics'] for d in ok]
        row = {
            'tag': tag, 'runs': len(ds), 'ok': len(ok), 'fail_rate': 1 - len(ok) / len(ds),
            'height_mean': _mean(h), 'height_std': float(np.std(h)) if len(h) > 1 else None,
            'width_mean': _mean(w), 'width_std': float(np.std(w)) if len(w) > 1 else None,
            'height_abs_err': _mean([m.get('height_abs_err') for m in mt]),
            'width_abs_err': _mean([m.get('width_abs_err') for m in mt]),
            'time_mean': _mean([m.get('total_time') for m in mt]),
            'taps_mean': _mean([m.get('n_taps') for m in mt]),
            'latency_ms': (_mean([m.get('latency_mean') for m in mt]) or 0) * 1000 or None,
            'peak_force': _mean([m.get('peak_force_max') for m in mt]),
            'errors': '; '.join(sorted({str(d.get('error'))[:60] for d in ds if d.get('status') != 'ok'})),
        }
        row['dim_err'] = _mean([row['height_abs_err'], row['width_abs_err']])
        row['dim_std'] = _mean([row['height_std'], row['width_std']])
        rows.append(row)
    return rows


def score(rows, w):
    terms = {'dim_err': w['dim'], 'dim_std': w['rep'], 'time_mean': w['time'], 'peak_force': w['force']}
    norms = {k: _norm([r[k] for r in rows]) for k in terms}
    for i, r in enumerate(rows):
        used = [k for k in terms if r[k] is not None and terms[k] > 0]
        tot_w = sum(terms[k] for k in used)
        r['J'] = (sum(terms[k] * norms[k][i] for k in used) / tot_w) if (tot_w and r['ok']) else math.inf
    rows.sort(key=lambda r: (r['fail_rate'] > 0, r['J']))
    return rows


def analyze(out_dir, a):
    runs = load_runs(out_dir)
    if not runs:
        print(f'[분석] {out_dir} 에 결과가 없음')
        return
    rows = score(summarize(runs), {'dim': a.w_dim, 'rep': a.w_rep, 'time': a.w_time, 'force': a.w_force})

    def f(v, fmt='{:.2f}'):
        return '-' if v is None or (isinstance(v, float) and math.isinf(v)) else fmt.format(v)

    print(f'\n========== Grid Search 결과 ({len(runs)}회, {out_dir}) ==========')
    print(f'가중치: 오차 {a.w_dim}, 반복편차 {a.w_rep}, 시간 {a.w_time}, 최대힘 {a.w_force}  (J 작을수록 좋음)')
    hdr = f'{"순위":>3} {"J":>5} {"성공":>5} {"높이":>8} {"σ":>5} {"너비":>8} {"σ":>5} {"오차":>6} ' \
          f'{"시간s":>6} {"찍기":>5} {"지연ms":>6} {"최대N":>6}  조합'
    print(hdr)
    for i, r in enumerate(rows, 1):
        print(f'{i:>4} {f(r["J"]):>5} {r["ok"]:>2}/{r["runs"]:<2} {f(r["height_mean"], "{:.1f}"):>8} '
              f'{f(r["height_std"]):>5} {f(r["width_mean"], "{:.1f}"):>8} {f(r["width_std"]):>5} '
              f'{f(r["dim_err"]):>6} {f(r["time_mean"], "{:.0f}"):>6} {f(r["taps_mean"], "{:.0f}"):>5} '
              f'{f(r["latency_ms"], "{:.0f}"):>6} {f(r["peak_force"], "{:.1f}"):>6}  {r["tag"]}'
              + (f'  [실패: {r["errors"]}]' if r['errors'] else ''))
    if all(r['dim_err'] is None for r in rows):
        print('※ 정답 치수가 없어 오차 항은 빠짐 (--true-height / --true-width 로 지정)')

    csv_path = os.path.join(out_dir, 'summary.csv')
    with open(csv_path, 'w', newline='') as fp:
        wr = csv.DictWriter(fp, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    print(f'\n저장: {csv_path}')


# =============================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description='wall_tap Grid Search')
    sub = ap.add_subparsers(dest='cmd', required=True)

    def add_weights(p):
        p.add_argument('--w-dim', type=float, default=1.0, help='치수 오차 가중치')
        p.add_argument('--w-rep', type=float, default=0.5, help='반복 측정 편차 가중치')
        p.add_argument('--w-time', type=float, default=0.3, help='탐색 시간 가중치')
        p.add_argument('--w-force', type=float, default=0.2, help='최대 접촉력 가중치')

    r = sub.add_parser('run', help='조합별 반복 실행 후 분석')
    r.add_argument('--mode', choices=['virtual', 'real'], default='virtual')
    r.add_argument('--grid', action='append', help='바꿔가며 실험할 값: 이름=값1,값2,... (여러 번 가능)')
    r.add_argument('--set', action='append', help='모든 실행에 고정할 값: 이름=값 (여러 번 가능)')
    r.add_argument('--repeat', type=int, default=3)
    r.add_argument('--true-height', type=float, default=0.0)
    r.add_argument('--true-width', type=float, default=0.0)
    r.add_argument('--out', default='', help='결과 폴더 (같은 폴더로 다시 실행하면 이어서 진행)')
    r.add_argument('--overwrite', action='store_true', help='이미 있는 결과도 다시 실행')
    r.add_argument('--dry-run', action='store_true', help='실행할 명령만 출력')
    r.add_argument('--no-stop-on-error', dest='stop_on_error', action='store_false',
                   help='(real) 실패해도 다음 실행 계속')
    add_weights(r)

    an = sub.add_parser('analyze', help='이미 모은 결과 분석')
    an.add_argument('out_dir')
    add_weights(an)

    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    if a.cmd == 'run':
        cmd_run(a)
    else:
        analyze(os.path.expanduser(a.out_dir), a)


if __name__ == '__main__':
    main()
