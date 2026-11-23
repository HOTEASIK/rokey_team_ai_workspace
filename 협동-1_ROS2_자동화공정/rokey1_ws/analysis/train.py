"""실험 데이터로 파라미터 고르기 + 간단한 모델 (ROS 노드 아님, DB만 읽음)

설치: sudo apt install python3-sklearn
실행: python3 ~/ws_cobot_pjt/ws_dsr/src/rokey1/analysis/train.py putty_scrape clay

대상: run.purpose = EXPERIMENT, 지정한 재료, 지정한 step, 첫 시도(attempt = 0)
입력: params 의 target_fz, vel + 틈 길이(gap 시작~끝 거리로 계산)
출력: 1) 조합별 표  2) 성공 확률(로지스틱 회귀)  3) 품질 점수 규칙(얕은 결정트리)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pymysql
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeRegressor, export_text

from utils.config import CFG

# 기존 Z축 기록과 현장 퍼티 X축 기록을 같은 분석 입력으로 읽는다.
FEATURES = ['target_force', 'vel', 'length_mm']

SQL_TABLE = """
SELECT COALESCE(t.params->>'$.target_force', t.params->>'$.target_fz') fz, t.params->>'$.vel' vel, COUNT(*) n,
       AVG(t.success) ok, AVG(t.contact_ratio) contact, AVG(q.score) score
FROM task_log t
JOIN run r ON r.run_id = t.run_id AND r.purpose = 'EXPERIMENT' AND r.material = %s
LEFT JOIN quality q ON q.run_id = t.run_id AND q.gap_id = t.gap_id AND q.task_code = t.task_code
WHERE t.step = %s AND t.attempt = 0
GROUP BY fz, vel ORDER BY score DESC, ok DESC
"""

SQL_ROWS = """
SELECT CAST(COALESCE(t.params->>'$.target_force', t.params->>'$.target_fz') AS DOUBLE), CAST(t.params->>'$.vel' AS DOUBLE),
       SQRT(POW(g.end_x-g.start_x,2)+POW(g.end_y-g.start_y,2)+POW(g.end_z-g.start_z,2)) length_mm, t.success, q.score
FROM task_log t
JOIN run r ON r.run_id = t.run_id AND r.purpose = 'EXPERIMENT' AND r.material = %s
JOIN gap g ON g.gap_id = t.gap_id
LEFT JOIN quality q ON q.run_id = t.run_id AND q.gap_id = t.gap_id AND q.task_code = t.task_code
WHERE t.step = %s AND t.attempt = 0
"""


def main():
    step = sys.argv[1] if len(sys.argv) > 1 else 'putty_scrape'
    material = sys.argv[2] if len(sys.argv) > 2 else 'clay'
    conn = pymysql.connect(**CFG['db'])
    with conn.cursor() as cur:
        cur.execute(SQL_TABLE, (material, step))
        table = cur.fetchall()
        cur.execute(SQL_ROWS, (material, step))
        rows = [r for r in cur.fetchall() if None not in r[:4]]
    conn.close()

    print(f'== 1) 조합별 결과 ({step}, {material}) ==')
    print('target_force  vel   n   성공률  접촉률  점수')
    for fz, vel, n, ok, contact, score in table:
        print(f'{str(fz):>9} {str(vel):>4} {n:>3}  {ok or 0:.2f}   {contact or 0:.2f}   {score or 0:.2f}')

    X = [r[:3] for r in rows]
    y = [int(r[3]) for r in rows]
    print(f'\n== 2) 성공 확률: 로지스틱 회귀 (데이터 {len(X)}개) ==')
    if len(set(y)) < 2:
        print('성공/실패가 모두 있어야 학습할 수 있다 → 건너뜀')
    else:
        m = LogisticRegression(max_iter=1000).fit(X, y)
        for name, c in zip(FEATURES, m.coef_[0]):
            print(f'{name:>10}: {c:+.3f}   (+ 이면 값이 클수록 성공 확률이 오른다)')

    scored = [r for r in rows if r[4] is not None]
    print(f'\n== 3) 품질 점수: 결정트리 (데이터 {len(scored)}개) ==')
    if len(scored) < 5:
        print('점수가 5개 이상 있어야 한다 → 건너뜀')
    else:
        t = DecisionTreeRegressor(max_depth=2).fit([r[:3] for r in scored], [r[4] for r in scored])
        print(export_text(t, feature_names=FEATURES))


if __name__ == '__main__':
    main()
