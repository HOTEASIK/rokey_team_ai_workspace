"""실험 데이터로 파라미터 고르기 + 간단한 모델 (ROS 노드 아님, DB만 읽음)

설치: sudo apt install python3-sklearn
실행: python3 ~/ws_cobot_pjt/ws_dsr/src/rokey1/analysis/train.py putty_scrape clay

대상: run.purpose = EXPERIMENT, 지정한 재료, 지정한 step, 첫 시도(attempt = 0)
입력: params 의 target_fz, vel (실험은 같은 종류의 틈으로 고정)
출력: 1) 조합별 표
      2) 접촉 유지율 2차 회귀(반응표면) → 실험 범위 안 최적 조합 + LOO 검증 R²
      3) 성공 확률(로지스틱 회귀, 표준화 계수)  4) 품질 점수 규칙(얕은 결정트리, 발표용)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pymysql
from sklearn.linear_model import LinearRegression, LogisticRegression
from sklearn.metrics import r2_score
from sklearn.model_selection import LeaveOneOut, cross_val_predict
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.tree import DecisionTreeRegressor, export_text

from utils.config import CFG

FEATURES = ['target_fz', 'vel']

SQL_TABLE = """
SELECT t.params->>'$.target_fz' fz, t.params->>'$.vel' vel, COUNT(*) n,
       AVG(t.success) ok, AVG(t.contact_ratio) contact, AVG(q.score) score
FROM task_log t
JOIN run r ON r.run_id = t.run_id AND r.purpose = 'EXPERIMENT' AND r.material = %s
LEFT JOIN quality q ON q.run_id = t.run_id AND q.gap_id = t.gap_id AND q.task_code = t.task_code
WHERE t.step = %s AND t.attempt = 0
GROUP BY fz, vel ORDER BY score DESC, ok DESC
"""

SQL_ROWS = """
SELECT CAST(t.params->>'$.target_fz' AS DOUBLE), CAST(t.params->>'$.vel' AS DOUBLE),
       t.success, t.contact_ratio, q.score
FROM task_log t
JOIN run r ON r.run_id = t.run_id AND r.purpose = 'EXPERIMENT' AND r.material = %s
LEFT JOIN quality q ON q.run_id = t.run_id AND q.gap_id = t.gap_id AND q.task_code = t.task_code
WHERE t.step = %s AND t.attempt = 0
"""


def fit_surface(X, y):
    """2차 회귀 → (LOO R², 최적 [fz, vel], 예측값). 최적은 실험 범위 안 11×11 격자에서 찾는다."""
    model = make_pipeline(PolynomialFeatures(2), LinearRegression())
    r2 = r2_score(y, cross_val_predict(model, X, y, cv=LeaveOneOut()))
    model.fit(X, y)
    (f0, v0), (f1, v1) = [min(c) for c in zip(*X)], [max(c) for c in zip(*X)]
    grid = [[f0 + (f1 - f0) * i / 10, v0 + (v1 - v0) * j / 10] for i in range(11) for j in range(11)]
    pred, best = max(zip(model.predict(grid), grid))
    return r2, best, pred


def main():
    step = sys.argv[1] if len(sys.argv) > 1 else 'putty_scrape'
    material = sys.argv[2] if len(sys.argv) > 2 else 'clay'
    conn = pymysql.connect(**CFG['db'])
    with conn.cursor() as cur:
        cur.execute(SQL_TABLE, (material, step))
        table = cur.fetchall()
        cur.execute(SQL_ROWS, (material, step))
        rows = [r for r in cur.fetchall() if None not in r[:3]]
    conn.close()

    print(f'== 1) 조합별 결과 ({step}, {material}) ==')
    print('target_fz  vel   n   성공률  접촉률  점수')
    for fz, vel, n, ok, contact, score in table:
        print(f'{str(fz):>9} {str(vel):>4} {n:>3}  {ok or 0:.2f}   {contact or 0:.2f}   {score or 0:.2f}')

    cr = [r for r in rows if r[3] is not None]
    print(f'\n== 2) 접촉 유지율: 2차 회귀 (데이터 {len(cr)}개) ==')
    if len(cr) < 10:
        print('10개 이상 있어야 한다 → 건너뜀')
    else:
        r2, (fz, vel), pred = fit_surface([r[:2] for r in cr], [r[3] for r in cr])
        print(f'LOO R²: {r2:.2f}   (0보다 커야 평균으로 찍는 것보다 낫다)')
        print(f'최적 조합: target_fz={fz:.1f}, vel={vel:.1f} → 예측 접촉률 {pred:.2f}')
        print('※ 최적이 실험 범위 끝이면 범위를 넓혀 다시 실험한다')

    y = [int(r[2]) for r in rows]
    print(f'\n== 3) 성공 확률: 로지스틱 회귀 (데이터 {len(y)}개, 실패 {y.count(0)}개) ==')
    if min(y.count(0), y.count(1)) < 3:
        print('성공·실패가 각각 3개 이상 있어야 한다 → 건너뜀')
    else:
        m = make_pipeline(StandardScaler(), LogisticRegression(max_iter=1000)).fit([r[:2] for r in rows], y)
        for name, c in zip(FEATURES, m[-1].coef_[0]):
            print(f'{name:>10}: {c:+.3f}   (표준화 계수. 절댓값이 클수록 영향이 크고, + 이면 값이 클수록 성공 확률이 오른다)')

    scored = [r for r in rows if r[4] is not None]
    print(f'\n== 4) 품질 점수: 결정트리 (데이터 {len(scored)}개, 발표용 규칙) ==')
    if len(scored) < 5:
        print('점수가 5개 이상 있어야 한다 → 건너뜀')
    else:
        t = DecisionTreeRegressor(max_depth=2).fit([r[:2] for r in scored], [r[4] for r in scored])
        print(export_text(t, feature_names=FEATURES))


if __name__ == '__main__':
    main()
