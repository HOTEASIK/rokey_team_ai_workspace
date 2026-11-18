"""힘제어 동작의 fz 기록 → contact_ratio, force_stats_json (Process feedback 용)

사용 예 (공정 노드):
    fz_list = []                      # 힘제어 중 주기적으로 get_tool_force()[2] 추가
    ...
    ratio, stats = summarize(fz_list, target=-10.0, tol=2.0)
    fb.contact_ratio = ratio
    fb.force_stats_json = stats
"""
import json
import statistics


def summarize(fz, target, tol, axis='fz'):
    """fz: 샘플 리스트, target: 목표 fz(N), tol: 허용 오차(N).
    반환: (목표 ±tol 안에 있던 비율 0~1, 통계 JSON 문자열). 샘플이 없으면 (-1.0, "")."""
    if not fz:
        return -1.0, ''
    ratio = sum(abs(v - target) <= tol for v in fz) / len(fz)
    if axis not in ('fx', 'fy', 'fz'):
        raise ValueError('힘 통계 축은 fx/fy/fz여야 합니다.')
    stats = {f'{axis}_mean': round(statistics.fmean(fz), 3), f'{axis}_min': min(fz), f'{axis}_max': max(fz),
             f'{axis}_std': round(statistics.pstdev(fz), 3), 'n': len(fz)}
    return ratio, json.dumps(stats)
