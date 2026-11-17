"""관절 기준 도달 가능 검사 (두산 컨트롤러 ikin / fkin 사용, 로봇은 움직이지 않음)

왜 필요한가:
  - 거리상 닿는 위치라도 "그 자세로" 가려면 관절 한계(예: J3 ±135도)를 넘을 수 있음
  - 컨트롤러는 도달 불가 이동을 거부하면서도 명령 응답은 '성공'으로 줌 → 미리 검사해야 함
  - movel 은 이동 중 팔 형태(solution space, sol)를 바꿀 수 없음 → 한 경로는 같은 sol 로 검사

검사 항목 (점마다):
  1. ikin 으로 관절값 계산 (sol 0~7)
  2. fkin 으로 되돌려 목표 위치/자세와 같은지 확인 (ikin 은 불가능해도 값을 돌려주기 때문)
  3. 관절 한계 (JOINT_LIMIT_DEG - 여유)
  4. 특이점: |J5| 가 너무 작으면 (손목 축이 일직선) 직선 이동이 불안정
  5. 연속성: 이웃한 점 사이에 관절이 너무 크게 바뀌면 (특이점 근처 / 팔 뒤집힘) 불가
"""
import math

from sanding import surface_math as sm

# 컨트롤러 안전 설정(global-normal)과 맞춰야 함. 교시기에서 확인 후 수정하세요.
#   알람 예: "JNT(3) POS LIMIT ERROR ... MAX(135.000)-MIN(-135.000)"
JOINT_LIMIT_DEG = [360.0, 360.0, 135.0, 360.0, 360.0, 360.0]
JOINT_LIMIT_MARGIN_DEG = 5.0
SINGULAR_J5_MIN_DEG = 5.0     # |J5| 가 이보다 작으면 특이점으로 보고 불가
JOINT_JUMP_MAX_DEG = 30.0     # 이웃 점 사이 관절 변화 최대 (경로 점 간격 ~25mm 기준)


class ReachError(Exception):
    """도달 불가 (이유 포함)"""


def vec6(x):
    """DSR 반환값(list / numpy 배열 / 실패 시 -1) → 6개 float 리스트 또는 None"""
    try:
        v = [float(a) for a in list(x)[:6]]
    except TypeError:
        return None
    return v if len(v) == 6 else None


def fmt(p, nd=1):
    return [round(float(a), nd) for a in p]


class ReachChecker:
    def __init__(self, dr, posx, logger=None,
                 joint_limits=JOINT_LIMIT_DEG, margin=JOINT_LIMIT_MARGIN_DEG,
                 j5_min=SINGULAR_J5_MIN_DEG, jump_max=JOINT_JUMP_MAX_DEG):
        self.dr, self.posx, self.log = dr, posx, logger
        self.limits, self.margin = list(joint_limits), margin
        self.j5_min, self.jump_max = j5_min, jump_max

    # ── 점 하나 ──────────────────────────────────────────────────
    def ik(self, pose, sol):
        """pose(posx 6) 를 sol 로 풀어 검사. (q 또는 None, 실패 이유)"""
        dr = self.dr
        q = vec6(dr.ikin(self.posx(list(pose)), sol, ref=dr.DR_BASE))
        if q is None:
            return None, "ikin 실패"
        fk = vec6(dr.fkin(dr.posj(q), ref=dr.DR_BASE))
        if fk is None:
            return None, "fkin 실패"
        perr = sm.norm(sm.sub(fk[:3], pose[:3]))
        if perr > 1.0:
            return None, f"도달 불가 (위치 오차 {perr:.0f}mm)"
        r_fk, r_t = sm.zyz_to_matrix(fk[3:]), sm.zyz_to_matrix(pose[3:])
        if sum(sm.norm(sm.sub(sm.column(r_fk, i), sm.column(r_t, i))) for i in range(3)) > 0.01:
            return None, "도달 불가 (자세 오차)"
        over = [f"J{i + 1}={a:.0f}(한계 ±{lim:.0f})" for i, (a, lim) in enumerate(zip(q, self.limits))
                if abs(a) > lim - self.margin]
        if over:
            return None, "관절 한계 초과 " + ", ".join(over)
        if abs(q[4]) < self.j5_min:
            return None, f"특이점 (J5={q[4]:.1f}도)"
        return q, ""

    # ── 경로 (같은 sol) ──────────────────────────────────────────
    def path_in_sol(self, poses, sol):
        """poses 전체를 같은 sol 로 검사. (q 리스트 또는 None, 통과 점 수, 실패 이유)"""
        qs = []
        for i, p in enumerate(poses):
            q, why = self.ik(p, sol)
            if q is None:
                return None, i, f"점 {i} {fmt(p[:3], 0)}: {why}"
            if qs:
                jump = max(abs(a - b) for a, b in zip(q, qs[-1]))
                if jump > self.jump_max:
                    return None, i, (f"점 {i - 1}→{i} {fmt(p[:3], 0)}: 관절 급변 {jump:.0f}도 "
                                     "(특이점 근처/팔 뒤집힘)")
            qs.append(q)
        return qs, len(poses), ""

    def solve_path(self, poses, q_ref=None, sols=range(8), what="경로"):
        """poses 전체가 되는 sol 을 찾음. q_ref 가 있으면 첫 점이 q_ref 와 가장 가까운 sol.

        반환: (sol, q 리스트).  불가면 ReachError (가장 멀리 간 sol 의 실패 이유 포함)
        """
        best, fail = None, (-1, "")
        for sol in sols:
            qs, n_ok, why = self.path_in_sol(poses, sol)
            if qs is None:
                if n_ok > fail[0]:
                    fail = (n_ok, f"sol {sol}: {why}")
                continue
            cost = 0.0 if q_ref is None else sum(abs(a - b) for a, b in zip(qs[0], q_ref))
            if best is None or cost < best[0]:
                best = (cost, sol, qs)
            if q_ref is None:
                break
        if best is None:
            raise ReachError(f"{what} 불가 - {fail[1]} ({fail[0]}/{len(poses)} 점 통과)")
        return best[1], best[2]


def sample_line(a, b, step):
    """a → b 를 step 간격으로 나눈 점들 (a, b 포함). a, b 는 위치 3개 또는 posx 6개"""
    d = sm.norm(sm.sub(a[:3], b[:3]))
    n = max(1, int(math.ceil(d / step - 1e-6)))
    return [sm.add(a[:3], sm.scale(sm.sub(b[:3], a[:3]), i / n)) + list(a[3:]) for i in range(n + 1)]


def sample_joint(qa, qb, step_deg=5.0):
    """movej 는 관절 공간 직선 보간 → qa → qb 를 step_deg 간격으로 나눈 관절값들"""
    d = max(abs(x - y) for x, y in zip(qa, qb))
    n = max(1, int(math.ceil(d / step_deg)))
    return [[x + (y - x) * i / n for x, y in zip(qa, qb)] for i in range(n + 1)]
