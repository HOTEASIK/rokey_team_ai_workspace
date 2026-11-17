import random
import statistics


class GapDetector:
    def __init__(
        self,
        calibration_count=10,
        threshold=0.8,
        confirm_count=3
    ):
        # 기준 힘을 측정할 데이터 개수
        self.calibration_count = calibration_count

        # 기준 힘과 현재 힘의 차이가 이 값 이상이면 이상 감지
        self.threshold = threshold

        # 연속으로 몇 번 감지되어야 틈으로 판단할지
        self.confirm_count = confirm_count

        self.calibration_data = []
        self.reference_force = None

        self.gap_detected = False
        self.candidate_count = 0
        self.normal_count = 0

        self.candidate_start = None
        self.gap_start = None
        self.last_gap_position = None

        self.gaps = []

    def set_reference_force(self):
        """초기 측정값으로 기준 힘을 설정합니다."""

        self.reference_force = statistics.median(
            self.calibration_data
        )

        print(f"기준 힘 설정 완료: {self.reference_force:.2f} N")

    def add_sample(self, position, force):
        """
        TCP 위치와 측정된 힘을 입력받아 틈을 검사합니다.

        position: TCP의 이동 위치(mm)
        force: 측정된 힘(N)
        """

        # 1. 처음에는 정상 벽의 힘을 측정합니다.
        if self.reference_force is None:

            self.calibration_data.append(force)

            if len(self.calibration_data) >= self.calibration_count:
                self.set_reference_force()

            return

        # 2. 기준 힘과 현재 힘의 차이를 계산합니다.
        force_change = abs(force - self.reference_force)

        # 3. 힘 변화가 임계값 이상인지 확인합니다.
        if force_change >= self.threshold:

            self.normal_count = 0

            if not self.gap_detected:

                # 틈 후보가 처음 발견된 위치 저장
                if self.candidate_count == 0:
                    self.candidate_start = position

                self.candidate_count += 1

                # 연속해서 감지되면 틈으로 확정
                if self.candidate_count >= self.confirm_count:

                    self.gap_detected = True
                    self.gap_start = self.candidate_start
                    self.last_gap_position = position

                    print(
                        f"[틈 감지] 시작 위치: "
                        f"{self.gap_start:.2f} mm"
                    )

            else:
                # 이미 틈으로 판단한 상태라면 마지막 위치 갱신
                self.last_gap_position = position

        else:
            self.candidate_count = 0
            self.candidate_start = None

            if self.gap_detected:

                self.normal_count += 1

                # 정상 상태가 연속으로 확인되면 틈 종료
                if self.normal_count >= self.confirm_count:

                    gap_end = self.last_gap_position

                    self.gaps.append(
                        (self.gap_start, gap_end)
                    )

                    print(
                        f"[틈 종료] 끝 위치: "
                        f"{gap_end:.2f} mm"
                    )

                    print(
                        f"[틈 폭 추정] "
                        f"{gap_end - self.gap_start:.2f} mm\n"
                    )

                    self.gap_detected = False
                    self.normal_count = 0
                    self.gap_start = None
                    self.last_gap_position = None

    def finish(self):
        """측정이 끝났을 때 아직 열린 틈이 있으면 마무리합니다."""

        if self.gap_detected:

            gap_end = self.last_gap_position

            self.gaps.append(
                (self.gap_start, gap_end)
            )

            print(f"[틈 종료] 끝 위치: {gap_end:.2f} mm")
            print(
                f"[틈 폭 추정] "
                f"{gap_end - self.gap_start:.2f} mm"
            )


def main():

    detector = GapDetector(
        calibration_count=10,
        threshold=0.8,
        confirm_count=3
    )

    print("=== 틈 감지 테스트 시작 ===\n")

    # TCP가 0mm부터 10mm까지 이동한다고 가정
    for i in range(101):

        position = i * 0.1

        # 기본적으로 벽에 접촉했을 때 힘은 5N
        force = 5.0

        # 4mm ~ 4.5mm 구간에서 힘이 감소한다고 가정
        if 4.0 <= position <= 4.5:
            force = 3.5

        # 실제 센서처럼 약간의 노이즈 추가
        force += random.gauss(0, 0.05)

        detector.add_sample(position, force)

    detector.finish()

    print("\n=== 최종 감지 결과 ===")

    if detector.gaps:
        for index, (start, end) in enumerate(
            detector.gaps, start=1
        ):
            print(f"{index}번째 틈")
            print(f"  시작 위치: {start:.2f} mm")
            print(f"  끝 위치: {end:.2f} mm")
            print(f"  폭 추정: {end - start:.2f} mm")
    else:
        print("감지된 틈이 없습니다.")


if __name__ == "__main__":
    main()