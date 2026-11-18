-- 모델 입력 추가: 틈 특징 + 힘 시계열 (기존 데이터 유지)
-- 실행: mysql -u cobot -pcobot < ~/ws_cobot_pjt/ws_dsr/src/rokey1_ws/db/migrate_02.sql   (한 번만)
-- 새로 만들 때도 schema.sql 다음에 이 파일을 실행한다
USE cobot;

-- 1) 틈 특징. 길이·방향은 좌표로 자동 계산, 폭·깊이는 측정한 경우만 (NULL 허용)
ALTER TABLE gap
  ADD COLUMN length_mm DOUBLE AS (SQRT(POW(end_x - start_x, 2) + POW(end_y - start_y, 2) + POW(end_z - start_z, 2))) STORED,
  ADD COLUMN orient    CHAR(1) AS (IF(ABS(end_z - start_z) > SQRT(POW(end_x - start_x, 2) + POW(end_y - start_y, 2)), 'V', 'H')) STORED,  -- H 가로 / V 세로
  ADD COLUMN width_mm  FLOAT NULL,
  ADD COLUMN depth_mm  FLOAT NULL;   -- 작업 전 깊이 (작업 후 깊이는 quality.depth_mm)

-- 2) 힘 시계열 (요약값으로 부족할 때 원인 분석용). 퍼티·샌딩 접촉 구간만, 10~20Hz
CREATE TABLE IF NOT EXISTS force_trace (
  log_id INT NOT NULL,
  t_ms   INT NOT NULL,                -- 세부동작 시작부터 ms
  fx FLOAT, fy FLOAT, fz FLOAT,
  mx FLOAT, my FLOAT, mz FLOAT,
  PRIMARY KEY (log_id, t_ms),
  FOREIGN KEY (log_id) REFERENCES task_log(log_id)
);
