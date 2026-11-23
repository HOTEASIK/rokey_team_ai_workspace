-- 11/17 첫 버전 DB → 데이터 수집용 버전으로 변경 (기존 데이터 유지)
-- 실행: mysql -u cobot -p < migrate_01.sql   (한 번만)
USE cobot;

ALTER TABLE command
  ADD COLUMN purpose  VARCHAR(10) DEFAULT 'TEST' AFTER status,
  ADD COLUMN material VARCHAR(20) AFTER purpose,
  ADD COLUMN note     VARCHAR(200) AFTER material;

CREATE TABLE IF NOT EXISTS run (
  run_id       INT PRIMARY KEY,
  purpose      VARCHAR(10),
  material     VARCHAR(20),
  code_version VARCHAR(40),
  started_at   DATETIME(3),
  ended_at     DATETIME(3),
  result       VARCHAR(10),
  note         VARCHAR(200)
);

ALTER TABLE task_log
  ADD COLUMN attempt       TINYINT DEFAULT 0 AFTER step,
  ADD COLUMN error_code    VARCHAR(20) AFTER success,
  ADD COLUMN params        JSON AFTER force_val,
  ADD COLUMN contact_ratio FLOAT AFTER params,
  ADD COLUMN force_stats   JSON AFTER contact_ratio,
  ADD COLUMN started_at    DATETIME(3) AFTER force_stats;

CREATE TABLE IF NOT EXISTS quality (
  quality_id INT AUTO_INCREMENT PRIMARY KEY,
  run_id     INT,
  gap_id     INT NOT NULL,
  task_code  VARCHAR(20),
  score      TINYINT,
  depth_mm   FLOAT NULL,
  rater      VARCHAR(20),
  note       VARCHAR(200),
  created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
  UNIQUE (run_id, gap_id, task_code),
  FOREIGN KEY (gap_id) REFERENCES gap(gap_id)
);
