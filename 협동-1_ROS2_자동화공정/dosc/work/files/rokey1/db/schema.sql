-- 벽면 보수 협동로봇 DB 스키마 (MySQL 8)
-- 새로 만들 때: sudo mysql < schema.sql
-- 이미 11/17 버전이 있으면: mysql -u cobot -p < migrate_01.sql
-- 구조 변경은 이 파일(+ migrate_xx.sql)로만 한다 (Git 관리, 담당자 1명)
--
-- 쓰기 담당: command(웹 INSERT, 매니저 status 갱신) / quality(웹) / 나머지는 매니저만

CREATE DATABASE IF NOT EXISTS cobot DEFAULT CHARACTER SET utf8mb4;
USE cobot;

-- 0) 명령 (웹 → 매니저)
CREATE TABLE IF NOT EXISTS command (
  cmd_id     INT AUTO_INCREMENT PRIMARY KEY,
  cmd        VARCHAR(20) NOT NULL,                   -- START / STOP
  status     VARCHAR(10) NOT NULL DEFAULT 'PENDING', -- PENDING / ACCEPTED / DONE / FAILED
  purpose    VARCHAR(10) DEFAULT 'TEST',             -- START 용: TEST / EXPERIMENT / DEMO
  material   VARCHAR(20),                            -- START 용: putty / clay
  note       VARCHAR(200),
  created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
  updated_at DATETIME DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

-- 1) 실행 1번 (START 1번). run_id = command.cmd_id
CREATE TABLE IF NOT EXISTS run (
  run_id       INT PRIMARY KEY,
  purpose      VARCHAR(10),
  material     VARCHAR(20),
  code_version VARCHAR(40),        -- git 짧은 해시
  started_at   DATETIME(3),
  ended_at     DATETIME(3),
  result       VARCHAR(10),        -- OK / HOME / HOLD / STOP / ESTOP ...
  note         VARCHAR(200)
);

-- 2) 벽: base 좌표계 기준 [x,y,z] mm
CREATE TABLE IF NOT EXISTS wall (
  wall_id    INT AUTO_INCREMENT PRIMARY KEY,
  run_id     INT,
  top_x   DOUBLE, top_y   DOUBLE, top_z   DOUBLE,   -- 높이
  floor_x DOUBLE, floor_y DOUBLE, floor_z DOUBLE,   -- 바닥
  right_x DOUBLE, right_y DOUBLE, right_z DOUBLE,   -- 오른쪽 끝점
  left_x  DOUBLE, left_y  DOUBLE, left_z  DOUBLE,   -- 왼쪽 끝점
  created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

-- 3) 틈
CREATE TABLE IF NOT EXISTS gap (
  gap_id   INT AUTO_INCREMENT PRIMARY KEY,
  wall_id  INT NOT NULL,
  start_x DOUBLE, start_y DOUBLE, start_z DOUBLE,
  end_x   DOUBLE, end_y   DOUBLE, end_z   DOUBLE,
  created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
  FOREIGN KEY (wall_id) REFERENCES wall(wall_id)
);

-- 4) 작업 기록 (세부동작 1개 = 1줄)
CREATE TABLE IF NOT EXISTS task_log (
  log_id        INT AUTO_INCREMENT PRIMARY KEY,
  run_id        INT,
  task_code     VARCHAR(20),          -- EXPLORE / PUTTY / SANDING / ESTOP
  step          VARCHAR(40),          -- 대분류_소분류
  attempt       TINYINT DEFAULT 0,    -- 0 = 첫 시도, 1 = 첫 재시도 ...
  success       BOOLEAN,
  error_code    VARCHAR(20),          -- E_FORCE 등 (성공이면 NULL)
  fail_reason   VARCHAR(100),         -- 사람이 읽는 설명
  gap_id        INT NULL,
  joint         JSON,                 -- [j1..j6] deg
  task          JSON,                 -- [x,y,z,rx,ry,rz]
  force_val     JSON,                 -- 동작 끝 [fx,fy,fz,mx,my,mz]
  params        JSON,                 -- 그때 사용한 파라미터 {"target_fz":-10,"vel":30,...}
  contact_ratio FLOAT,                -- 목표 힘 ±tol 안에 있었던 비율 0~1 (힘제어 없는 동작은 NULL)
  force_stats   JSON,                 -- {"fz_mean","fz_min","fz_max","fz_std","n"}
  started_at    DATETIME(3),
  duration_ms   INT,
  created_at    DATETIME(3) DEFAULT CURRENT_TIMESTAMP(3),
  FOREIGN KEY (gap_id) REFERENCES gap(gap_id)
);

-- 5) 품질 점수 (작업 후 웹에서 입력/수정. 틈 1개 × 공정 1개 = 1줄)
CREATE TABLE IF NOT EXISTS quality (
  quality_id INT AUTO_INCREMENT PRIMARY KEY,
  run_id     INT,
  gap_id     INT NOT NULL,
  task_code  VARCHAR(20),             -- PUTTY / SANDING
  score      TINYINT,                 -- 1~5
  depth_mm   FLOAT NULL,              -- 작업 후 다시 잰 틈 깊이 (측정 가능하면)
  rater      VARCHAR(20),
  note       VARCHAR(200),
  created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
  UNIQUE (run_id, gap_id, task_code),
  FOREIGN KEY (gap_id) REFERENCES gap(gap_id)
);
