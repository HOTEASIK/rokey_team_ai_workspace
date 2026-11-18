-- 웹(hmi_server) 직접 실행 구조로 변경: command 삭제, run 에 실행 정보 통합 (2026-11-18)
-- 실행: mysql -u cobot -pcobot < ~/ws_cobot_pjt/ws_dsr/src/rokey1/db/migrate_02.sql   (한 번만, 백업 후)
USE cobot;

DROP TABLE IF EXISTS command;

ALTER TABLE run
  MODIFY run_id INT AUTO_INCREMENT,
  MODIFY result VARCHAR(20),
  ADD COLUMN task       VARCHAR(10) AFTER run_id,     -- ALL / EXPLORE / PUTTY / SANDING
  ADD COLUMN gap_id     INT NULL AFTER task,          -- PUTTY/SANDING 단독 실행 때 지정한 틈
  ADD COLUMN mode       VARCHAR(10) AFTER material,   -- virtual / real
  ADD COLUMN data_check VARCHAR(200) AFTER result;    -- OK 또는 빠진 데이터 목록

-- 예전 task_log/wall 의 run_id 와 겹치지 않게 그 다음 번호부터 시작
SET @n = (SELECT GREATEST(IFNULL(MAX(run_id), 0), (SELECT IFNULL(MAX(run_id), 0) FROM wall)) + 1 FROM task_log);
SET @sql = CONCAT('ALTER TABLE run AUTO_INCREMENT=', @n);
PREPARE stmt FROM @sql;
EXECUTE stmt;
DEALLOCATE PREPARE stmt;
