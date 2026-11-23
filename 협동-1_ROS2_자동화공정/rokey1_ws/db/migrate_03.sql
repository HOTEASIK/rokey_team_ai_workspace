-- run_id 외래키 추가 (2026-11-18). 짝 없는 예전 행은 지우지 않고 LEGACY run 으로 묶는다
-- 실행: mysql -u cobot -pcobot < ~/ws_cobot_pjt/ws_dsr/src/rokey1/db/migrate_03.sql   (한 번만, migrate_02 다음)
USE cobot;

INSERT INTO run (run_id, task, result, note)
SELECT DISTINCT o.run_id, 'LEGACY', 'LEGACY', 'command 테이블 시절 데이터 (migrate_03)'
FROM (SELECT run_id FROM task_log UNION SELECT run_id FROM wall UNION SELECT run_id FROM quality) o
WHERE o.run_id IS NOT NULL AND o.run_id NOT IN (SELECT run_id FROM run);

ALTER TABLE wall     ADD FOREIGN KEY (run_id) REFERENCES run(run_id);
ALTER TABLE task_log ADD FOREIGN KEY (run_id) REFERENCES run(run_id);
ALTER TABLE quality  ADD FOREIGN KEY (run_id) REFERENCES run(run_id);
