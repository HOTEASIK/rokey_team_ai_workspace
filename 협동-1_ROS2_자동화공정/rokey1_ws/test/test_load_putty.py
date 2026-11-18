"""판 퍼티 작업의 좌표 계약·접촉/힘 검증·정리. 로봇/ROS graph 없이 실행한다."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from putty_demo import load_putty as module
from putty_demo.action_protocol import process_load_goal, worker_command


def settings():
    # 테스트 전용 임의 값. 운영 기본값/권장 힘이 아니다.
    return module.make_parser().parse_args(['--execute-motion',
        '--approach-height','10','--distance','35','--search-distance','20',
        '--search-speed','2','--contact-force','2','--hold-force','4','--force-limit','7',
        '--force-tolerance','1','--force-wait-timeout','5','--force-stable-time','0.2',
        '--force-loss-timeout','0.2','--scrape-speed','5','--lift','20','--lift-speed','5',
        '--acceleration','10','--force-stiffness','3000','3000','3000','200','200','200'])


class IO:
    def __init__(self, args, overshoot=False):
        self.args = args
        self.position = module.approach_pose(args)
        self.phase = 'above'
        self.events = []
        self.overshoot = overshoot
    def read_joints(self): return self.position  # IO 모의 관절 식별값
    def move_joint_pose(self,target): self.position=target;self.events.append(('joint_pose',target))
    def move_joint(self,target): self.position=target;self.events.append(('joint',target))
    def read_pose(self): return self.position
    def read_force(self):
        force = 0 if self.phase=='above' else 2 if self.phase=='contact' else 4
        if self.overshoot and self.phase=='hold': force=8
        return [0.,0.,float(force),0.,0.,0.]
    def read_state(self): return 1
    def move_async(self, target, speed):
        self.events.append(('move',tuple(target)))
        if self.phase=='above':
            self.position = self.args.contact_pose
            self.phase='contact'
        else:
            self.position=target
    def stop_and_wait(self): self.events.append(('stop',None))
    def compliance(self, stiffness): self.events.append(('compliance',None))
    def set_force(self, force): self.phase='hold'; self.events.append(('force',force))
    def release(self): self.events.append(('release',None))
    def move_line(self, target, speed): self.position=target; self.events.append(('lift',target))
    def log(self, *args): pass


def fast_clock(monkeypatch):
    now=[0.]
    def clock(): now[0]+=0.05; return now[0]
    monkeypatch.setattr(module.time,'monotonic',clock)
    monkeypatch.setattr(module.time,'sleep',lambda _:None)


def test_contact_tcp_is_not_offset_again_and_x_scrape_keeps_orientation(monkeypatch):
    args=settings(); module.validate(args); io=IO(args);fast_clock(monkeypatch)
    module.execute(io,args)
    moves=[p for label,p in io.events if label=='move']
    assert len(moves)==2
    assert moves[0][2] == pytest.approx(113.68)  # 준비 Z 133.68에서 -20 탐색 목표. 실제 접촉 시 조기 정지.
    assert moves[1][0] == pytest.approx(-431.88+35)
    assert moves[1][2] == 123.68 and moves[1][3:]==tuple(args.contact_pose[3:])
    names=[name for name,_ in io.events]
    assert names.index('compliance') < names.index('force') < names.index('release') < names.index('lift')


def test_wrong_staging_pose_causes_no_motion():
    args=settings();io=IO(args);io.position=(0.,0.,0.,0.,0.,0.)
    with pytest.raises(ValueError,match='준비 위치'): module.execute(io,args)
    assert not io.events


def test_force_failure_never_scrapes_and_stops_before_release(monkeypatch):
    args=settings();io=IO(args,overshoot=True);fast_clock(monkeypatch)
    with pytest.raises(RuntimeError,match='상한'):module.execute(io,args)
    assert len([x for x in io.events if x[0]=='move'])==1
    assert io.events[-2:]==[('stop',None),('release',None)]


@pytest.mark.parametrize('key,value',[('distance',41),('hold_force',float('nan')),
    ('force_limit',4),('approach_height',20),('search_distance',21)])
def test_invalid_settings_rejected(key,value):
    args=settings();setattr(args,key,value)
    with pytest.raises(ValueError):module.validate(args)


def test_load_action_validates_required_arguments_before_worker():
    goal=SimpleNamespace(task='load',virtual_mode=False,tool_held_confirmed=True,arguments=[])
    with pytest.raises(ValueError):worker_command(goal)


def test_process_load_maps_tcp_goal_and_requires_explicit_confirmation():
    from utils.config import CFG
    cfg=deepcopy(CFG)
    values=vars(settings()).copy()
    values.pop('contact_pose');values.pop('execute_motion');values.pop('distance')
    cfg['force']['putty_load'].update(values,tool_held_confirmed=True)
    req=SimpleNamespace(task_code='PUTTY_LOAD',start_point=[-431.88,-1.52,123.68],
                        end_point=[-396.88,-1.52,123.68])
    goal=process_load_goal(req,cfg)
    assert goal.task=='load' and goal.tool_held_confirmed
    cmd=worker_command(goal)
    assert cmd[4]=='load' and '--edge-offset' not in cmd
    cfg['force']['putty_load']['tool_held_confirmed']=False
    with pytest.raises(ValueError,match='E_GRIP'):process_load_goal(req,cfg)


def test_server_cancel_cleanup_releases_load_force():
    from test_putty_actions import fake_server, FakeWorker, Handle
    from putty_demo.putty_task_server import PuttyTaskServer
    server=fake_server();worker=FakeWorker(server,Handle())
    PuttyTaskServer.cleanup_worker(server,worker,'load')
    assert 'release' in server.events and 'release_comp' in server.events
    assert server.events.index('robot_stop_confirmed') < server.events.index('release')


def test_load_trace_uses_fz_and_completed_feedback(monkeypatch):
    import json
    from test_putty_actions import fake_server, Handle
    from putty_demo import putty_task_server as server_module
    from rokey_interfaces.action import PuttyTask
    server=fake_server();server.process_virtual_mode=False
    server.process_snapshot=lambda:dict(joints=[0.]*6,pose=[0.]*6,force=[0.,0.,-4.,0.,0.,0.])
    def execute(view,release_slot):
        view.observe_line('[판힘진단] '+json.dumps(dict(stage='판 긁기 중',force=[99,0,-4,0,0,0])))
        assert view.fx==[-4]
        view.succeed();result=PuttyTask.Result();result.success=True
        return result
    server.execute=execute
    monkeypatch.setattr(server_module,'process_putty_goal',lambda *a:SimpleNamespace(task='load'))
    from utils.config import CFG
    monkeypatch.setitem(CFG['force']['putty_load'],'hold_force',4)
    monkeypatch.setitem(CFG['force']['putty_load'],'force_tolerance',1)
    handle=Handle();handle.request=SimpleNamespace(task_code='PUTTY_LOAD',run_id=2,gap_id=-1,
        start_point=[-431.88,-1.52,123.68],end_point=[-396.88,-1.52,123.68])
    feedback=[];handle.publish_feedback=feedback.append
    result=server_module.PuttyTaskServer.execute_process(server,handle)
    assert result.success and feedback[0].step=='putty_load'
    assert json.loads(feedback[0].force_stats_json)['fz_mean']==-4
    assert json.loads(feedback[0].params_json)['target_fz']==-4


def test_ros_adapter_builds_real_service_requests_without_robot(monkeypatch):
    import rclpy
    import utils.shutdown
    args=settings();args.joint_speed=10.;args.joint_acceleration=20.;current=[list(module.approach_pose(args))];sent=[]
    class Client:
        def __init__(self,typ,path):self.typ,self.path=typ,path
        def wait_for_service(self,**kw):return True
        def call_async(self,request):
            sent.append((self.path,request))
            if self.path.endswith(('/move_line','/ikin')):current[0]=list(request.pos)
            response=SimpleNamespace(success=True,info='GripperDA_v1',robot_state=1,
                pos=[0.]*6,conv_posj=[0.]*6,sol_space=0,
                task_pos_info=[SimpleNamespace(data=current[0])],tool_force=[0.,0.,-4.,0.,0.,0.])
            return SimpleNamespace(done=lambda:True,result=lambda:response)
    node=SimpleNamespace(create_client=lambda typ,path:Client(typ,path))
    monkeypatch.setattr(rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(rclpy,'create_node',lambda *a,**kw:node)
    monkeypatch.setattr(utils.shutdown,'safe_shutdown',lambda _:None)
    monkeypatch.setattr(module,'start_singularity_watch',lambda _:SimpleNamespace(
        wait_ready=lambda:None,check=lambda:None,close=lambda:None))
    def execute(io,args):
        io.move_joint_pose(module.approach_pose(args))
        io.compliance(args.force_stiffness);io.set_force(args.hold_force)
        io.move_async(args.contact_pose,args.search_speed)
        io.stop_and_wait();io.release()
        io.move_line(module.approach_pose(args),args.lift_speed)
    monkeypatch.setattr(module,'execute',execute)
    module.run_robot(args)
    request=next(req for path,req in sent if path.endswith('/set_desired_force'))
    assert list(request.fd)==[0.,0.,-4.,0.,0.,0.]
    assert list(request.dir)==[0,0,1,0,0,0] and request.ref==request.mod==0
    moves=[req for path,req in sent if path.endswith('/move_line')]
    assert [req.sync_type for req in moves]==[1,0]
    assert all(req.mode==0 and req.ref==0 for req in moves)
    joint=next(req for path,req in sent if path.endswith('/move_joint'))
    assert joint.vel==10 and joint.acc==20 and joint.sync_type==0
    ikin=next(req for path,req in sent if path.endswith('/ikin'))
    assert ikin.ref==0 and ikin.sol_space==0


def test_force_never_forms_prevents_x_scrape(monkeypatch):
    args=settings();io=IO(args);fast_clock(monkeypatch)
    original=io.read_force
    io.read_force=lambda: [0.]*6 if io.phase=='hold' else original()
    with pytest.raises(TimeoutError,match='힘 형성 시간 초과'):
        module.execute(io,args)
    assert len([x for x in io.events if x[0]=='move'])==1
    assert io.events[-2:]==[('stop',None),('release',None)]


def test_cancel_during_contact_hold_stops_and_releases(monkeypatch):
    args=settings();io=IO(args);fast_clock(monkeypatch)
    def canceled(force):
        io.phase='hold'
        raise KeyboardInterrupt()
    io.set_force=canceled
    with pytest.raises(KeyboardInterrupt):module.execute(io,args)
    assert io.events[-2:]==[('stop',None),('release',None)]
    assert not any(label=='lift' for label,_ in io.events)


def test_auto_transfer_raises_rotates_translates_then_descends(monkeypatch):
    args=settings();args.transfer_z=400.;args.transfer_speed=10.;args.joint_speed=10.;args.joint_acceleration=20.
    module.validate(args);io=IO(args);io.position=(300.,100.,220.,0.,170.,0.)
    fast_clock(monkeypatch)
    module.execute(io,args)
    route=[(name,target) for name,target in io.events if name in ('lift','joint_pose','joint')]
    assert route[0]==('lift',(300.,100.,400.,0.,170.,0.))
    assert route[1]==('joint_pose',(-431.88,-1.52,400.,*args.contact_pose[3:]))
    assert route[2]==('lift',module.approach_pose(args))
    assert route[-2:]==[('joint',(300.,100.,400.,0.,170.,0.)),('joint',(300.,100.,220.,0.,170.,0.))]


def test_auto_transfer_failure_stops_before_any_contact_search():
    args=settings();args.transfer_z=400.;args.transfer_speed=10.;args.joint_speed=10.;args.joint_acceleration=20.
    io=IO(args);io.position=(300.,100.,220.,0.,170.,0.)
    def fail(target,speed):raise TimeoutError('transfer failed')
    io.move_line=fail
    with pytest.raises(TimeoutError):module.execute(io,args)
    assert io.events==[('stop',None)]
