"""Installed SDK + fake bus and encoded simulated capture. Never opens I2C."""
import argparse
import json
from pathlib import Path
import time
import cv2
from .hardware import PacedBus, RealRobot, DemoRobot, FeedbackError
from .core import Engine
from .multicamera import make_rig
from .storage import load


class FakeReplyBus:
    def __init__(self, robot):
        self.robot=robot
        self.mode="delayed"
        self.query=None
        self.reads=0
        self.query_count=0
        self.j6_attempts=0
        self.forbidden_attempts=0

    def write_byte_data(self,addr,reg,value):
        if addr!=0x15 or reg not in range(0x31,0x37) or value!=0:
            self.forbidden_attempts+=1
            raise AssertionError("Non-query write in fake SDK bus")
        self.query=(addr,reg);self.reads=0;self.query_count+=1
        if reg==0x31:self.j6_attempts=0
        if reg==0x36:self.j6_attempts+=1

    def read_word_data(self,addr,reg):
        assert (addr,reg)==self.query
        self.reads+=1
        if reg==0x36:
            if self.mode=="persistent" or self.mode=="delayed" and self.j6_attempts<=2:return 0
            if self.mode=="out_of_range":return 65535
        j=reg-0x31;q=self.robot.pose[j]
        if j in (1,2,3):q=180-q
        raw=int(380+q*3320/270) if j==4 else int(900+q*2200/180)
        return ((raw&255)<<8)|(raw>>8)

    def close(self):
        pass


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();args.root.mkdir(parents=True,exist_ok=True)
    # Importing a class does not open a device; never call Arm_Device.__init__.
    from Arm_Lib import Arm_Device
    robot=DemoRobot();robot.pose=[90,120,35,25,90,100]
    fake=FakeReplyBus(robot)
    arm=Arm_Device.__new__(Arm_Device);arm.addr=0x15;arm.bus=fake
    fake.mode="healthy"
    baseline=arm.Arm_serial_servo_read(6)
    fake.mode="delayed"
    fake.j6_attempts=0
    assert arm.Arm_serial_servo_read(6) is None
    assert arm.Arm_serial_servo_read(6) is None
    arm.bus=PacedBus(fake)
    adapter=RealRobot.__new__(RealRobot);adapter.arm=arm
    assert adapter.read()[5]==baseline
    trace=[t for t in adapter.last_feedback_io if t['joint']==6]
    assert [t['requested_wait_ms'] for t in trace]==[3,6,10]
    assert all(len(t['words'])==1 for t in trace)
    checks=["two_failed_queries_reproduce_old_two_attempt_limit", "fresh_queries_with_adaptive_wait_recover_and_preserve_sdk_decoding"]
    fake.mode="out_of_range"
    assert arm.Arm_serial_servo_read(6) is None
    checks.append("sdk_still_rejects_nonzero_out_of_range_reply")
    fake.mode="persistent"
    try:
        adapter.read()
    except FeedbackError as error:
        assert error.pose[5] is None
    else:raise AssertionError("Persistent failure accepted")
    checks.append("persistent_failure_is_not_filled_from_cache_or_diagnostic_angle")
    fake.mode="delayed"
    def read(bounds=None):
        try:return adapter.read(bounds)
        finally:
            robot.last_feedback_io=adapter.last_feedback_io
            robot.last_read_rechecks=adapter.last_read_rechecks
    robot.read=lambda:read()
    robot.read_with_bounds=read
    camera=make_rig(cv2,{'mode':'single','wrist':'/dev/video0','overview':None},True)
    camera.start()
    engine=Engine(robot,args.root/'data',args.root/'session.json',camera,cv2,True)
    try:
        document=engine.store.new_teaching("模拟：SDK延迟回复",robot.pose)
        document.update(status="recorded",simulated=True,quality_warnings=[])
        document['samples']=[{'t':i/10,'q':[90,120,35,25,90,g]} for i,g in
                             enumerate([100,100,102,104,106,106,106,104,102,100,100,100])]
        engine.store.write_teaching(document)
        engine.teaching=document;engine.holding='confirmed';engine.stage='saved_teach'
        engine.replay_capture(True,gripper_extra_degrees=3,joint3_offset_degrees=0)
        assert engine.capture.meta['status']=='draft' and engine.capture.meta['capture_checks_passed']
        rows=[json.loads(line) for line in (engine.capture.path/'samples.jsonl').read_text().splitlines()]
        assert all(len(row['observation.state'])==6 for row in rows)
        assert max(row['action'][5] for row in rows)==109
        assert load(engine.capture.path/'teaching.json')==document
        successful_frames=len(rows)
        checks.append("delayed_sdk_replies_survive_full_encoded_capture_with_real_state_action_timestamps")
        # Inject persistent transport failure during a second simulated replay.
        trigger=len(robot.writes)+4;original_write=robot.write
        def write(pose,duration):
            original_write(pose,duration)
            if len(robot.writes)>=trigger:fake.mode='persistent'
        robot.write=write
        try:engine.replay_capture(True,gripper_extra_degrees=3)
        except FeedbackError:pass
        else:raise AssertionError("Persistent mid-capture failure accepted")
        assert engine.capture.meta['status']=='error' and not engine.capture.meta['capture_checks_passed']
        assert engine.capture.meta['operator_success'] is None
        checks.append("persistent_mid_capture_failure_still_stops_and_cannot_be_saved_as_complete")
        assert robot.torques==[] and fake.forbidden_attempts==0
        report={'passed':True,'simulated':True,'uses_installed_sdk':True,'hardware_devices_opened':0,
                'real_position_requests':0,'real_torque_requests':0,'fake_bus_query_count':fake.query_count,
                'encoded_frames':successful_frames,'checks':checks}
        (args.root/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False),flush=True)
    finally:engine.close()


if __name__=='__main__':main()
