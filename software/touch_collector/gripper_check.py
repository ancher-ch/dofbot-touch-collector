"""Touchscreen compensation choices + two encoded captures, demo hardware only."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
from PyQt5 import QtCore,QtWidgets as W,QtTest
from .app import Worker,Window
from .storage import load

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--shots',type=Path,required=True)
    args=parser.parse_args()
    args.shots.mkdir(parents=True,exist_ok=True)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']=QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app=W.QApplication([])
    worker=Worker(args.root,True);window=Window(worker)
    window.show();worker.start()
    checks=[]
    def wait(predicate,timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():
                window.update_state();app.processEvents();return
            time.sleep(.01)
        raise AssertionError(worker.engine.message if worker.engine else worker.error)
    def choose(extra,execute):
        errors=[]
        seen=[]
        def accept():
            try:
                dialog=app.activeModalWidget();assert dialog
                combo=dialog.findChild(W.QComboBox,'gripperCompensation')
                if combo is not None:
                    if not seen:assert combo.currentIndex()==window.replay_gripper_extra
                    seen.append('settings')
                    combo.setCurrentIndex(extra)
                    dialog.grab().save(str(args.shots/f'choice_{extra}_{execute}.png'))
                    if not execute:
                        dialog.reject();return
                    QtCore.QTimer.singleShot(80,accept)
                    button=next(b for b in dialog.findChildren(W.QPushButton) if b.text()=='下一步：检查回放路径')
                else:
                    seen.append('path')
                    assert any(f'夹爪夹紧补偿：{extra}°' in label.text() for label in dialog.findChildren(W.QLabel))
                    for box in dialog.findChildren(W.QCheckBox):box.setChecked(True)
                    button=next(b for b in dialog.findChildren(W.QPushButton) if b.text()=='确认执行')
                QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
            except BaseException as error:
                errors.append(str(error))
                if app.activeModalWidget():app.activeModalWidget().reject()
        QtCore.QTimer.singleShot(80,accept)
        button=window.actions['replay'] if worker.engine.stage=='saved_teach' else window.actions['rerecord']
        assert button.isVisible() and button.isEnabled()
        QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
        assert not errors,errors
        assert seen==(['settings','path'] if execute else ['settings']),seen
    try:
        wait(lambda:worker.engine is not None and worker.engine.valid)
        e=worker.engine
        e.robot.pose[5]=100
        doc=e.store.new_teaching('模拟：方块抓取补偿测试',e.robot.pose)
        doc.update(status='recorded',simulated=True,quality_warnings=[])
        doc['samples']=[{'t':i/10,'q':[90,120,35,25,90,g]} for i,g in
                        enumerate([100,100,102,104,106,106,106,104,102,100,100,100])]
        source=copy.deepcopy(doc)
        e.store.write_teaching(doc)
        e.teaching=doc;e.stage='saved_teach';e.holding='confirmed';e.persist()
        path=e.store.item('teachings',doc['id'])/'trajectory.json'
        source_hash=hashlib.sha256(path.read_bytes()).hexdigest()
        wait(lambda:e.pose[5]==100)
        assert window.replay_gripper_extra==1
        choose(2,False)
        assert not worker.pending and e.robot.writes==e.robot.torques==[]
        checks.append('cancel_choice_does_not_actuate')
        episodes=[]
        for extra in (1,2):
            choose(extra,True)
            wait(lambda:not worker.pending and e.stage=='capture_review',25)
            assert e.capture.meta['status']=='draft',e.message
            rows=[json.loads(x) for x in (e.capture.path/'samples.jsonl').read_text().splitlines()]
            assert max(row['action'][5] for row in rows)==106+extra
            assert rows[0]['action'][5]==rows[-1]['action'][5]==100
            assert load(e.capture.path/'teaching.json')==source
            assert e.capture.meta['gripper_compensation']['max_extra_degrees']==extra
            assert hashlib.sha256(path.read_bytes()).hexdigest()==source_hash
            assert e.capture.meta['capture_checks_passed']
            episodes.append({'id':e.capture.id,'frames':len(rows),'extra_degrees':extra,
                             'peak_command':max(row['action'][5] for row in rows)})
            window.grab().save(str(args.shots/f'capture_{extra}.png'))
        assert e.robot.torques==[] and 'Arm_Lib' not in sys.modules
        checks.extend(['one_and_two_degree_choices_reach_bus_targets','raw_teaching_unchanged',
                       'episode_actions_and_policy_match','opening_preserved','video_encoding_passed'])
        report={'passed':True,'simulated':True,'real_motor_requests':0,'checks':checks,'episodes':episodes}
        (args.shots/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:
        worker.close();window.hide();app.quit()

if __name__=='__main__':main()
