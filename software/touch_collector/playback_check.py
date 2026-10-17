"""Exercise saved-video confirmation and playback with fake hardware only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtCore,QtWidgets as W,QtTest
from .app import Worker,Window
from .playback import VideoDialog,Clip,timeline


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',required=True,type=Path)
    parser.add_argument('--shots',required=True,type=Path)
    args=parser.parse_args()
    args.shots.mkdir(parents=True,exist_ok=True)
    import cv2,numpy as np
    for k in range(2):
        path=args.root/'episodes'/f'20261017T08000{k}Z_test{k}'
        (path/'images').mkdir(parents=True)
        meta={'status':'saved','frames':8,'task':'模拟数据：观看功能检查','operator_success':[None,True][k]}
        (path/'metadata.json').write_text(json.dumps(meta),encoding='utf-8')
        rows=[]
        for i in range(8):
            image=np.full((480,640,3),(80+k*80,100,160),np.uint8)
            cv2.rectangle(image,(50+i*30,200),(100+i*30,250),(230,230,230),-1)
            cv2.imwrite(str(path/f'images/{i:06d}.jpg'),image)
            rows.append({'frame_index':i,'timestamp':i/10,'frame_receive_monotonic_ns':1_000_000_000+i*100_000_000-(33_000_000 if i%2 else 0),
                         'image':f'images/{i:06d}.jpg','observation.state':[90]*6,'action':[90]*6})
        (path/'samples.jsonl').write_text('\n'.join(json.dumps(x) for x in rows),encoding='utf-8')
    def hashes():
        return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.root/'episodes').rglob('*') if p.is_file()}
    before=hashes()
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']=QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app=W.QApplication([])
    worker=Worker(args.root,True)
    window=Window(worker);window.show();worker.start()
    errors=[]
    checks=[]
    def wait(predicate,timeout=8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():return
            time.sleep(.01)
        raise AssertionError(worker.error or 'timeout')
    def guard(callback):
        def wrapped():
            try:callback()
            except BaseException as error:
                errors.append(repr(error))
                modal=app.activeModalWidget()
                if modal:modal.reject()
        return wrapped
    def gate(accept):
        dialog=app.activeModalWidget()
        assert dialog is not None and not isinstance(dialog,VideoDialog)
        boxes=dialog.findChildren(W.QCheckBox)
        go=next(b for b in dialog.findChildren(W.QPushButton) if b.text()=='开始观看')
        assert len(boxes)==2 and not go.isEnabled()
        if accept:
            boxes[0].setChecked(True);assert not go.isEnabled()
            boxes[1].setChecked(True);assert go.isEnabled()
            dialog.grab().save(str(args.shots/'01_fixed_confirmation.png'))
            go.click()
        else:dialog.reject()
    try:
        def watchdog():
            errors.append('viewer test exceeded timeout')
            modal=app.activeModalWidget()
            if modal:modal.reject()
        watchdog_timer=QtCore.QTimer()
        watchdog_timer.setSingleShot(True)
        watchdog_timer.timeout.connect(watchdog)
        watchdog_timer.start(10000)
        wait(lambda:worker.engine is not None and worker.engine.valid)
        window.update_state()
        QtCore.QTimer.singleShot(80,guard(lambda:gate(False)))
        window.saved_videos_btn.click()
        assert not worker.viewing.is_set()
        checks.append('cancel confirmation does not enter viewing')
        def opened():
            dialog=app.activeModalWidget()
            assert isinstance(dialog,VideoDialog)
            assert worker.viewing.is_set() and dialog.items.count()==2
            assert not worker.submit('return_home',confirmed=True)
            assert not worker.submit('release',confirmed=True)
            try:worker.command('hold_here',{'confirmed':True})
            except RuntimeError:pass
            else:raise AssertionError('viewing did not block direct command dispatch')
            assert worker.engine.robot.writes+worker.engine.robot.torques==[]
            assert abs(dialog.player.clip.times[1]-.067)<1e-6
            dialog.player.play.click()
            def played():
                assert dialog.player.index>=2
                dialog.player.pause()
                at=dialog.player.index
                assert not dialog.player.timer.isActive()
                dialog.player.slider.setValue(7)
                assert dialog.player.index==7
                dialog.player.step(-1);assert dialog.player.index==6
                dialog.player.speed.setCurrentIndex(0)
                dialog.items.setCurrentRow(1)
                assert dialog.player.index==0 and not dialog.player.playing
                dialog.grab().save(str(args.shots/'02_saved_video_library.png'))
                checks.extend(['play advances on recorded image time','pause','seek','frame step','speed','select older saved video','controls locked while watching'])
                dialog.accept()
            QtCore.QTimer.singleShot(400,guard(played))
        QtCore.QTimer.singleShot(80,guard(lambda:gate(True)))
        QtCore.QTimer.singleShot(300,guard(opened))
        window.saved_videos_btn.click()
        wait(lambda:not worker.pending and not worker.viewing.is_set())
        assert not errors,errors
        assert worker.engine.robot.writes+worker.engine.robot.torques==[]
        assert worker.engine.teaching is None and worker.engine.capture is None
        assert hashes()==before
        # Verify MP4 decoding when raw JPEG files are absent.
        source=args.root/'episodes/20261017T080000Z_test0'
        fallback=args.root/'video_only';fallback.mkdir()
        shutil.copy2(source/'samples.jsonl',fallback/'samples.jsonl')
        subprocess.run(['ffmpeg','-nostdin','-loglevel','error','-framerate','10','-i',str(source/'images/%06d.jpg'),'-c:v','libx264','-threads','2','-pix_fmt','yuv420p',str(fallback/'video.mp4')],check=True)
        clip=Clip(fallback,cv2)
        assert not clip.pixmap(0).isNull() and not clip.pixmap(6).isNull()
        clip.close()
        checks.append('MP4 fallback without raw JPEGs')
        bad=args.root/'bad';bad.mkdir()
        (bad/'samples.jsonl').write_text(json.dumps({'frame_index':0,'timestamp':0,'image':'../../outside.jpg'}))
        try:Clip(bad,cv2)
        except ValueError:pass
        else:raise AssertionError('image path escaped episode')
        worker.engine.holding='free';window.update_state()
        assert not window.saved_videos_btn.isEnabled()
        checks.extend(['reject path escape','free arm cannot enter viewer','no dataset mutation','no motion or torque requests','current work unchanged'])
        watchdog_timer.stop()
        report={'result':'passed','simulated':True,'real_position_requests':0,'real_torque_requests':0,'checks':checks}
        (args.shots/'playback_report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:
        worker.close();window.hide();app.quit()


if __name__=='__main__':main()
