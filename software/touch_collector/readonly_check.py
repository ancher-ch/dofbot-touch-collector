"""Real sensors, offscreen native UI, actuator methods blocked by a test adapter."""
import argparse
import json
import os
from pathlib import Path
import time
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
from PyQt5 import QtCore,QtWidgets as W
from .app import Worker,Window
from .hardware import RealRobot


class ReadOnlyRobot(RealRobot):
    blocked_requests = 0
    def write(self,*args):
        self.blocked_requests += 1
        raise AssertionError("Actuator request blocked during read-only verification")
    def torque(self,*args):
        self.blocked_requests += 1
        raise AssertionError("Torque request blocked during read-only verification")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root",type=Path,required=True)
    p.add_argument("--screenshot",type=Path,required=True)
    args = p.parse_args()
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QtCore.QLibraryInfo.location(QtCore.QLibraryInfo.PluginsPath)
    app = W.QApplication([])
    worker = Worker(args.root,False)
    window = Window(worker)
    window.show()
    try:
        with patch("touch_collector.app.RealRobot",ReadOnlyRobot):
            worker.start()
            deadline = time.monotonic()+12
            while time.monotonic()<deadline:
                app.processEvents()
                if worker.engine and worker.engine.valid and worker.engine.preview:
                    break
                time.sleep(.05)
            else:
                raise RuntimeError(worker.error or (worker.engine.error if worker.engine else "Device unavailable"))
            e = worker.engine
            r = e.robot
            readings = []
            last = 0
            deadline = time.monotonic()+1.5
            while time.monotonic()<deadline:
                app.processEvents()
                if e.read_at != last:
                    readings.append(e.read_at)
                    last = e.read_at
                time.sleep(.005)
            gaps = [b-a for a,b in zip(readings,readings[1:])]
            assert len(gaps)>=8 and sum(gaps)/len(gaps)<.115,("sampling cadence",gaps)
            assert r.position_requests==r.torque_requests==r.blocked_requests==0
            window.update_state()
            app.processEvents()
            args.screenshot.parent.mkdir(parents=True,exist_ok=True)
            window.grab().save(str(args.screenshot))
            report = {"real_sensors":True,"position_requests":r.position_requests,"torque_requests":r.torque_requests,
                      "blocked_actuator_attempts":r.blocked_requests,"pose":e.pose,"camera_preview":bool(e.preview),
                      "mean_read_interval_s":sum(gaps)/len(gaps),
                      "screenshot":str(args.screenshot),"dataset_written":False}
            (args.root/"readonly_report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print(json.dumps(report,ensure_ascii=False,indent=2))
    finally:
        worker.close()
        window.hide()
        app.quit()


if __name__=="__main__":
    main()
