"""Exercise the complete workflow with simulated joints and real video encoding only."""
import argparse
import json
from pathlib import Path
import subprocess
import time

from .core import Engine
from .hardware import DemoRobot,DemoCamera
from .storage import load,identifier


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root",type=Path,required=True)
    args = p.parse_args()
    import cv2
    root = args.root/identifier()
    robot = DemoRobot()
    camera = DemoCamera(cv2)
    camera.start()
    e = Engine(robot,root,root/"demo_session.json",camera,cv2,True)
    e.set_home(True)
    e.confirm_holding(True)
    e.release(True)
    e.begin_teaching()
    for i in range(13):
        robot.pose[4] = 90 + min(i,6)
        e.teaching_tick()
        time.sleep(.1)
    e.end_teaching()
    assert not list(root.rglob("*.jpg")),"Manual phase must not save any images"
    assert robot.writes == [],"Manual phase must not send joint targets"
    e.hold_here(True)
    e.confirm_holding(True)
    e.save_teaching()
    e.replay_capture(True)
    first = e.capture
    assert first.meta["status"] == "draft"
    e.save_capture(None)
    assert first.meta["operator_success"] is None
    e.replay_capture(True)
    second = e.capture
    assert first.path.exists() and first.meta["status"] == "saved"
    e.replay_capture(True)
    third = e.capture
    assert not second.path.exists(),"Re-record archives only the unsaved take"
    assert first.path.exists(),"Accepted take must survive re-record"
    e.save_capture(False)
    rows = [json.loads(x) for x in (third.path/"samples.jsonl").read_text().splitlines()]
    probe = subprocess.run(["ffprobe","-v","error","-select_streams","v:0","-count_frames",
                            "-show_entries","stream=nb_read_frames,width,height","-of","json",str(third.path/"video.mp4")],capture_output=True,text=True,check=True)
    info = json.loads(probe.stdout)["streams"][0]
    assert len(rows) == int(info["nb_read_frames"]) == third.meta["frames"]
    assert all(r["frame_index"] == i and (third.path/r["image"]).exists() for i,r in enumerate(rows))
    assert all(r["state_read_end_monotonic_ns"] <= r["action_write_start_monotonic_ns"] for r in rows if r["command_sent"])
    assert all(r["action_write_start_monotonic_ns"] is None and r["action_write_end_monotonic_ns"] is None for r in rows if not r["command_sent"])
    assert all(0 <= r["frame_age_at_state_start_ms"] <= 200 for r in rows)
    e.discard_teaching()
    assert first.path.exists() and third.path.exists(),"Saved episodes embed their own teaching copy"
    e.next_teaching()
    e.close()
    report = {"root":str(root),"simulated":True,"real_motor_commands":0,
              "result":"passed", "video_frames":len(rows),"video_resolution":[info["width"],info["height"]],
              "saved_episodes":[str(first.path),str(third.path)],
              "tested":["manual joint-only teaching","hold at current pose","save teaching","home and replay",
                        "encode real MP4","save capture","re-record unsaved take","preserve accepted takes","delete teaching","next teaching"]}
    (root/"integration_report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
