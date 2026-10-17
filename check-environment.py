"""Read-only prerequisites check: never import/instantiate Arm_Lib, open devices or move motors."""
import argparse
import importlib.util
import os
from pathlib import Path
import platform
import shutil
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--demo', action='store_true')
    mode.add_argument('--real', action='store_true')
    args = parser.parse_args()
    missing = []

    def check(name, ok, note=''):
        print(f"{'OK' if ok else 'MISSING'}  {name}" + (f' — {note}' if note else ''))
        if not ok:
            missing.append(name)

    print('Mode:', 'REAL prerequisites only' if args.real else 'DEMO prerequisites only')
    print('Python:', sys.executable, platform.python_version())
    check('Python >= 3.10', sys.version_info >= (3, 10))
    check('Linux deployment target', platform.system() == 'Linux', 'Windows runtime is not validated')
    for module in ['PyQt5', 'cv2', 'numpy'] + (['Arm_Lib', 'smbus'] if args.real else []):
        try:
            spec = importlib.util.find_spec(module)
        except (ImportError, ValueError):
            spec = None
        check('module ' + module, spec is not None, 'module presence only; no module imported by this check')
    for command in ['ffmpeg', 'ffprobe'] + (['v4l2-ctl'] if args.real else []):
        check(command, shutil.which(command) is not None)
    kit = Path(__file__).resolve().parent
    check('complete software layout', (kit / 'software/touch_collector/app.py').is_file())
    if args.real:
        node = Path('/dev/i2c-7')
        check('/dev/i2c-7 exists', node.exists(), 'the released driver uses I2C bus 7')
        check('/dev/i2c-7 read/write permission', os.access(node, os.R_OK | os.W_OK))
        nodes = sorted(Path('/dev').glob('video[0-9]*'))
        check('video device node', bool(nodes), 'nodes alone do not prove independent RGB cameras or adequate FPS')
        for node in nodes:
            print('INFO ', node, 'read/write:', os.access(node, os.R_OK | os.W_OK))
        print('INFO  display:', os.environ.get('DISPLAY', '(not set; use the robot graphical desktop)'))
    print('No camera/I2C opened. No robot API instantiated. Passing is not hardware compatibility validation.')
    return 1 if missing else 0


if __name__ == '__main__':
    raise SystemExit(main())
