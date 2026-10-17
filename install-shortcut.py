"""Create user launchers only. Does not run the collector or change robot state."""
import argparse
import os
from pathlib import Path
import platform
import shutil
import subprocess


def desktop_entry(kit, real):
    launcher = kit / ('launch-robot.sh' if real else 'launch-demo.sh')
    # Desktop Entry Exec quoting requires escaping these characters (not shell quoting).
    escaped = str(launcher).replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$').replace('%', '%%')
    title = 'DOFBOT 采集台 · 真机' if real else 'DOFBOT 采集台 · 模拟'
    return ('[Desktop Entry]\nType=Application\nVersion=1.0\n'
            f'Name={title}\nComment=DOFBOT data collector v0.1.7.post1\n'
            f'Exec=/bin/sh "{escaped}"\n'
            'Icon=applications-engineering\nTerminal=false\nCategories=Education;Science;\nStartupNotify=true\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real', action='store_true', help='create REAL hardware shortcut; default is DEMO')
    args = parser.parse_args()
    if platform.system() != 'Linux':
        parser.error('Run on the robot Linux desktop, not Windows.')
    kit = Path(__file__).resolve().parent
    desktop = Path.home() / 'Desktop'
    if shutil.which('xdg-user-dir'):
        result = subprocess.run(['xdg-user-dir', 'DESKTOP'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0 and result.stdout.strip():
            desktop = Path(result.stdout.strip())
    appdir = Path.home() / '.local/share/applications'
    name = 'dofbot-share-real.desktop' if args.real else 'dofbot-share-demo.desktop'
    content = desktop_entry(kit, args.real)
    for folder in (appdir, desktop):
        folder.mkdir(parents=True, exist_ok=True)
        dest = folder / name
        dest.write_text(content, encoding='utf-8')
        dest.chmod(0o755)
        if shutil.which('gio'):
            env = os.environ.copy()
            bus = Path(f'/run/user/{os.getuid()}/bus')
            if bus.exists():
                env.setdefault('DBUS_SESSION_BUS_ADDRESS', f'unix:path={bus}')
            subprocess.run(['gio', 'set', str(dest), 'metadata::trusted', 'true'], capture_output=True, env=env, timeout=5)
        print(dest)
    print('Shortcut created. The application was not started. Keep this kit at the same path.')


if __name__ == '__main__':
    main()
