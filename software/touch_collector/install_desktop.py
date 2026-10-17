"""Install launchers for the current local app. No system Python or startup changes."""
from pathlib import Path
import os
import shutil
import subprocess


def main():
    here = Path(__file__).resolve().parent
    launcher = here/"launch.sh"
    launcher.chmod(0o755)
    applications = Path.home()/".local/share/applications"
    applications.mkdir(parents=True,exist_ok=True)
    desktop = Path.home()/"Desktop"
    desktop.mkdir(exist_ok=True)
    text = ("[Desktop Entry]\nType=Application\nVersion=1.0\n"
            "Name=DOFBOT 数据采集台\nComment=手动关节示教与同步视频动作采集\n"
            f'Exec="{launcher}"\nPath={here.parent}\nIcon=applications-engineering\n'
            "Terminal=false\nCategories=Education;Science;\nStartupNotify=true\n")
    for path in [applications/"dofbot-collector.desktop",desktop/"DOFBOT 数据采集台.desktop"]:
        path.write_text(text,encoding="utf-8")
        path.chmod(0o755)
        if shutil.which("gio"):
            env = os.environ.copy()
            bus = Path(f"/run/user/{os.getuid()}/bus")
            if bus.exists():
                env.setdefault("DBUS_SESSION_BUS_ADDRESS",f"unix:path={bus}")
            subprocess.run(["gio","set",str(path),"metadata::trusted","true"],capture_output=True,env=env)
        print(path)


if __name__ == "__main__":
    main()
