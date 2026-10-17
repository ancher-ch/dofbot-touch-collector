"""Check tracked files for publication hazards without printing matched secrets."""
import ipaddress
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
FORBIDDEN_PARTS = {'.ssh', '__pycache__', 'datasets', 'episodes', 'teachings', 'trash', 'exports',
                   'reports', 'demo-data', 'touch_demo_data', 'test-results', 'dofbot-data'}
FORBIDDEN_NAMES = {'.env', '.git-credentials', '.learning_session.json', '.teleop_record.lock',
                   'home.json', 'work.json', 'safety.json', 'camera_config.json', 'demo_session.json',
                   'samples.jsonl', 'id_rsa', 'id_ed25519'}
SUFFIXES = {'.key', '.pem', '.p12', '.pfx', '.mp4', '.avi', '.mov', '.jpg', '.jpeg', '.png', '.zip', '.log'}
PATTERNS = [
    ('private key header', re.compile(r'-----BEGIN (?:[A-Z ]+)?PRIVATE KEY-----')),
    ('GitHub credential', re.compile(r'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})')),
    ('AWS access key', re.compile(r'(?:AKIA|ASIA)[A-Z0-9]{16}')),
    ('credential in URL', re.compile(r'https?://[^\s/:@]+:[^\s/@]+@')),
    ('personal Windows path', re.compile(r'[A-Za-z]:[\\/]+Users[\\/]+[^\\/\s]+')),
]

def inspect(name, data):
    problems = []
    p = Path(name)
    if (set(p.parts) & FORBIDDEN_PARTS or p.name in FORBIDDEN_NAMES or
        (p.name.startswith('.env.') and p.name != '.env.example') or p.suffix.lower() in SUFFIXES):
        problems.append('forbidden file type/state path')
    try:
        text = data.decode('utf-8')
    except UnicodeDecodeError:
        return problems + ['unexpected binary content']
    for label, pattern in PATTERNS:
        if pattern.search(text):
            problems.append(label)
    for match in re.finditer(r'(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])', text):
        try:
            ip = ipaddress.ip_address(match.group())
            if (ip.is_private or ip.is_link_local) and not ip.is_loopback and str(ip) not in ('0.0.0.0', '255.255.255.255'):
                problems.append('embedded private network address')
        except ValueError:
            pass
    return sorted(set(problems))

def main():
    result = subprocess.run(['git', '-C', str(ROOT), 'ls-files', '-z'], capture_output=True, check=True)
    names = [x.decode('utf-8') for x in result.stdout.split(b'\0') if x]
    if not names:
        print('No tracked/staged files; stage an explicit reviewed file list first.', file=sys.stderr)
        return 2
    failures = []
    for name in names:
        p = ROOT / name
        if p.is_symlink():
            failures.append((name, ['symlink not allowed']))
            continue
        try:
            content = subprocess.run(['git', '-C', str(ROOT), 'show', ':' + name], capture_output=True, check=True).stdout
            problems = inspect(name, content)
        except (OSError, subprocess.CalledProcessError):
            problems = ['cannot read staged file']
        if problems:
            failures.append((name, problems))
    for name, problems in failures:
        print(name + ': ' + ', '.join(problems))  # Never print matching data.
    if not failures:
        print(f'PASS: {len(names)} staged/tracked files; no publication-policy findings. Also run Gitleaks.')
    return 1 if failures else 0

if __name__ == '__main__':
    raise SystemExit(main())
