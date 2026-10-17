"""Verify shipped files only; does not import the application or access hardware."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / 'PACKAGE-MANIFEST.json').read_text(encoding='utf-8'))
    bad = []
    for name, expected in manifest['files'].items():
        path = (root / name).resolve()
        try:
            path.relative_to(root)
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != expected:
                bad.append(name + ': hash mismatch')
        except (OSError, ValueError):
            bad.append(name + ': missing or invalid path')
    if bad:
        print('\n'.join(bad))
        return 1
    print(f"PASS: {len(manifest['files'])} shipped files match SHA256; software {manifest['software_version']}")
    print('Files created later during use are not covered. Hashes verify transfer integrity, not publisher identity.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
