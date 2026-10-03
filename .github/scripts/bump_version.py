"""Auto-increment the published patch version for each successful dev build."""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'usb_manager/config.yaml'
CHANGELOG = ROOT / 'usb_manager/CHANGELOG.md'


def parse_version(value):
    match = re.fullmatch(r'(\d+)\.(\d+)\.(\d+)', value)
    if not match:
        raise ValueError('Version must use major.minor.patch')
    return tuple(int(part) for part in match.groups())


def next_version(current, tags):
    baseline = parse_version(current)
    released = [parse_version(tag[1:]) for tag in tags if re.fullmatch(r'v\d+\.\d+\.\d+', tag)]
    latest = max(released, default=None)
    if latest is None or baseline > latest:
        return current
    major, minor, patch = latest
    return f'{major}.{minor}.{patch + 1}'


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def main():
    text = CONFIG.read_text(encoding='utf-8')
    match = re.search(r'^version:\s*"?([^"\n]+)', text, re.M)
    if not match:
        raise SystemExit('Missing add-on version')
    version = next_version(match.group(1).strip(), git('tag', '--list', 'v*').splitlines())
    text = re.sub(r'^version:.*$', f'version: "{version}"', text, flags=re.M)
    CONFIG.write_text(text, encoding='utf-8', newline='\n')
    changelog = CHANGELOG.read_text(encoding='utf-8')
    if not re.search(rf'^## {re.escape(version)}\s*$', changelog, re.M):
        commit = git('rev-parse', 'HEAD')
        entry = f'## {version}\n\n- Maintenance update.\n- Source: [{commit[:7]}](https://github.com/thaihoang987/addon-usb-manager/commit/{commit}).\n\n'
        first_entry = re.search(r'^## ', changelog, re.M)
        if first_entry:
            changelog = changelog[:first_entry.start()] + entry + changelog[first_entry.start():]
        else:
            changelog += '\n' + entry
        CHANGELOG.write_text(changelog, encoding='utf-8', newline='\n')
    print(version)


if __name__ == '__main__':
    main()
