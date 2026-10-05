#!/usr/bin/env python3
"""Inspect or switch only the recorded plane-extraction files; never restart ROS."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def git(root, *args):
    result = subprocess.run(['git', '-C', str(root), *args], capture_output=True)
    if result.returncode:
        raise ValueError(result.stderr.decode(errors='replace').strip())
    return result.stdout


def snapshot(root, ref, paths):
    try:
        commit = git(root, 'rev-parse', '--verify', ref+'^{commit}').decode().strip()
    except ValueError:
        if not ref.startswith('refs/tags/plane-extraction/'):
            raise
        remote = ref.replace('refs/tags/', 'refs/remotes/origin/', 1)
        commit = git(root, 'rev-parse', '--verify', remote+'^{commit}').decode().strip()
    entries = {}
    for line in git(root, 'ls-tree', '-r', '-z', commit, '--', *paths).split(b'\0'):
        if not line:
            continue
        info, name = line.split(b'\t', 1)
        mode, kind, blob = info.split()
        if kind != b'blob' or mode not in (b'100644', b'100755'):
            raise ValueError('Unsupported snapshot file: '+name.decode())
        entries[name.decode()] = (git(root, 'cat-file', 'blob', blob.decode()), int(mode, 8) & 0o777)
    return {path: entries.get(path) for path in paths}


def current_files(root, paths):
    result = {}
    for name in paths:
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Invalid managed path: '+name)
        path = root/relative
        if any(part.is_symlink() for part in [path, *path.parents] if part != root.parent):
            raise ValueError('Refusing symlink path: '+name)
        if path.exists() and not path.is_file():
            raise ValueError('Not a regular file: '+name)
        mode = 0o755 if path.exists() and path.stat().st_mode & 0o111 else 0o644
        result[name] = (path.read_bytes(), mode) if path.exists() else None
    return result


def write_files(root, files):
    for name, value in files.items():
        path = root/name
        if value is None:
            path.unlink(missing_ok=True)
        else:
            content, mode = value
            path.parent.mkdir(parents=True, exist_ok=True)
            fd, temp = tempfile.mkstemp(prefix='.plane-version-', dir=path.parent)
            try:
                with os.fdopen(fd, 'wb') as stream:
                    stream.write(content)
                permissions = (path.stat().st_mode & 0o666) if path.exists() else 0o644
                os.chmod(temp, permissions | (mode & 0o111))
                os.replace(temp, path)
            finally:
                Path(temp).unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('list')
    sub.add_parser('status')
    switch = sub.add_parser('switch')
    switch.add_argument('version')
    switch.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    config = json.loads((root/'config/plane_extraction_versions.json').read_text())
    paths, versions = config['paths'], config['versions']
    current = current_files(root, paths)
    saved = {key: snapshot(root, item['ref'], paths) for key, item in versions.items()}
    active = next((key for key, files in saved.items() if files == current), None)
    if args.command == 'list':
        print(json.dumps({'versions': [dict(version=key, active=key == active, **item)
                                      for key, item in versions.items()]}, ensure_ascii=False, indent=2))
    elif args.command == 'status':
        print(json.dumps({'version': active or 'modified',
                          'restart_required_after_switch': True}, ensure_ascii=False))
    else:
        if args.version not in versions:
            raise ValueError('Unknown version: '+args.version)
        if active is None:
            unknown = [p for p in paths if not any(current[p] == files[p] for files in saved.values())]
            raise ValueError('Unrecorded changes or mixed versions; preserve them before switching: '
                             +', '.join(unknown or paths))
        target = saved[args.version]
        changed = {path: value for path, value in target.items() if value != current[path]}
        if not args.dry_run:
            # Validate every file before the first write, including concurrent edits.
            if current_files(root, paths) != current:
                raise ValueError('Files changed during version selection; retry after preserving edits')
            try:
                write_files(root, changed)
            except OSError:
                write_files(root, {path: current[path] for path in changed})
                raise
        print(json.dumps({'from': active, 'to': args.version, 'dry_run': args.dry_run,
                          'changed': list(changed),
                          'note': 'Only source files switched. Restart perception to load them.'},
                         ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
