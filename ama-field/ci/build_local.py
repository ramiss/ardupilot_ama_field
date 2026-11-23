#!/usr/bin/env python3
"""
Build AMA field mode firmware locally, exactly as the release CI does.

Run from VS Code: Terminal > Run Task > "AMA field: Build locally" (asks for the
boards, default FlyingRCF4WingMini), or directly:

    python ama-field/ci/build_local.py [BOARD ...]

It builds the working copy as it is, uncommitted changes included, with the CI's
Docker image and ama-field/ci/build_boards.sh: the same field-mode overlay, trim
tiers, file names and feature check. On Windows the build runs in WSL; on Linux it
runs directly. Docker is needed in either case.

The build happens in a Linux clone (~/ama-field-build in WSL) that keeps the
submodules and the compiler cache, so the first run takes a while (clone and
submodules, then a full build) and later runs only rebuild what changed.

Output, as in CI, in ama-field-out/ next to this repo's root:
    files/ArduPlane-<fw>_AMAFieldMode_local-<sha>[-dirty]_<board>.apj / _with_bl.hex
    results/, features/, logs/

AP_FLAKE8_CLEAN
"""
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except (AttributeError, ValueError):
    pass

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BOARDS = ['FlyingRCF4WingMini']
WSL_SCRIPT = 'ama-field/ci/build_local_wsl.sh'
WINDOWS = platform.system() == 'Windows'
# above this many changed files, ship the whole working tree instead of a path list
MAX_PATHS = 400


def git(*args, env=None, check=True):
    result = subprocess.run(['git'] + list(args), cwd=str(REPO_ROOT), capture_output=True, text=True, env=env)
    if check and result.returncode != 0:
        fail('git %s failed:\n%s' % (' '.join(args), (result.stderr or result.stdout).strip()))
    return result.stdout


def fail(msg):
    print('\n[ERROR] %s' % msg, file=sys.stderr)
    sys.exit(1)


def linux_path(path):
    """path as the build script sees it (WSL path on Windows)"""
    if not WINDOWS:
        return str(path)
    # -e: run without the WSL shell, which would eat the backslashes
    out = subprocess.run(['wsl', '-e', 'wslpath', '-a', '-u', str(path).replace('\\', '/')],
                         capture_output=True, text=True)
    if out.returncode != 0 or not out.stdout.strip():
        fail('WSL is needed to build on Windows (wsl wslpath failed: %s)' % out.stderr.strip())
    return out.stdout.strip()


def boards_from_args(argv):
    boards = []
    for a in argv:
        boards += [b for b in a.replace(',', ' ').split() if b]
    return boards or DEFAULT_BOARDS


def snapshot(tmp):
    """the working copy's uncommitted state, as git would commit it (LF line endings):
    (changes.tar or None, deleted.txt or None, dirty)"""
    status = git('status', '--porcelain', '--untracked-files=all')
    if not status.strip():
        return None, None, False
    # a tree object of the whole working copy, built in a temporary index so the
    # real index is untouched; 'git add' applies the same line-ending conversion as
    # a commit
    index = Path(tmp) / 'index'
    real_index = Path(git('rev-parse', '--git-path', 'index').strip())
    if not real_index.is_absolute():
        real_index = REPO_ROOT / real_index
    # start from the real index: its file stats save re-hashing every file
    shutil.copy(str(real_index), str(index))
    env = dict(os.environ, GIT_INDEX_FILE=str(index))
    git('add', '-A', '.', env=env)
    tree = git('write-tree', env=env).strip()

    changed = [p for p in git('diff', '--name-only', '--no-renames', '--diff-filter=d', 'HEAD', tree).splitlines() if p]
    deleted = [p for p in git('diff', '--name-only', '--no-renames', '--diff-filter=D', 'HEAD', tree).splitlines() if p]

    tar = None
    if changed:
        tar = Path(tmp) / 'changes.tar'
        args = ['archive', '--format=tar', '-o', str(tar), tree]
        if len(changed) <= MAX_PATHS:
            args += ['--'] + changed
        git(*args)
    dele = None
    if deleted:
        dele = Path(tmp) / 'deleted.txt'
        dele.write_text('\n'.join(deleted) + '\n', encoding='utf-8')
    print('      %u changed or new file(s), %u deleted, not committed: included in the build'
          % (len(changed), len(deleted)))
    return tar, dele, True


def read_results(out):
    rows = []
    for f in sorted((out / 'results').glob('*.tsv')):
        if f.name.endswith('.removed.tsv'):
            continue
        fields = f.read_text(encoding='utf-8').rstrip('\n').split('\t')
        rows.append((fields + [''] * 7)[:7])
    return rows


def main():
    boards = boards_from_args(sys.argv[1:])
    print('\n=== AMA field mode: local build ===\n')
    print('Boards: %s' % ' '.join(boards))

    sha = git('rev-parse', '--short=7', 'HEAD').strip()
    branch = git('branch', '--show-current').strip() or '(detached)'
    with tempfile.TemporaryDirectory(prefix='ama-build-') as tmp:
        print('[1/3] Working copy: %s at %s' % (branch, sha))
        tar, deleted, dirty = snapshot(tmp)
        version = 'local-%s%s' % (sha, '-dirty' if dirty else '')
        print('      Version: %s' % version)

        print('[2/3] Building in %s (same Docker image and script as the release CI)...'
              % ('WSL' if WINDOWS else 'Linux'))
        script = linux_path(REPO_ROOT / WSL_SCRIPT)
        args = [linux_path(REPO_ROOT), linux_path(tar) if tar else '-', linux_path(deleted) if deleted else '-',
                version] + boards
        cmd = (['wsl', '-e', 'bash', script] if WINDOWS else ['bash', script]) + args
        rc = subprocess.run(cmd, cwd=str(REPO_ROOT)).returncode

    out = REPO_ROOT / 'ama-field-out'
    print('\n[3/3] Results (%s)' % out)
    rows = read_results(out) if out.is_dir() else []
    if not rows:
        fail('no results: see the output above%s' % (' (exit code %d)' % rc if rc else ''))
    print('  %-24s %-6s %-12s %12s %12s  %s' % ('Board', 'Status', 'Build', 'Flash used', 'Flash free', 'Note'))
    failed = False
    for board, status, build, used, free, files, reason in rows:
        failed |= status != 'ok'
        print('  %-24s %-6s %-12s %12s %12s  %s' % (board, status, build, used, free, reason))
    files = sorted((out / 'files').glob('*')) if (out / 'files').is_dir() else []
    if files:
        print('\n  Files:')
        for f in files:
            print('    %s' % f)
    print('\n  Logs: %s' % (out / 'logs'))
    sys.exit(1 if failed or rc else 0)


if __name__ == '__main__':
    main()
