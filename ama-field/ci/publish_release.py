#!/usr/bin/env python3
"""
Publish all AMA field mode builds as a GitHub Release.

Run from VS Code: Terminal > Run Task > "AMA field: Publish all builds", or
directly: python ama-field/ci/publish_release.py

It walks through:

  1. the working tree is clean;
  2. origin is a GitHub remote;
  3. the branch is ama-field-4.7.2 (asks before using another one);
  4. the branch is pushed: the tag must point at a commit that is on GitHub;
  5. asks for the tag (suggests the next ama-field-v<N>), and a one-line message;
  6. creates the annotated tag and pushes it.

The pushed tag starts .github/workflows/ama_field_release.yml, which builds every
board in ama-field/ci/boards.txt and publishes the Release for that tag
(roughly 30-60 minutes). See ama-field/docs/DEVELOPMENT.md, "Cutting a release".

If the tag already exists, it offers to move it to the current commit and push it
again, which rebuilds and replaces that release's notes and files.

Non-interactive use: set RELEASE_TAG (e.g. 17 or ama-field-v17) and optionally
RELEASE_NOTES; the final confirmation is still asked unless RELEASE_YES=1.
--dry-run goes through every check and question but creates and pushes nothing
(uncommitted changes are reported instead of stopping it).

AP_FLAKE8_CLEAN
"""
import os
import re
import subprocess
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except (AttributeError, ValueError):
    pass

REPO_ROOT = Path(__file__).resolve().parents[2]
RELEASE_BRANCH = 'ama-field-4.7.2'
TAG_PREFIX = 'ama-field-v'
# the workflow's tag filter is 'ama-field-v*'; the part after the prefix becomes the
# version in the file names, so keep it to file-name-safe characters
TAG_RE = re.compile(r'^ama-field-v\d+[0-9A-Za-z.\-]*$')
HISTORY = REPO_ROOT / 'ama-field' / 'docs' / 'DEVELOPMENT.md'


def run(cmd, check=True):
    result = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True)
    if check and result.returncode != 0:
        fail('%s failed:\n%s' % (' '.join(cmd), (result.stderr or result.stdout).strip()))
    return result.stdout.strip()


def fail(msg):
    print('\n[ERROR] %s' % msg, file=sys.stderr)
    sys.exit(1)


def ask(prompt, default=''):
    # full line first, then read on the next line: task terminals flush whole lines
    print(prompt + (' [%s]' % default if default else ''), flush=True)
    sys.stdout.write('> ')
    sys.stdout.flush()
    try:
        # some Windows consoles and pipes prefix a byte-order mark
        answer = input().replace('﻿', '').strip()
    except (EOFError, KeyboardInterrupt):
        print('\nAborted.')
        sys.exit(1)
    return answer or default


def confirm(prompt):
    if os.environ.get('RELEASE_YES', '').strip().lower() in ('1', 'true', 'yes', 'y'):
        print('%s [y/N] y (RELEASE_YES)' % prompt)
        return True
    return ask(prompt + ' [y/N]').lower() in ('y', 'yes')


def owner_repo(url):
    m = re.search(r'github\.com[:/]([^/]+)/([^/]+?)(?:\.git)?/?$', url)
    return (m.group(1), m.group(2)) if m else (None, None)


def remote_tags():
    out = run(['git', 'ls-remote', '--tags', 'origin', TAG_PREFIX + '*'], check=False)
    return {line.split('refs/tags/')[1].replace('^{}', '') for line in out.splitlines() if 'refs/tags/' in line}


def tag_number(tag):
    m = re.match(r'^ama-field-v(\d+)', tag)
    return int(m.group(1)) if m else -1


def suggest_tag(tags):
    if tags:
        return TAG_PREFIX + str(max(tag_number(t) for t in tags) + 1)
    # no release yet: the newest version in DEVELOPMENT.md's version history
    try:
        versions = re.findall(r'^\| v(\d+)\b', HISTORY.read_text(encoding='utf-8'), re.M)
        if versions:
            return TAG_PREFIX + versions[-1]
    except OSError:
        pass
    return TAG_PREFIX + '1'


def normalise(answer):
    a = answer.strip()
    if re.match(r'^\d', a):
        return TAG_PREFIX + a
    if a.startswith('v') and re.match(r'^v\d', a):
        return 'ama-field-' + a
    return a


def main():
    dry_run = '--dry-run' in sys.argv
    print('\n=== AMA field mode: publish all builds%s ===\n' % (' (dry run)' if dry_run else ''))

    print('[1/6] Working tree clean?')
    dirty = subprocess.run(['git', 'status', '--porcelain'], cwd=str(REPO_ROOT),
                           capture_output=True, text=True).stdout.rstrip()
    if dirty:
        print(dirty)
        msg = ('Commit or stash these changes first: the release is built from the tagged commit, '
               'so it would not contain them.')
        if not dry_run:
            fail(msg)
        print('[DRY RUN] ' + msg)
    else:
        print('      OK.')

    print('[2/6] GitHub remote?')
    url = run(['git', 'remote', 'get-url', 'origin'], check=False)
    owner, repo = owner_repo(url)
    if not owner:
        fail("origin is '%s', not a GitHub repository." % url)
    print('      %s/%s' % (owner, repo))

    print('[3/6] Branch?')
    branch = run(['git', 'branch', '--show-current'])
    if not branch:
        fail('HEAD is detached. Check out %s first.' % RELEASE_BRANCH)
    print('      %s' % branch)
    if branch != RELEASE_BRANCH:
        print('\n[WARN] Releases are cut from %s, and this is %s.' % (RELEASE_BRANCH, branch))
        if not confirm('Publish from %s anyway?' % branch):
            sys.exit(1)

    print('[4/6] Pushed to GitHub?')
    run(['git', 'fetch', '--quiet', 'origin', branch], check=False)
    head = run(['git', 'rev-parse', 'HEAD'])
    upstream = run(['git', 'rev-parse', '--verify', '--quiet', 'origin/' + branch], check=False)
    if not upstream:
        fail('origin/%s does not exist. Push the branch first: git push -u origin %s' % (branch, branch))
    if head != upstream:
        ahead = run(['git', 'rev-list', '--count', 'origin/%s..HEAD' % branch])
        behind = run(['git', 'rev-list', '--count', 'HEAD..origin/%s' % branch])
        fail('%s is %s commit(s) ahead of and %s behind origin/%s.\n'
             'Push (or pull) first, so the release is built from what is on GitHub:\n'
             '  git pull --ff-only && git push origin %s' % (branch, ahead, behind, branch, branch))
    print('      OK, %s = origin/%s.' % (head[:10], branch))

    local = set(run(['git', 'tag', '--list', TAG_PREFIX + '*']).split())
    remote = remote_tags()
    existing = local | remote
    latest = max(existing, key=tag_number) if existing else None
    suggested = suggest_tag(existing)
    print('[5/6] Latest release tag: %s' % (latest or '(none)'))

    answer = os.environ.get('RELEASE_TAG', '').strip()
    if answer:
        print('      Using RELEASE_TAG=%s' % answer)
    else:
        print()
        answer = ask('Tag to publish (17, v17 or ama-field-v17):', suggested)
    tag = normalise(answer)
    if not TAG_RE.match(tag):
        fail("'%s' is not a valid tag. Use ama-field-v<N>, e.g. ama-field-v17 "
             "(the workflow only runs for tags starting with ama-field-v)." % tag)

    redo = tag in existing
    if redo:
        where = run(['git', 'rev-parse', '--verify', '--quiet', tag + '^{commit}'], check=False)
        print('\n[WARN] %s already exists%s.' % (tag, ' (at %s)' % where[:10] if where else ' on GitHub'))
        print('       Moving it to %s rebuilds every board and REPLACES that release\'s notes and files.'
              % head[:10])
        if not confirm('Move %s to this commit and republish?' % tag):
            sys.exit(1)

    notes = os.environ.get('RELEASE_NOTES', '').strip()
    default_notes = 'AMA field mode %s' % tag[len('ama-field-'):]
    if not notes:
        print()
        notes = ask('One-line tag message:', default_notes)

    title_version = tag[len('ama-field-'):]
    print()
    print('=' * 64)
    print('  Tag:      %s%s' % (tag, '  (moved: replaces the existing release)' if redo else ''))
    print('  Commit:   %s  %s' % (head[:10], run(['git', 'log', '-1', '--format=%s'])))
    print('  Branch:   %s' % branch)
    print('  Message:  %s' % notes)
    print('  Builds:   every board in ama-field/ci/boards.txt, files named ..._AMAFieldMode_%s_<board>.apj'
          % title_version)
    print('  Release:  https://github.com/%s/%s/releases/tag/%s' % (owner, repo, tag))
    print('=' * 64)
    print()
    if dry_run:
        print('[DRY RUN] Would now run: git tag -a%s %s -m "%s" and git push%s origin %s'
              % (' -f' if redo else '', tag, notes, ' -f' if redo else '', tag))
        sys.exit(0)
    if not confirm('Create and push the tag now?'):
        print('Aborted, nothing changed.')
        sys.exit(0)

    print('\n[6/6] Tagging and pushing %s...' % tag)
    run(['git', 'tag', '-a'] + (['-f'] if redo else []) + [tag, '-m', notes])
    push = subprocess.run(['git', 'push'] + (['-f'] if redo else []) + ['origin', 'refs/tags/' + tag],
                          cwd=str(REPO_ROOT))
    if push.returncode != 0:
        if not redo:
            run(['git', 'tag', '-d', tag], check=False)
            print('\n[ERROR] Push failed. The local tag was removed, so you can simply run this again.')
        else:
            print('\n[ERROR] Push failed. The local tag was moved but not pushed; run this again to retry.')
        sys.exit(1)

    print()
    print('=' * 64)
    print('  %s pushed. GitHub Actions is building every board (roughly 30-60 minutes).' % tag)
    print()
    print('  Progress:  https://github.com/%s/%s/actions/workflows/ama_field_release.yml' % (owner, repo))
    print('  Release:   https://github.com/%s/%s/releases/tag/%s' % (owner, repo, tag))
    print('=' * 64)


if __name__ == '__main__':
    main()
