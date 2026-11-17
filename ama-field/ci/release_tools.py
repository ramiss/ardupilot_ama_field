#!/usr/bin/env python3
"""
Helpers for the AMA field mode release workflow
(.github/workflows/ama_field_release.yml).

  matrix  read the board list, check every board exists, split it into
          build batches and write the job matrix to GITHUB_OUTPUT
  report  turn the per-board results into the job summary, the release
          notes and SHA256SUMS.txt

AP_FLAKE8_CLEAN
"""

import argparse
import hashlib
import json
import math
import os
import sys

MIN_BATCH = 5
MAX_JOBS = 20   # free-plan concurrent job limit

WARNING = ("Flight-tested only on the FlyingRC F4Wing Mini. Builds for every other board "
           "compile but are untested in flight. Ground-test everything, and fly with a "
           "MANUAL/FBWA override switch.")


def read_list(path):
    """Board names from a file: one per line, '#' starts a comment."""
    boards = []
    with open(path) as f:
        for line in f:
            name = line.split('#', 1)[0].strip()
            if name:
                boards.append(name)
    return boards


def read_notes(path):
    notes = {}
    if path and os.path.exists(path):
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                name, _, text = line.partition(' ')
                notes[name] = text.strip()
    return notes


def set_output(name, value):
    out = os.environ.get('GITHUB_OUTPUT')
    if out:
        with open(out, 'a') as f:
            f.write('%s=%s\n' % (name, value))
    print('%s=%s' % (name, value))


def cmd_matrix(args):
    if args.boards and args.boards.strip():
        boards = args.boards.replace(',', ' ').split()
        source = 'workflow input'
    else:
        boards = read_list(args.boards_file)
        source = args.boards_file

    # de-duplicate, keep order
    seen = set()
    boards = [b for b in boards if not (b in seen or seen.add(b))]

    missing = [b for b in boards if not os.path.isfile(os.path.join(args.hwdef_dir, b, 'hwdef.dat'))]
    if missing:
        for b in missing:
            print('::error::%s (from %s) is not a board under %s' % (b, source, args.hwdef_dir))
        return 1
    if not boards:
        print('::error::no boards to build (from %s)' % source)
        return 1

    # 5 boards per job, more only once the list would need over 20 jobs
    size = max(MIN_BATCH, math.ceil(len(boards) / MAX_JOBS))
    batches = []
    for i in range(0, len(boards), size):
        n = len(batches) + 1
        batches.append({'id': 'b%02d' % n, 'boards': ' '.join(boards[i:i + size])})

    print('%u boards from %s in %u batches of up to %u' % (len(boards), source, len(batches), size))
    set_output('matrix', json.dumps({'batch': batches}, separators=(',', ':')))
    set_output('boards', ' '.join(boards))
    return 0


def kb(value):
    if not value:
        return '?'
    return '%u KB' % round(int(value) / 1024)


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def cmd_report(args):
    boards = args.boards.split()
    notes = read_notes(args.notes_file)

    results = {}
    for b in boards:
        path = os.path.join(args.results, b + '.tsv')
        if not os.path.exists(path):
            results[b] = {'ok': False, 'used': '', 'free': '', 'files': [],
                          'reason': 'build job did not finish'}
            continue
        with open(path) as f:
            fields = (f.read().rstrip('\n').split('\t') + [''] * 6)[:6]
        _, status, used, free, files, reason = fields
        results[b] = {'ok': status == 'ok', 'used': used, 'free': free,
                      'files': files.split(), 'reason': reason.replace('`', "'")}

    ok = [b for b in boards if results[b]['ok']]
    failed = [b for b in boards if not results[b]['ok']]

    # checksums for every published file
    files = sorted(f for f in os.listdir(args.files) if f != 'SHA256SUMS.txt') if os.path.isdir(args.files) else []
    if files:
        with open(os.path.join(args.files, 'SHA256SUMS.txt'), 'w', newline='\n') as f:
            for name in files:
                f.write('%s  %s\n' % (sha256(os.path.join(args.files, name)), name))

    def board_table(link_files):
        rows = ['| Board | Status | Flash used | Flash free | Files |',
                '|---|---|---|---|---|']
        for b in boards:
            r = results[b]
            if r['ok']:
                names = []
                for name in r['files']:
                    label = 'with_bl.hex' if name.endswith('_with_bl.hex') else 'apj'
                    if link_files:
                        names.append('[%s](%s/%s)' % (label, link_files, name))
                    else:
                        names.append(label)
                rows.append('| %s | OK | %s | %s | %s |' % (b, kb(r['used']), kb(r['free']), ', '.join(names)))
            else:
                rows.append('| %s | **failed**: %s | – | – | not available |' % (b, r['reason']))
        return rows

    # job summary
    s = ['## AMA field mode firmware %s' % args.version, '',
         '%u of %u boards built. Commit `%s`.' % (len(ok), len(boards), args.sha[:10]), '']
    s += board_table(None)
    if failed:
        s += ['', 'Failed boards: %s. Each build log is in the workflow artifacts.' % ', '.join(failed)]
    with open(args.summary, 'w', newline='\n', encoding='utf-8') as f:
        f.write('\n'.join(s) + '\n')

    # release notes
    dl = 'https://github.com/%s/releases/download/%s' % (args.repo, args.tag) if args.tag else None
    readme = 'https://github.com/%s/blob/%s/README.md' % (args.repo, args.sha)
    n = []
    n += ['ArduPlane %s with AMA field mode, release %s, built by CI from commit `%s`.'
          % (args.fw_version, args.version, args.sha[:10]), '']
    n += ['> [!WARNING]', '> **%s**' % WARNING, '']
    n += ['What the firmware does, and how to set it up: [README](%s).' % readme, '']
    n += ['## Find your board', '',
          'Each file is named `ArduPlane-%s_AMAFieldMode_%s_<Board>`. The board name is '
          'ArduPilot\'s name for the flight controller, the same one shown on its '
          '[ArduPilot hardware page](https://ardupilot.org/plane/docs/common-autopilots.html). '
          'Pick the file whose name matches your board exactly: a firmware for a similar '
          'board will not work and may not boot.' % (args.fw_version, args.version), '']
    n += board_table(dl)
    if failed:
        n += ['', 'Boards marked failed are **not available** in this release, usually because '
              'the firmware no longer fits in a 1 MB flash.']
    n += ['', '## Which file do I need?', '',
          '- **Board already runs ArduPilot** (any version): use the **`.apj`**.',
          '- **Board runs INAV, Betaflight or other firmware**: use the **`_with_bl.hex`** once, '
          'over DFU. It adds the ArduPilot bootloader. Use the `.apj` for every update after that.', '']
    n += ['## Flashing the .apj with Mission Planner', '',
          '1. Save your current parameters (*Config → Full Parameter List → Save*).',
          '2. Connect the flight controller by USB. Do not press *Connect* in Mission Planner.',
          '3. Open **Install Firmware**, click **Load custom firmware**, and pick the `.apj` for your board.',
          '4. When it finishes, if Mission Planner will not reconnect, **unplug and replug the USB cable**. '
          'Windows often gives the port a new COM number; pick the new one in the port list.',
          '5. Check the boot messages show `ArduPlane V%s`, then load the AMA field parameters '
          '(see the README).' % args.fw_version, '']
    n += ['## First flash from INAV or Betaflight (DFU, `_with_bl.hex`)', '',
          'This erases the board, including any INAV/Betaflight settings. Note them down first.', '',
          '1. Hold the board\'s **BOOT** (bootloader) button while you plug in USB, then release it. '
          'The board is now in DFU mode and shows as "STM32 BOOTLOADER" or "DFU in FS Mode" in '
          'Windows Device Manager.',
          '2. Flash the `_with_bl.hex` with one of:',
          '   - **STM32CubeProgrammer**: choose *USB*, click *Connect*, open *Erasing & Programming*, '
          'select the file, then *Start Programming*.',
          '   - **INAV or Betaflight Configurator**: *Firmware Flasher* → *Load Firmware [Local]*, '
          'tick *Full chip erase*, then *Flash Firmware*. If the board is not found, run '
          '*ImpulseRC Driver Fixer* to install the DFU driver.',
          '3. Unplug and replug USB. The board now runs ArduPilot; use Mission Planner from here on.', '']
    noted = [b for b in boards if b in notes]
    if noted:
        n += ['Board-specific notes:', '']
        n += ['- **%s**: %s' % (b, notes[b]) for b in noted]
        n += ['']
    n += ['Each board\'s own flashing notes are in its README under '
          '[`libraries/AP_HAL_ChibiOS/hwdef/`](https://github.com/%s/tree/%s/libraries/AP_HAL_ChibiOS/hwdef).'
          % (args.repo, args.sha), '']
    n += ['## Checksums', '',
          '`SHA256SUMS.txt` lists a SHA-256 checksum for every file. On Windows: '
          '`certutil -hashfile <file> SHA256`.', '']
    n += ['## Version', '',
          'The boot banner shows `ArduPlane V%s (%s)`; the hash in brackets is the git commit '
          'this release was built from.' % (args.fw_version, args.sha[:8])]
    with open(args.notes, 'w', newline='\n', encoding='utf-8') as f:
        f.write('\n'.join(n) + '\n')

    print('%u of %u boards built' % (len(ok), len(boards)))
    if not ok:
        print('::error::no board built successfully')
        return 1
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    m = sub.add_parser('matrix')
    m.add_argument('--boards-file', default='ama-field/ci/boards.txt')
    m.add_argument('--hwdef-dir', default='libraries/AP_HAL_ChibiOS/hwdef')
    m.add_argument('--boards', default='', help='space or comma separated; overrides the file')

    r = sub.add_parser('report')
    r.add_argument('--boards', required=True, help='space separated, in table order')
    r.add_argument('--results', required=True)
    r.add_argument('--files', required=True)
    r.add_argument('--version', required=True)
    r.add_argument('--fw-version', required=True)
    r.add_argument('--sha', required=True)
    r.add_argument('--repo', required=True)
    r.add_argument('--tag', default='')
    r.add_argument('--notes-file', default='ama-field/ci/board_notes.txt')
    r.add_argument('--summary', required=True)
    r.add_argument('--notes', required=True)

    args = p.parse_args()
    if args.cmd == 'matrix':
        return cmd_matrix(args)
    return cmd_report(args)


if __name__ == '__main__':
    sys.exit(main())
