#!/usr/bin/env python3
"""
Helpers for the AMA field mode release workflow
(.github/workflows/ama_field_release.yml).

  matrix    read the board list, check every board exists, split it into
            build batches and write the job matrix to GITHUB_OUTPUT
  overlays  write (or with --check, verify) the trim-tier<N>.hwdef overlays
            from TRIM_TIERS below and this tree's Tools/scripts/build_options.py
  probe     report the effective value of every trimmable and protected
            feature for a configured board, using its real compile command
  trimdiff  compare a stock and a trimmed probe: list what the trim removed,
            fail if it removed a protected feature
  report    turn the per-board results into the job summary, the release
            notes and SHA256SUMS.txt

AP_FLAKE8_CLEAN
"""

import argparse
import hashlib
import importlib.util
import json
import math
import os
import re
import subprocess
import sys

MIN_BATCH = 5
MAX_JOBS = 20   # free-plan concurrent job limit

WARNING = ("Flight-tested only on the FlyingRC F4Wing Mini. Builds for every other board "
           "compile but are untested in flight. Ground-test everything, and fly with a "
           "MANUAL/FBWA override switch.")
TRIM_WARNING = "Trimmed builds have some less-used features removed; see the list for your board."

CI_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(CI_DIR, '..', '..'))

# Features turned off, in order, for a 1 MB board that does not fit. Labels are
# build_options.py feature labels; everything that depends on them is turned off
# too. 'extra' is for switches that are not build options.
TRIM_TIERS = [
    {
        'name': 'tier 1',
        'title': 'features AMA field mode cannot use',
        'labels': ['QUADPLANE', 'QUICKTUNE', 'DEEPSTALL', 'SOARING', 'ICE Engine', 'EFI', 'PARACHUTE'],
        'extra': {},
    },
    {
        'name': 'tier 2',
        'title': 'peripherals a novice field aircraft rarely has',
        'labels': ['ADSB', 'Camera', 'MOUNT', 'GRIPPER', 'GENERATOR', 'PROXIMITY', 'AC_AVOID',
                   'BEACON', 'VISUALODOM'],
        'extra': {'AP_TERRAIN_AVAILABLE': 'Terrain database and terrain following'},
    },
]

# Never removed. An overlay may not name a define from these headers or with
# these prefixes, and a trimmed build fails if one of them was on in the stock
# build and is off in the trimmed one.
PROTECTED_HEADERS = [
    'libraries/AC_Fence/AC_Fence_config.h',
    'libraries/AP_Airspeed/AP_Airspeed_config.h',
    'libraries/AP_Baro/AP_Baro_config.h',
    'libraries/AP_BattMonitor/AP_BattMonitor_config.h',
    'libraries/AP_Compass/AP_Compass_config.h',
    'libraries/AP_GPS/AP_GPS_config.h',
    'libraries/AP_Logger/AP_Logger_config.h',
    'libraries/AP_OSD/AP_OSD_config.h',
    'libraries/AP_RangeFinder/AP_RangeFinder_config.h',
    'libraries/AP_RCProtocol/AP_RCProtocol_config.h',
]
PROTECTED_EXTRA = ['MODE_AUTOLAND_ENABLED', 'HAL_GCS_ENABLED', 'HAL_LOGGING_ENABLED',
                   'AP_BOOTLOADER_FLASHING_ENABLED']
PROTECTED_PREFIXES = ['AP_RCPROTOCOL_', 'HAL_LOGGING_', 'AP_AIRSPEED_', 'AP_RANGEFINDER_', 'AP_BATTERY_',
                      'AP_GPS_', 'AP_COMPASS_', 'AP_BARO_', 'OSD_', 'AP_OSD_', 'AP_FENCE_', 'HAL_GCS_',
                      'AP_BOOTLOADER_', 'MODE_AUTOLAND_']
# Battery backends that only read from a device a tier removes; listed as removed,
# not treated as a protected feature.
PROTECTED_CONSEQUENCES = {
    'AP_BATTERY_EFI_ENABLED': 'Battery monitor fed by an EFI unit',
    'AP_BATTERY_GENERATOR_ENABLED': 'Battery monitor fed by a generator',
}

# Turned on for every board by field-mode.hwdef. Only the last fallback build
# (trim tier 2 without field-mode.hwdef) may lose them; the release notes show
# them as "baro comp", not in the removed lists.
FIELD_MODE_DEFINES = {
    'AP_BARO_THST_COMP_ENABLED': 'Baro thrust compensation (BARO1_THST_SCALE)',
    'HAL_BARO_WIND_COMP_ENABLED': 'Baro wind (airspeed) compensation (BARO1_WCF_*)',
}
FIELD_MODE_OVERLAY = 'field-mode.hwdef'
TUNING_NO_BARO = 'ama-field/docs/TUNING.md#44-boards-without-baro-compensation'
BARO_PRIMARY_NOTE = ('Baro thrust compensation applies to the first barometer only: on a board with more '
                     'than one, leave `BARO_PRIMARY` at 0.')


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


# ---------------------------------------------------------------- trim tiers

def build_options():
    path = os.path.join(ROOT, 'Tools', 'scripts', 'build_options.py')
    spec = importlib.util.spec_from_file_location('build_options', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.BUILD_OPTIONS


def closure(labels, options):
    """labels plus every build option that depends on one of them"""
    by_label = {o.label: o for o in options}
    for label in labels:
        if label not in by_label:
            raise ValueError('%s is not a feature label in build_options.py' % label)
    ret = set(labels)
    added = True
    while added:
        added = False
        for o in options:
            if o.label in ret or not o.dependency:
                continue
            if any(d in ret for d in o.dependency.split(',')):
                ret.add(o.label)
                added = True
    return ret


def tier_defines(options):
    """[(tier, {define: description})] with each tier holding only its own additions"""
    by_label = {o.label: o for o in options}
    done = set()
    seen_labels = []
    ret = []
    for tier in TRIM_TIERS:
        seen_labels += tier['labels']
        labels = closure(seen_labels, options)
        defines = {}
        for label in sorted(labels, key=lambda la: (by_label[la].category, la)):
            o = by_label[label]
            if o.define not in done:
                defines[o.define] = re.sub(r'^Enable\s+', '', o.description).strip()
                done.add(o.define)
        for name, desc in tier['extra'].items():
            if name not in done:
                defines[name] = desc
                done.add(name)
        ret.append((tier, defines))
    return ret


def all_defines_in_tree():
    """every NAME seen in '#define NAME' or 'define NAME' in the source tree"""
    names = set()
    pat = re.compile(r'^\s*#?\s*define\s+([A-Z0-9_]+)', re.M)
    for top in ('libraries', 'ArduPlane'):
        for dirpath, _, files in os.walk(os.path.join(ROOT, top)):
            for f in files:
                if f.endswith(('.h', '.inc', '.dat')):
                    try:
                        with open(os.path.join(dirpath, f), errors='replace') as fh:
                            names.update(pat.findall(fh.read()))
                    except OSError:
                        pass
    return names


def protected_names():
    names = set(PROTECTED_EXTRA)
    pat = re.compile(r'^\s*#\s*define\s+([A-Z0-9_]+_ENABLED)\b', re.M)
    for h in PROTECTED_HEADERS:
        with open(os.path.join(ROOT, h)) as f:
            names.update(pat.findall(f.read()))
    return names


def is_protected(name):
    return name in PROTECTED_EXTRA or any(name.startswith(p) for p in PROTECTED_PREFIXES)


def overlay_text(index, tier, defines):
    lines = ['# AMA field mode release CI: trim %s, %s.' % (tier['name'], tier['title']),
             '# Used only for a 1 MB board whose stock build does not fit; see',
             '# ama-field/docs/DEVELOPMENT.md, "Release CI".',
             '# Generated by "ama-field/ci/release_tools.py overlays"; edit TRIM_TIERS there.',
             '']
    if index > 0:
        lines += ['include trim-tier%u.hwdef' % index, '']
    for name, desc in defines.items():
        lines += ['# %s' % desc, 'undef %s' % name, 'define %s 0' % name]
    return '\n'.join(lines) + '\n'


def cmd_overlays(args):
    options = build_options()
    tiers = tier_defines(options)
    known = all_defines_in_tree()
    rc = 0
    for i, (tier, defines) in enumerate(tiers):
        for name in defines:
            if name not in known:
                print('::error::%s (%s) is not defined anywhere in this tree' % (name, tier['name']))
                rc = 1
            if is_protected(name) and name not in PROTECTED_CONSEQUENCES:
                print('::error::%s (%s) is a protected feature' % (name, tier['name']))
                rc = 1
        for name in defines:
            if name in FIELD_MODE_DEFINES:
                print('::error::%s (%s) is turned on by %s' % (name, tier['name'], FIELD_MODE_OVERLAY))
                rc = 1
        path = os.path.join(CI_DIR, 'trim-tier%u.hwdef' % (i + 1))
        text = overlay_text(i, tier, defines)
        if args.check:
            old = open(path).read() if os.path.exists(path) else ''
            if old.replace('\r\n', '\n') != text:
                print('::error::%s is out of date; run "python3 ama-field/ci/release_tools.py overlays"'
                      % os.path.relpath(path, ROOT))
                rc = 1
        else:
            with open(path, 'w', newline='\n') as f:
                f.write(text)
            print('wrote %s (%u defines)' % (os.path.relpath(path, ROOT), len(defines)))
    # the hand-written field-mode overlay must turn on exactly FIELD_MODE_DEFINES
    with open(os.path.join(CI_DIR, FIELD_MODE_OVERLAY)) as f:
        on = set(re.findall(r'^\s*define\s+([A-Z0-9_]+)\s+1\b', f.read(), re.M))
    if on != set(FIELD_MODE_DEFINES):
        print('::error::%s turns on %s, expected %s'
              % (FIELD_MODE_OVERLAY, ' '.join(sorted(on)), ' '.join(sorted(FIELD_MODE_DEFINES))))
        rc = 1
    for name in FIELD_MODE_DEFINES:
        if name not in known:
            print('::error::%s (%s) is not defined anywhere in this tree' % (name, FIELD_MODE_OVERLAY))
            rc = 1
    if args.check and rc == 0:
        print('trim overlays are up to date')
    return rc


def probe_names():
    names = {}
    for _, defines in tier_defines(build_options()):
        names.update(defines)
    for name in protected_names():
        names.setdefault(name, name)
    return names


def cmd_probe(args):
    """effective 0/1 value of each probed define, as the ArduPlane sources see it"""
    with open(os.path.join(args.build_dir, 'compile_commands.json')) as f:
        db = json.load(f)
    entry = None
    for e in db:
        if e['file'].replace('\\', '/').endswith('ArduPlane/Plane.cpp'):
            entry = e
            break
    if entry is None:
        print('::error::no ArduPlane/Plane.cpp entry in compile_commands.json')
        return 1
    cwd = entry['directory']
    src = os.path.normpath(os.path.join(cwd, entry['file']))

    names = sorted(probe_names())
    probe = os.path.join(os.path.abspath(args.build_dir), 'ama_feature_probe.cpp')
    lines = ['#include "Plane.h"', '#include <AP_Terrain/AP_Terrain_config.h>']
    for h in PROTECTED_HEADERS:
        lines.append('#include <%s>' % h.split('/', 1)[1])
    # the name goes in a string literal so the preprocessor doesn't expand it
    for n in names:
        lines += ['#if !defined(%s)' % n, 'ama_probe "%s" undefined' % n,
                  '#elif %s' % n, 'ama_probe "%s" 1' % n, '#else', 'ama_probe "%s" 0' % n, '#endif']
    with open(probe, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    cmd = []
    it = iter(entry['arguments'])
    for a in it:
        if a in ('-o', '-MF', '-MT', '-MQ'):
            next(it, None)
            continue
        if a in ('-c', '-MMD', '-MD', '-MP') or a.startswith('-o'):
            # waf joins the output flag and path: -oArduPlane/Plane.cpp.0.o
            continue
        if not a.startswith('-') and os.path.normpath(os.path.join(cwd, a)) == src:
            continue
        cmd.append(a)
    cmd += ['-E', '-P', '-I' + os.path.dirname(src), probe]
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if p.returncode != 0:
        print(p.stderr[-4000:])
        print('::error::feature probe failed to preprocess')
        return 1
    result = {}
    for line in p.stdout.splitlines():
        a = line.split()
        if len(a) == 3 and a[0] == 'ama_probe':
            result[a[1].strip('"')] = None if a[2] == 'undefined' else int(a[2])
    if len(result) != len(names):
        print('::error::feature probe read %u of %u defines' % (len(result), len(names)))
        return 1
    with open(args.out, 'w') as f:
        json.dump(result, f, indent=1, sort_keys=True)
    print('probed %u defines -> %s' % (len(result), args.out))
    return 0


def cmd_trimdiff(args):
    with open(args.stock) as f:
        stock = json.load(f)
    with open(args.trimmed) as f:
        trimmed = json.load(f)
    labels = probe_names()
    labels.update(PROTECTED_CONSEQUENCES)
    removed = [n for n in sorted(stock) if stock[n] and not trimmed.get(n)]
    # the baro compensation is reported in its own column, not as a removed feature
    lost_field = [n for n in removed if n in FIELD_MODE_DEFINES]
    removed = [n for n in removed if n not in FIELD_MODE_DEFINES]
    bad = [n for n in removed if is_protected(n) and n not in PROTECTED_CONSEQUENCES]
    if lost_field and not args.no_baro_comp:
        bad += lost_field
    with open(args.out, 'w', newline='\n') as f:
        for n in removed:
            f.write('%s\t%s\n' % (n, labels.get(n, n)))
    if bad:
        print('protected features removed: %s' % ' '.join(bad))
        return 1
    print('%u features removed%s' % (len(removed), ', no baro compensation' if lost_field else ''))
    return 0


# ---------------------------------------------------------------- report

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


def read_results(boards, results_dir):
    results = {}
    for b in boards:
        path = os.path.join(results_dir, b + '.tsv')
        r = {'ok': False, 'tier': '', 'used': '', 'free': '', 'files': [], 'reason': 'build job did not finish',
             'removed': [], 'baro': ''}
        if os.path.exists(path):
            with open(path) as f:
                fields = (f.read().rstrip('\n').split('\t') + [''] * 8)[:8]
            _, status, tier, used, free, files, reason, baro = fields
            r.update({'ok': status == 'ok', 'tier': tier, 'used': used, 'free': free,
                      'files': files.split(), 'reason': reason.replace('`', "'"), 'baro': baro})
        rpath = os.path.join(results_dir, b + '.removed.tsv')
        if os.path.exists(rpath):
            with open(rpath) as f:
                r['removed'] = [line.rstrip('\n').split('\t', 1) for line in f if line.strip()]
        results[b] = r
    return results


def cmd_report(args):
    boards = args.boards.split()
    notes = read_notes(args.notes_file)
    results = read_results(boards, args.results)

    ok = [b for b in boards if results[b]['ok']]
    failed = [b for b in boards if not results[b]['ok']]
    trimmed = [b for b in ok if results[b]['tier'] not in ('', 'stock')]
    no_baro = [b for b in ok if results[b]['baro'] != 'yes']
    tuning = 'https://github.com/%s/blob/%s/%s' % (args.repo, args.sha, TUNING_NO_BARO)

    def baro_cell(r):
        if r['baro'] == 'yes':
            return 'yes'
        return '**%s**: relies on AUTOLAND_TD_OFS, see [TUNING.md](%s)' % (r['baro'] or '?', tuning)

    # checksums for every published file
    files = sorted(f for f in os.listdir(args.files) if f != 'SHA256SUMS.txt') if os.path.isdir(args.files) else []
    if files:
        with open(os.path.join(args.files, 'SHA256SUMS.txt'), 'w', newline='\n') as f:
            for name in files:
                f.write('%s  %s\n' % (sha256(os.path.join(args.files, name)), name))

    def board_table(link_files):
        rows = ['| Board | Status | Build | Baro comp | Flash used | Flash free | Files |',
                '|---|---|---|---|---|---|---|']
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
                tier = r['tier'] or 'stock'
                if tier != 'stock':
                    tier = '**%s** ([%u removed](#removed-%s))' % (tier, len(r['removed']), b.lower())
                rows.append('| %s | OK | %s | %s | %s | %s | %s |'
                            % (b, tier, baro_cell(r), kb(r['used']), kb(r['free']), ', '.join(names)))
            else:
                rows.append('| %s | **failed**: %s | – | – | – | – | not available |' % (b, r['reason']))
        return rows

    def removed_lists():
        out = []
        for b in trimmed:
            r = results[b]
            out += ['<a id="removed-%s"></a>' % b.lower(), '<details>',
                    '<summary><b>%s</b>: %s, %u features removed</summary>' % (b, r['tier'], len(r['removed'])), '']
            out += ['- %s (`%s`)' % (desc, name) for name, desc in r['removed']]
            out += ['', '</details>', '']
        return out

    tier_help = ['Build: **stock** is the board\'s normal ArduPilot configuration. A 1 MB board '
                 'that does not fit is rebuilt with **tier 1** (%s) and, if still too big, '
                 '**tier 2** (%s). The list below the table shows exactly what each trimmed '
                 'board is missing.' % (TRIM_TIERS[0]['title'], TRIM_TIERS[1]['title']), '',
                 'Baro comp: every build has baro thrust and airspeed compensation (`BARO1_THST_SCALE`, '
                 'learned on each auto takeoff, and `BARO1_WCF_*`), unless the board only fits without it. '
                 'A board marked **no** relies on `AUTOLAND_TD_OFS` (learned on each landing, like every '
                 'board) to take up the larger float; see [TUNING.md](%s). %s' % (tuning, BARO_PRIMARY_NOTE)]

    # job summary
    s = ['## AMA field mode firmware %s' % args.version, '',
         '%u of %u boards built, %u of them trimmed, %u without baro compensation. Commit `%s`.'
         % (len(ok), len(boards), len(trimmed), len(no_baro), args.sha[:10]), '']
    s += board_table(None)
    if failed:
        s += ['', 'Failed boards: %s. Each build log is in the workflow artifacts.' % ', '.join(failed)]
    if trimmed:
        s += ['', '### Features removed from trimmed boards', '']
        s += removed_lists()
    with open(args.summary, 'w', newline='\n', encoding='utf-8') as f:
        f.write('\n'.join(s) + '\n')

    # release notes
    dl = 'https://github.com/%s/releases/download/%s' % (args.repo, args.tag) if args.tag else None
    readme = 'https://github.com/%s/blob/%s/README.md' % (args.repo, args.sha)
    n = []
    n += ['ArduPlane %s with AMA field mode, release %s, built by CI from commit `%s`.'
          % (args.fw_version, args.version, args.sha[:10]), '']
    n += ['> [!WARNING]', '> **%s**' % WARNING, '>', '> **%s**' % TRIM_WARNING, '']
    n += ['What the firmware does, and how to set it up: [README](%s).' % readme, '']
    n += ['## Find your board', '',
          'Each file is named `ArduPlane-%s_AMAFieldMode_%s_<Board>`. The board name is '
          'ArduPilot\'s name for the flight controller, the same one shown on its '
          '[ArduPilot hardware page](https://ardupilot.org/plane/docs/common-autopilots.html). '
          'Pick the file whose name matches your board exactly: a firmware for a similar '
          'board will not work and may not boot.' % (args.fw_version, args.version), '']
    n += board_table(dl)
    n += [''] + tier_help
    if failed:
        n += ['', 'Boards marked failed are **not available** in this release, usually because '
              'the firmware does not fit in a 1 MB flash even with tier 2 trimming and no baro '
              'compensation.']
    if trimmed:
        n += ['', '## Features removed from trimmed boards', '']
        n += removed_lists()
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

    print('%u of %u boards built, %u trimmed, %u without baro compensation'
          % (len(ok), len(boards), len(trimmed), len(no_baro)))
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

    o = sub.add_parser('overlays')
    o.add_argument('--check', action='store_true', help='verify instead of writing')

    pr = sub.add_parser('probe')
    pr.add_argument('--build-dir', required=True, help='build/<Board> after configure and a build attempt')
    pr.add_argument('--out', required=True)

    t = sub.add_parser('trimdiff')
    t.add_argument('--stock', required=True)
    t.add_argument('--trimmed', required=True)
    t.add_argument('--out', required=True)
    t.add_argument('--no-baro-comp', action='store_true',
                   help='build without field-mode.hwdef: the baro compensation may be off')

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
    return {'matrix': cmd_matrix, 'overlays': cmd_overlays, 'probe': cmd_probe,
            'trimdiff': cmd_trimdiff, 'report': cmd_report}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
