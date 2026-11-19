#!/usr/bin/env python3
"""
Rangefinder engagement on the landing: replays ArduPlane's in-range rule on the
raw TOFSense-F samples (RFSS) of each landing in a log, old rule vs new rule.

  old: 10 counted samples (a new distance, more than 5% of RNGFNDx_MAX from the
       first reading), reset by any rejected sample or a jump of 20% of max range;
       readings that disagree with the baro by more than 2.5 m + 20% are rejected
       unless the signal is strong (this firmware's plausibility gate)
  new: as old, but 3 new strong samples in a row (strength >= 128, i.e. quality
       >= 50%) are enough on their own (v17)

The logs are at the 10 Hz the driver used to poll. Heights are the distance of
the sample that completed the count (the firmware engages on its next 50 Hz
call). The 50 Hz estimate for the new rule starts from the first strong 10 Hz
sample and applies the sink rate over the sample timing (see est50 below).

Usage: rf_engage_replay.py LOG.BIN [LOG2.BIN ...]
Needs pymavlink and numpy.

AP_FLAKE8_CLEAN
"""
import sys

import numpy as np
from pymavlink import mavutil

STRONG = 128
OLD_COUNT = 10
NEW_STRONG = 3


def load(path):
    m = mavutil.mavlink_connection(path)
    S, P, msgs, parm = [], [], [], {}
    while True:
        x = m.recv_match(type=['RFSS', 'POS', 'MSG', 'PARM'])
        if x is None:
            break
        t = x.get_type()
        if t == 'RFSS':
            S.append((x.TimeUS / 1e6, x.D, x.Str, x.St))
        elif t == 'POS':
            P.append((x.TimeUS / 1e6, x.RelHomeAlt))
        elif t == 'MSG':
            msgs.append((x.TimeUS / 1e6, x.Message))
        else:
            parm.setdefault(x.Name, x.Value)
    return S, P, msgs, parm


def landings(S, P):
    """touchdown times: a strong reading under 0.3 m within 15 s of being above 5 m (baro)"""
    if not P:
        return []
    pt = np.array([p[0] for p in P])
    pa = np.array([p[1] for p in P])
    out = []
    for t, d, s, st in S:
        if st == 1 and s >= STRONG and d < 0.3 and (not out or t - out[-1] > 60):
            k = (pt > t - 15) & (pt < t)
            if k.any() and pa[k].max() > 5:
                out.append(t)
    return out


def replay(samples, baro_at, max_m, strong_rule):
    """first sample at which in_range would be set: (t, distance) or None"""
    count = 0
    strong = 0
    last = -1.0
    init = None
    for t, d, s, st in samples:
        ok = st == 1 and 0.02 < d < max_m
        b = baro_at(t)
        if ok and s < STRONG and abs(d - b) > 2.5 + 0.2 * abs(b):
            ok = False
        if not ok:
            count = 0
            strong = 0
            continue
        if init is None:
            init = d
        new = d != last
        if new and abs(init - d) > 0.05 * max_m:
            count += 1
        if strong_rule and new:
            strong = strong + 1 if s >= STRONG else 0
        if abs(last - d) > 0.2 * max_m:
            count = 0
            strong = 0
        if strong_rule and strong >= NEW_STRONG:
            count = OLD_COUNT
        last = d
        if count >= OLD_COUNT:
            # engaged on the next 50 Hz call, 20 ms later, at about this distance
            return t + 0.02, d
    return None


def fmt(e, td):
    return '%.2f m (%.1f s)' % (e[1], td - e[0]) if e else 'not engaged'


def main():
    rows = []
    for path in sys.argv[1:]:
        S, P, msgs, parm = load(path)
        name = path.replace('\\', '/').split('/')[-1]
        if not S:
            print('%s: no RFSS samples' % name)
            continue
        max_m = parm.get('RNGFND1_MAX', 6.0) or 6.0
        pt = np.array([p[0] for p in P])
        pa = np.array([p[1] for p in P])

        def baro_at(t):
            return float(np.interp(t, pt, pa))

        for td in landings(S, P):
            win = [s for s in S if td - 25 <= s[0] <= td + 0.5]
            old = replay(win, baro_at, max_m, False)
            new = replay(win, baro_at, max_m, True)
            first = next(((t, d) for t, d, s, st in win if st == 1 and s >= STRONG and d > 0.3), None)
            # sink rate from the strong readings below 3 m
            strong = [(t, d) for t, d, s, st in win if st == 1 and s >= STRONG and 0.3 < d < 3.0]
            sink = None
            if len(strong) >= 3:
                a = np.array(strong)
                sink = -np.polyfit(a[:, 0], a[:, 1], 1)[0]
            est50 = None
            if first and sink:
                # the strong range starts on average 50 ms (half a 10 Hz sample) above the
                # first strong 10 Hz sample; at 50 Hz the first strong sample comes 10 ms
                # after that on average, then 2 more samples (40 ms) and the 20 ms to the
                # next call: 0.05 - 0.01 - 0.04 - 0.02 = 20 ms of sink below the first sample
                est50 = first[1] - sink * 0.02
            logged = next((m for t, m in msgs if td - 25 <= t <= td + 2 and 'Rangefinder engaged' in m), '-')
            rows.append((name, td, first, old, new, est50, sink, logged))
            print('%s landing at %.0f s: first strong %s | old rule %s | new rule %s | new at 50 Hz ~%s '
                  '(sink %s) | logged: %s'
                  % (name, td,
                     '%.2f m' % first[1] if first else '-',
                     fmt(old, td), fmt(new, td),
                     '%.2f m' % est50 if est50 else '-',
                     '%.1f m/s' % sink if sink else '-', logged))
    if rows:
        print('\nheights in m (s before touchdown); an earlier engagement can show a lower height'
              ' when the aircraft floats or balloons after the flare')
        print('log           landing   first strong   old rule          new rule (10 Hz)   new rule (50 Hz est.)')
        for name, td, first, old, new, est50, sink, logged in rows:
            print('%-13s %6.0f s   %8s       %-16s  %-16s   %8s' % (
                name, td, '%.2f' % first[1] if first else '-', fmt(old, td), fmt(new, td),
                '%.2f' % est50 if est50 else '-'))


if __name__ == '__main__':
    main()
