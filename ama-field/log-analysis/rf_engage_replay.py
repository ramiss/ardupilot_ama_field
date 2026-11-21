#!/usr/bin/env python3
"""
Rangefinder engagement on the landing: replays ArduPlane's in-range rules on the
raw rangefinder samples (RFSS) of each landing in a log.

  stock: 10 counted samples (a new distance, more than 5% of RNGFNDx_MAX from the
         first reading), reset by a rejected sample or a jump of 20% of max range
  v17:   stock, plus 3 new samples in a row with quality >= 50% (superseded)
  new:   the AUTOLAND-final rule: a new sample counts if it agrees with the
         previous one plus the aircraft's own vertical motion (vertical speed x
         time between samples, heights tilt-corrected) within 0.3 m + 0.5 m/s x dt
         at quality >= 50%, else 0.1 m + 0.3 m/s x dt; quality below 20% restarts
         the run; in range once the run spans >= 0.18 s with >= 3 samples if all
         have quality >= 50%, else >= 6

All three also apply this firmware's plausibility gate: a sample with quality
below 50% that disagrees with the baro by more than 2.5 m + 20% is rejected.

Sensor agnostic: uses RFSS.Q (0-100%) when the log has it. Older logs (TOFSense-F
only) have no Q, so it is derived from the MT-06 strength the way the driver
does (strength / 255). Invalid samples: a TOFSense-F sample with St != 1 never
reaches ArduPlane (skipped); for other drivers an invalid sample is out of range
(restarts the run).

The vertical speed is the EKF's in the firmware; these logs have no EKF messages,
so GPS.VZ is used (the EKF fuses it). Heights are the distance of the sample that
completed the run; the firmware engages on its next 50 Hz call.

--scan also runs each rule over the whole flight with the baro above 3 m (the
firmware only applies the new rule on an AUTOLAND final, so this is a stress
test) and checks each engagement against the next second of samples: real
ground keeps agreeing with the vertical motion, random readings don't. An
engagement after which fewer than 60% of the next second's samples agree is
counted as false.

Usage: rf_engage_replay.py [--scan] LOG.BIN [LOG2.BIN ...]
Needs pymavlink and numpy.

AP_FLAKE8_CLEAN
"""
import math
import sys

import numpy as np
from pymavlink import mavutil

OLD_COUNT = 10
V17_STRONG = 3
TOFSENSE_TYPE = 40


def load(path):
    m = mavutil.mavlink_connection(path)
    S, P, msgs, parm, att, gps = [], [], [], {}, [], []
    while True:
        x = m.recv_match(type=['RFSS', 'POS', 'MSG', 'PARM', 'ATT', 'GPS'])
        if x is None:
            break
        t = x.get_type()
        if t == 'RFSS':
            q = getattr(x, 'Q', None)
            ty = getattr(x, 'Ty', TOFSENSE_TYPE)
            if q is None:
                q = min(100, x.Str * 100 // 255) if x.St == 1 else 0
            S.append((x.TimeUS / 1e6, x.D, q, x.St == 1, ty))
        elif t == 'POS':
            P.append((x.TimeUS / 1e6, x.RelHomeAlt))
        elif t == 'MSG':
            msgs.append((x.TimeUS / 1e6, x.Message))
        elif t == 'ATT':
            att.append((x.TimeUS / 1e6, math.radians(x.Roll), math.radians(x.Pitch)))
        elif t == 'GPS' and getattr(x, 'I', 0) == 0:
            gps.append((x.TimeUS / 1e6, x.VZ))
        else:
            parm.setdefault(x.Name, x.Value)
    return S, P, msgs, parm, att, gps


class Ctx:
    def __init__(self, P, att, gps, parm):
        self.pt = np.array([p[0] for p in P])
        self.pa = np.array([p[1] for p in P])
        self.at = np.array([a[0] for a in att]) if att else np.array([0.0])
        self.ar = np.array([a[1] for a in att]) if att else np.array([0.0])
        self.ap = np.array([a[2] for a in att]) if att else np.array([0.0])
        self.gt = np.array([g[0] for g in gps]) if gps else np.array([0.0])
        self.gv = np.array([g[1] for g in gps]) if gps else np.array([0.0])
        self.rmin = parm.get('RNGFND1_MIN', 0.05) or 0.05
        self.rmax = parm.get('RNGFND1_MAX', 6.0) or 6.0

    def baro(self, t):
        return float(np.interp(t, self.pt, self.pa))

    def tilt(self, t):
        return math.cos(float(np.interp(t, self.at, self.ar))) * math.cos(float(np.interp(t, self.at, self.ap)))

    def vz(self, t):
        return float(np.interp(t, self.gt, self.gv))


def samples_for_rules(win, ctx):
    """(t, d, h, q, ok, in_range) for each sample ArduPlane would see; ok includes the baro gate"""
    out = []
    for t, d, q, valid, ty in win:
        if not valid and ty == TOFSENSE_TYPE:
            continue
        in_range = valid and ctx.rmin <= d <= ctx.rmax
        h = d * ctx.tilt(t)
        b = ctx.baro(t)
        ok = in_range and not (q < 50 and abs(h - b) > 2.5 + 0.2 * abs(b))
        out.append((t, d, h, q, ok, in_range))
    return out


def rule_stock(samples, ctx, v17=False):
    """engagements [(t, d)] of the stock count rule (with the v17 strong path)"""
    out = []
    count = strong = 0
    last = -1.0
    init = None
    for t, d, h, q, ok, _ in samples:
        if not ok:
            count = strong = 0
            continue
        if init is None:
            init = d
        new = d != last
        if new and abs(init - d) > 0.05 * ctx.rmax:
            count += 1
        if v17 and new:
            strong = strong + 1 if q >= 50 else 0
        if abs(last - d) > 0.2 * ctx.rmax:
            count = strong = 0
        if v17 and strong >= V17_STRONG:
            count = OLD_COUNT
        last = d
        if count >= OLD_COUNT:
            out.append((t + 0.02, d))
            count = strong = 0
            init = None
    return out


def rule_new(samples, ctx):
    """engagements [(t, d)] of the AUTOLAND-final consistency rule"""
    out = []
    count = 0
    start = last_t = 0.0
    last_h = 0.0
    last_d = -1.0
    weak = False
    for t, d, h, q, ok, _ in samples:
        if not ok:
            count = 0
            continue
        dt = t - last_t if last_t else 1.0
        last_t = t
        if d == last_d:
            continue
        last_d = d
        if 0 <= q < 20:
            count = 0
            continue
        strong = q >= 50
        tol = (0.3 + 0.5 * dt) if strong else (0.1 + 0.3 * dt)
        pred = last_h - ctx.vz(t) * dt
        if count == 0 or dt > 0.5 or abs(h - pred) > tol:
            count = 1
            start = t
            weak = False
        else:
            count += 1
        weak = weak or not strong
        last_h = h
        if count >= (6 if weak else 3) and t - start >= 0.18:
            out.append((t + 0.02, d))
            count = 0
    return out


def sustained(samples, ctx, t_eng):
    """after an engagement at t_eng: do the next second's valid samples keep agreeing
    with the vertical motion? (the baro gate is left out: near the edge of a weak
    sensor's range it can reject real readings when the baro reads low)"""
    nxt = [s for s in samples if t_eng - 0.02 <= s[0] <= t_eng + 1.0 and s[5]]
    if len(nxt) < 3:
        return None
    agree = 0
    for a, b in zip(nxt, nxt[1:]):
        dt = b[0] - a[0]
        if abs(b[2] - (a[2] - ctx.vz(b[0]) * dt)) <= 0.3 + 0.5 * dt:
            agree += 1
    return agree >= 0.6 * (len(nxt) - 1)


def gate_rejects(samples, t_eng):
    """valid samples the baro gate rejected in the second after an engagement"""
    return sum(1 for s in samples if t_eng - 0.02 <= s[0] <= t_eng + 1.0 and s[5] and not s[4])


def landings(S, P):
    """touchdown times: a strong reading under 0.3 m within 15 s of being above 5 m (baro)"""
    if not P:
        return []
    pt = np.array([p[0] for p in P])
    pa = np.array([p[1] for p in P])
    out = []
    for t, d, q, valid, ty in S:
        if valid and q >= 50 and d < 0.3 and (not out or t - out[-1] > 60):
            k = (pt > t - 15) & (pt < t)
            if k.any() and pa[k].max() > 5:
                out.append(t)
    return out


def fmt(e, td):
    return '%.2f m (%.1f s)' % (e[1], td - e[0]) if e else 'not engaged'


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    scan = '--scan' in sys.argv
    rows = []
    scans = []
    for path in args:
        S, P, msgs, parm, att, gps = load(path)
        name = path.replace('\\', '/').split('/')[-1]
        if not S or not P:
            print('%s: no RFSS samples' % name)
            continue
        ctx = Ctx(P, att, gps, parm)
        for td in landings(S, P):
            win = samples_for_rules([s for s in S if td - 25 <= s[0] <= td + 0.5], ctx)
            first = next(((t, d) for t, d, h, q, ok, _ in win if ok and q >= 50 and d > 0.3), None)
            res = []
            for rule in ('stock', 'v17', 'new'):
                e = rule_new(win, ctx) if rule == 'new' else rule_stock(win, ctx, v17=(rule == 'v17'))
                res.append(e[0] if e else None)
            logged = next((m for t, m in msgs if td - 25 <= t <= td + 2 and 'Rangefinder engaged' in m), '-')
            rows.append((name, td, first) + tuple(res) + (logged,))
            sus = sustained(win, ctx, res[2][0] - 0.02) if res[2] else None
            gr = gate_rejects(win, res[2][0] - 0.02) if res[2] else 0
            print('%s landing at %.0f s: first strong %s | stock %s | v17 %s | new %s%s | logged: %s'
                  % (name, td, '%.2f m' % first[1] if first else '-', fmt(res[0], td), fmt(res[1], td),
                     fmt(res[2], td), {True: ' (sustained', False: ' (NOT sustained', None: ' ('}[sus]
                     + ', %d gate rejects next 1 s)' % gr, logged))
        if scan:
            win = samples_for_rules([s for s in S if ctx.baro(s[0]) > 3.0], ctx)
            secs = sum(1 for s in win if s[4]) * 0.1
            res = []
            for e in (rule_stock(win, ctx), rule_stock(win, ctx, v17=True), rule_new(win, ctx)):
                bad = sum(1 for t, d in e if sustained(win, ctx, t - 0.02) is False)
                res.append('%d/%d' % (bad, len(e)))
            scans.append((name, secs) + tuple(res))
    if rows:
        print('\nheights in m (s before touchdown); an earlier engagement can show a lower height'
              ' when the aircraft floats or balloons after the flare')
        print('log           landing   first strong   stock             v17               new')
        for name, td, first, a, b, c, logged in rows:
            print('%-13s %6.0f s   %8s       %-16s  %-16s  %-16s' % (
                name, td, '%.2f' % first[1] if first else '-', fmt(a, td), fmt(b, td), fmt(c, td)))
    if scans:
        print('\nwhole flight with the baro above 3 m: false engagements / all engagements')
        print('log           valid samples   stock     v17       new')
        for name, secs, a, b, c in scans:
            print('%-13s %8.0f s       %-8s  %-8s  %-8s' % (name, secs, a, b, c))


if __name__ == '__main__':
    main()
