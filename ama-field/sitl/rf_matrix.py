#!/usr/bin/env python3
"""
Rangefinder sensor matrix: one row per autoland_circuit_test.py run (rfprof=...),
from the run's dataflash log, against the simulator's truth (SIM).

  engaged     "Rangefinder engaged at" height, and the true height then
  flare       the source of the flare decision: rangefinder (in range at the
              flare) or baro
  tier        touchdown detection tier (TDET: 1 ground visible, 2 blind zone,
              3 not flying) and its delay after the true first contact
              ("SIM Hit ground"); if the logger dropped TDET, from TDLN
  td          true touchdown point along the runway from the true roll start
  step        landing height (ALRF.LH) jump where the rangefinder range ended,
              beyond the aircraft's own vertical motion; and the largest jump the
              stock correction rule (ALRF.LHS) would have made
  checks      FALSE ENG if engaged more than 1 m from the true height, EARLY TD if
              touchdown was detected more than 0.5 s before the true contact

usage: rf_matrix.py RUNDIR [RUNDIR ...]

AP_FLAKE8_CLEAN
"""
import glob
import json
import math
import os
import re
import sys

from pymavlink import mavutil


def load(d):
    logs = sorted(glob.glob(os.path.join(d, 'logs', '*.BIN')), key=os.path.getmtime)
    if not logs:
        return None
    m = mavutil.mavlink_connection(logs[-1])
    r = dict(sim=[], msg=[], alrf=[], tdet=[], mode=[], tdln=[])
    while True:
        x = m.recv_match(type=['SIM', 'MSG', 'ALRF', 'TDET', 'MODE', 'TDLN'])
        if x is None:
            break
        t = x.TimeUS / 1e6
        ty = x.get_type()
        if ty == 'SIM':
            r['sim'].append((t, x.Lat, x.Lng, x.Alt))
        elif ty == 'MSG':
            r['msg'].append((t, x.Message))
        elif ty == 'ALRF':
            r['alrf'].append((t, x.InR, x.LH, x.LHS, x.VZ, x.St, x.Hold))
        elif ty == 'TDET':
            r['tdet'].append((t, x.Tier, x.Dly))
        elif ty == 'MODE':
            r['mode'].append((t, x.Mode))
        elif ty == 'TDLN':
            r['tdln'].append((t, x.Src))
    return r


def nearest(rows, t):
    return min(rows, key=lambda s: abs(s[0] - t))


def analyse(d):
    r = load(d)
    if r is None or not r['sim']:
        return None
    opt = {}
    try:
        opt = json.load(open(os.path.join(d, 'result.json')))['opt']
    except (OSError, ValueError, KeyError):
        pass
    msg = r['msg']
    t_to = next((t for t, mo in r['mode'] if mo == 13), r['sim'][0][0])
    rest = nearest(r['sim'], t_to)
    ground = rest[3]

    def true_h(t):
        return nearest(r['sim'], t)[3] - ground

    out = dict(run=os.path.basename(d.rstrip('/')), prof=opt.get('rfprof', '-'))
    eng = next(((t, float(re.search(r'at ([\d.]+)m', mm).group(1))) for t, mm in msg if 'Rangefinder engaged' in mm), None)
    flare = next((t for t, mm in msg if mm.startswith('Flare')), None)
    hit = next((t for t, mm in msg if 'Hit ground' in mm), None)
    hdg = next((float(re.search(r'direction= *(-?\d+)', mm).group(1)) for t, mm in msg if 'direction=' in mm), None)
    out['eng'] = '%.2f (true %.2f)' % (eng[1], true_h(eng[0])) if eng else 'no'
    out['false_eng'] = bool(eng and abs(eng[1] - true_h(eng[0])) > 1.0)
    out['eng_before_flare'] = bool(eng and flare and eng[0] < flare)
    if flare and r['alrf']:
        before = [a for a in r['alrf'] if a[0] <= flare]
        out['flare_src'] = 'rangefinder' if before and before[-1][1] else 'baro'
    else:
        out['flare_src'] = '-'
    tdet = r['tdet'][0] if r['tdet'] else None
    if tdet is None and r['tdln'] and r['tdln'][0][1] in (1, 2):
        # TDET dropped by the logger (DSF.Dp, a busy SITL host): the touchdown
        # learning confirms a rangefinder touchdown exactly 2 s after detecting it
        tdet = (r['tdln'][0][0] - 2.0, r['tdln'][0][1], 0)
    if tdet and hit:
        t, tier, dly = tdet
        out['tier'] = tier
        out['td_delay'] = t - hit
        out['early_td'] = t < hit - 0.5
    else:
        out['tier'] = '-'
        out['td_delay'] = None
        out['early_td'] = False
    if hit and hdg is not None:
        s = nearest(r['sim'], hit)
        dn = (s[1] - rest[1]) * 111111
        de = (s[2] - rest[2]) * 111111 * math.cos(math.radians(s[1]))
        h = math.radians(hdg)
        out['td_m'] = dn * math.cos(h) + de * math.sin(h)
    else:
        out['td_m'] = None
    # landing height steps where the range ends, beyond the vertical motion
    step = stock = 0.0
    a = sorted(r['alrf'])
    end_t = (hit or a[-1][0] if a else 0) + 8
    for p, q in zip(a, a[1:]):
        if q[0] > end_t:
            break
        dt = q[0] - p[0]
        motion = -q[4] * dt
        if p[1] and not q[1]:
            step = max(step, abs(q[2] - p[2] - motion))
        if not q[1]:
            stock = max(stock, abs(q[3] - p[3] - motion))
    out['step'] = step
    out['stock_step'] = stock
    return out


def main():
    rows = [x for x in (analyse(d) for d in sys.argv[1:]) if x]
    print('%-14s %-9s %-16s %-5s %-11s %-4s %-8s %-8s %-7s %-7s %s' % (
        'run', 'profile', 'engaged (true)', 'pre', 'flare by', 'tier', 'tier dly', 'td m', 'step', 'stock', 'checks'))
    for x in rows:
        checks = ' '.join(c for c, bad in (('FALSE-ENG', x['false_eng']), ('EARLY-TD', x['early_td'])) if bad) or 'ok'
        print('%-14s %-9s %-16s %-5s %-11s %-4s %-8s %-8s %-7.2f %-7.2f %s' % (
            x['run'], x['prof'], x['eng'], 'yes' if x['eng_before_flare'] else 'no', x['flare_src'], x['tier'],
            '%+.2f s' % x['td_delay'] if x['td_delay'] is not None else '-',
            '%+.1f' % x['td_m'] if x['td_m'] is not None else '-', x['step'], x['stock_step'], checks))


if __name__ == '__main__':
    main()
