#!/usr/bin/env python3
"""SITL: baro compensation, takeoff learning and rangefinder engagement, from the
dataflash log of an autoland_circuit_test.py run.

usage: baro_check.py <outdir> [<outdir> ...]   (reads the newest <outdir>/logs/*.BIN)

  - BTLN: what AUTOLAND_BAROLRN measured on the takeoff run-up
  - run-up: BARO.Alt change from parked while TAKEOFF is selected and the aircraft
    is still below 0.8 m/s (should stay within +-0.3 m once compensated)
  - airborne: BARO.Alt - true height (SIM.Alt), by throttle band, above 5 m
  - rangefinder: "Rangefinder engaged" height, against the true height then
"""
import glob
import os
import sys

import numpy as np
from pymavlink import mavutil


def load(path):
    m = mavutil.mavlink_connection(path)
    D = {k: [] for k in ('BARO', 'SIM', 'RCOU', 'GPS', 'MODE', 'MSG', 'BTLN', 'PARM')}
    while True:
        x = m.recv_match(type=list(D))
        if x is None:
            break
        t = x.get_type()
        if t == 'BARO' and x.I != 0:
            continue
        if t == 'PARM':
            D[t].append((x.Name, x.Value))
            continue
        T = x.TimeUS / 1e6
        if t == 'BARO':
            D[t].append((T, x.Alt))
        elif t == 'SIM':
            D[t].append((T, x.Alt))
        elif t == 'RCOU':
            D[t].append((T, x.C3))
        elif t == 'GPS':
            D[t].append((T, x.Spd))
        elif t == 'MODE':
            D[t].append((T, x.Mode))
        elif t == 'MSG':
            D[t].append((T, x.Message))
        elif t == 'BTLN':
            D[t].append((T, x.N, x.Dur, x.ThMx, x.Meas, x.Old, x.New, x.BSd, x.Res))
    return D


def interp(arr, ts, i=1):
    a = np.array(arr, dtype=float)
    return np.interp(ts, a[:, 0], a[:, i])


def check(d):
    logs = sorted(glob.glob(os.path.join(d, 'logs', '*.BIN')), key=os.path.getmtime)
    if not logs:
        print('%s: no logs' % d)
        return
    D = load(logs[-1])
    p = dict(D['PARM'])
    print('== %s (%s)' % (d, os.path.basename(logs[-1])))
    print('  SIM_BARO_THST %s  SIM_BARO_WCF_FWD %s | BARO1_THST_SCALE %s  BARO1_WCF_ENABLE %s  BARO1_WCF_FWD %s'
          % (p.get('SIM_BARO_THST'), p.get('SIM_BARO_WCF_FWD'), p.get('BARO1_THST_SCALE'),
             p.get('BARO1_WCF_ENABLE'), p.get('BARO1_WCF_FWD')))
    res = ['accepted', 'run-up too short', 'too noisy', 'out of range', 'no still baseline']
    for t, n, dur, thmx, meas, old, new, bsd, r in D['BTLN']:
        print('  BTLN %.0f s: %s, %u samples over %.1f s, max throttle %.0f%%, measured %.1f Pa, '
              'scale %.1f -> %.1f, baseline sd %.1f Pa' % (t, res[int(r)], n, dur, thmx * 100, meas, old, new, bsd))

    B = np.array(D['BARO'])
    S = np.array(D['SIM'])
    ground = np.median(S[:20, 1])
    # run-up: from TAKEOFF selection until 0.8 m/s
    for tm, mode in D['MODE']:
        if mode != 13:
            continue
        g = np.array(D['GPS'])
        moving = g[(g[:, 0] > tm) & (g[:, 1] >= 0.8)]
        t_end = moving[0, 0] if len(moving) else tm + 10
        pre = B[(B[:, 0] > tm - 2) & (B[:, 0] < tm)]
        run = B[(B[:, 0] >= tm) & (B[:, 0] < t_end)]
        if len(pre) < 3 or len(run) < 3:
            continue
        dev = run[:, 1] - np.median(pre[:, 1])
        thr = (interp(D['RCOU'], run[:, 0]) - 1000) / 10
        print('  run-up %.1f s long, throttle up to %.0f%%: baro change %+.2f to %+.2f m (at max throttle %+.2f m)'
              % (t_end - tm, thr.max(), dev.min(), dev.max(), dev[thr >= thr.max() - 5].mean()))
        break

    # airborne error by throttle band
    h = interp(D['SIM'], B[:, 0]) - ground
    thr = np.clip((interp(D['RCOU'], B[:, 0]) - 1000) / 10, 0, 100)
    err = B[:, 1] - h
    air = h > 5
    if air.sum() > 50:
        bands = [(0, 20), (20, 40), (40, 60), (60, 101)]
        txt = []
        for lo, hi in bands:
            k = air & (thr >= lo) & (thr < hi)
            if k.sum() > 20:
                txt.append('%u-%u%%: %+.2f m (n=%u)' % (lo, min(hi, 100), np.median(err[k]), k.sum()))
        print('  airborne baro - true height: ' + ', '.join(txt))

    for t, msg in D['MSG']:
        if 'Rangefinder engaged' in msg or 'Flare' in msg or 'Baro:' in msg:
            print('  %.1f s %s (true height %.2f m)' % (t, msg, interp(D['SIM'], [t])[0] - ground))


for d in sys.argv[1:]:
    check(d)
