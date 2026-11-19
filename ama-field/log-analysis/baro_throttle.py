#!/usr/bin/env python3
"""
Baro error vs throttle and airspeed, from ArduPlane dataflash logs.

Fuselage pressurisation (prop wash and ram air entering the cowling) makes the
baro read too low. This script measures it three ways, with no BARO messages
needed (AHR2.Alt is the DCM altitude, taken from the baro):

  1. takeoff roll: baro change from parked as the throttle comes up, while still
     on the ground (the first ~2 s are a static run-up test)
  2. near-ground flight: baro vs strong rangefinder returns (RFSS from the
     TOFSense-F driver, or RFND with a signal quality), throttle below 10%,
     0.5-3 m; baseline = parked after landing with the motor off
  3. airborne: least-squares fit of (baro - GPS altitude) against throttle and
     airspeed squared, with an offset and a linear drift term

With compensation already on (BARO1_THST_SCALE, BARO1_WCF_ENABLE/_FWD, read from
the log's PARM messages) everything measured is a residual: the suggested values
are the current value plus the increment, and both are printed.

  --replay  run the AUTOLAND_BAROLRN algorithm (the firmware's takeoff learning)
            on every ground TAKEOFF in the logs and show what it would learn,
            starting from BARO1_THST_SCALE 0
  --models  compare throttle models fitted to the run-ups (line through the
            origin, line with an offset, throttle squared) by how well each
            corrects the airborne baro error

Usage: baro_throttle.py [--replay] [--models] LOG.BIN [LOG2.BIN ...]
Needs pymavlink and numpy. The throttle output is the RCOU channel whose
SERVOn_FUNCTION is 70, scaled with its SERVOn_MIN/MAX.

AP_FLAKE8_CLEAN
"""
import argparse

import numpy as np
from pymavlink import mavutil

TAKEOFF = 13

# AUTOLAND_BAROLRN constants (ArduPlane/mode_autoland.cpp)
LRN_BASE_S = 1.0          # baseline: last second parked
LRN_BASE_MIN_S = 0.5
LRN_PARK_THR = 0.05
LRN_PARK_GS = 0.5
LRN_ROLL_GS = 0.8
LRN_ROLL_ACCEL = 1.0
LRN_FIT_THR = 0.25
LRN_MIN_DUR = 1.0
LRN_MIN_THR_MAX = 0.35
LRN_MIN_SPAN = 0.2
LRN_MAX_BASE_SD = 4.0
LRN_MAX_SCALE = 150.0
LRN_MAX_DUR = 10.0


def load(path):
    m = mavutil.mavlink_connection(path)
    D = {k: [] for k in ('AHR2', 'GPS', 'CTUN', 'RFND', 'RFSS', 'MODE', 'RCOU', 'BTLN')}
    D['PARM'] = {}
    D['PARM_T'] = []
    while True:
        x = m.recv_match(type=list(D))
        if x is None:
            break
        t = x.get_type()
        if t == 'PARM':
            D['PARM'].setdefault(x.Name, x.Value)
            D['PARM_T'].append((x.TimeUS / 1e6, x.Name, x.Value))
            continue
        T = x.TimeUS / 1e6
        if t == 'AHR2':
            D[t].append((T, x.Alt))
        elif t == 'GPS':
            if getattr(x, 'I', 0) == 0:
                D[t].append((T, x.Alt, x.Spd, x.Status))
        elif t == 'CTUN':
            D[t].append((T, x.ThO, x.As))
        elif t == 'RFND':
            if getattr(x, 'Instance', 0) == 0:
                D[t].append((T, x.Dist, x.Stat, getattr(x, 'Quality', -1)))
        elif t == 'RFSS':
            D[t].append((T, x.D, x.Str, x.St))
        elif t == 'MODE':
            D[t].append((T, x.Mode))
        elif t == 'RCOU':
            D[t].append((T,) + tuple(getattr(x, 'C%u' % i, 0) for i in range(1, 15)))
        elif t == 'BTLN':
            D[t].append((T, x.N, x.ThMx, x.Meas, x.Old, x.New, x.BSd, x.Res))
    return D


def interp(arr, ts, i=1):
    a = np.asarray(arr, dtype=float)
    return np.interp(ts, a[:, 0], a[:, i])


def throttle_channel(P):
    for n in range(1, 15):
        if int(P.get('SERVO%u_FUNCTION' % n, -1)) == 70:
            return n, P.get('SERVO%u_MIN' % n, 1100), P.get('SERVO%u_MAX' % n, 1900)
    return 3, P.get('SERVO3_MIN', 1100), P.get('SERVO3_MAX', 1900)


def thr_frac(D, ts):
    """throttle output 0..1 as the firmware sees it (slew-limited output, from RCOU)"""
    ch, lo, hi = D['thr_ch']
    return np.clip((interp(D['RCOU'], ts, ch) - lo) / max(hi - lo, 1), 0, 1)


def param_at(D, name, t):
    """value of a parameter at time t (PARM messages are logged on every change)"""
    v = D['PARM'].get(name)
    for tp, n, val in D['PARM_T']:
        if n == name and tp <= t:
            v = val
    return v


def ground_takeoffs(D):
    """(entry time, throttle rise time) of each TAKEOFF selected while parked"""
    out = []
    G = D['GPS']
    for tm, mode in D['MODE']:
        if mode != TAKEOFF:
            continue
        if interp(G, [tm], 2)[0] > LRN_PARK_GS:
            print('  TAKEOFF at %.0f s selected while moving or flying: skipped' % tm)
            continue
        ch, lo, hi = D['thr_ch']
        rise = next((r[0] for r in D['RCOU'] if tm <= r[0] < tm + 60 and
                     (r[ch] - lo) / max(hi - lo, 1) >= LRN_PARK_THR), None)
        if rise is not None:
            out.append((tm, rise))
    return out


def takeoff_rolls(D):
    out = []
    A = np.array(D['AHR2'])
    pa = D['pa_per_m']
    for tm, rise in ground_takeoffs(D):
        lo = next((r[0] for r in D['RFND'] if r[0] > rise and r[1] > 0.5 and r[2] == 4), None)
        pre = A[(A[:, 0] > rise - 6) & (A[:, 0] < rise - 0.3)]
        if len(pre) < 3 or max(interp(D['GPS'], pre[:, 0], 2)) > LRN_PARK_GS:
            continue
        base = np.median(pre[:, 1])
        scale = param_at(D, 'BARO1_THST_SCALE', rise) or 0
        print('  takeoff roll at %.0f s%s, BARO1_THST_SCALE %.0f during it:'
              % (rise, ' (liftoff %.1f s later)' % (lo - rise) if lo else '', scale))
        print('     t   thr%  gs m/s  baro change m')
        t = rise - 0.5
        while t < (lo or rise + 8) + 0.6:
            sel = A[(A[:, 0] > t - 0.25) & (A[:, 0] <= t + 0.25)]
            if len(sel):
                print('  %4.1f %5.0f %6.1f %9.2f' % (t - rise, thr_frac(D, [t])[0] * 100,
                                                     interp(D['GPS'], [t], 2)[0], sel[:, 1].mean() - base))
            t += 0.5
        st = A[(A[:, 0] > rise) & (A[:, 0] < (lo or rise + 8))]
        g = interp(D['GPS'], st[:, 0], 2)
        th = thr_frac(D, st[:, 0])
        k = (g < 1.0) & (th > 0.3)
        if k.sum() >= 2:
            # residual in m per 100% throttle, and in Pa of BARO1_THST_SCALE
            res_m = ((st[k, 1] - base) / th[k]).mean()
            out.append((res_m, scale, -res_m * pa))
    return out


def strong_rf(D):
    """(t, distance) of strong rangefinder samples: RFSS strength >= 128, or RFND quality >= 50"""
    if D['RFSS']:
        return [(s[0], s[1]) for s in D['RFSS'] if s[3] == 1 and s[2] >= 128]
    return [(r[0], r[1]) for r in D['RFND'] if r[2] == 4 and r[3] >= 50]


def near_ground(D):
    S = strong_rf(D)
    if not S:
        print('  near ground: no strong rangefinder samples (RFSS, or RFND with a quality)')
        return []
    ts = np.array([s[0] for s in S])
    d = np.array([s[1] for s in S])
    b = interp(D['AHR2'], ts)
    gs = interp(D['GPS'], ts, 2)
    thr = thr_frac(D, ts)
    asp = interp(D['CTUN'], ts, 2)
    pa = D['pa_per_m']
    rho = D['rho']
    fly = np.where((gs > 5) & (asp > 5))[0]
    rows = []
    groups = np.split(fly, np.where(np.diff(ts[fly]) > 20)[0] + 1) if len(fly) else []
    for g in groups:
        tend = ts[g[-1]]
        park = np.where((ts > tend) & (ts < tend + 90) & (d < 0.4) & (gs < 0.3) & (thr < 0.05))[0]
        if len(park) < 3:
            continue
        d_park = np.median(d[park])
        base = np.median(b[park])
        clean = g[(d[g] >= 0.5) & (d[g] <= 3.0) & (thr[g] < 0.10)]
        if len(clean) < 3:
            print('  landing ending %.0f s: only %u clean samples (throttle < 10%%, strong, 0.5-3 m)'
                  % (tend, len(clean)))
            continue
        e = b[clean] - (d[clean] - d_park) - base
        q = 0.5 * rho * asp[clean] ** 2
        inc = -np.median(e) * pa / np.median(q)
        print('  landing ending %.0f s: n=%3u  baro error %+5.2f m  airspeed %4.1f m/s  -> BARO1_WCF_FWD %+.2f'
              % (tend, len(clean), np.median(e), np.median(asp[clean]), inc))
        rows += [(e[k], q[k]) for k in range(len(clean))]
    return rows


def airborne_data(D):
    G = [g for g in D['GPS'] if g[3] >= 3]
    if len(G) < 100:
        return None
    ts = np.array([g[0] for g in G])
    galt = np.array([g[1] for g in G])
    b = interp(D['AHR2'], ts)
    thr = thr_frac(D, ts)
    asp = interp(D['CTUN'], ts, 2)
    m = ((galt - np.percentile(galt, 5)) > 8) & (asp > 8)
    if m.sum() < 60:
        return None
    # uncompensated error: take back out what BARO1_THST_SCALE and BARO1_WCF_FWD added
    # (each raises the baro altitude by its pressure correction / Pa per m)
    pa = D['pa_per_m']
    comp = np.array([(param_at(D, 'BARO1_THST_SCALE', t) or 0) for t in ts[m]]) * thr[m]
    if D['PARM'].get('BARO1_WCF_ENABLE', 0):
        comp = comp + (D['PARM'].get('BARO1_WCF_FWD', 0) or 0) * 0.5 * D['rho'] * asp[m] ** 2
    return ts[m], b[m] - galt[m] - comp / pa, thr[m], asp[m]


def airborne(D):
    a = airborne_data(D)
    if a is None:
        return None
    ts, e, thr, asp = a
    X = np.column_stack([np.ones(len(ts)), ts - ts.mean(), thr, asp ** 2])
    co, *_ = np.linalg.lstsq(X, e, rcond=None)
    r = e - X @ co
    print('  airborne fit: %+5.2f m per 100%% throttle, %+5.2f m at 15 m/s airspeed '
          '(resid sd %.2f m, n=%u, airspeed %.0f-%.0f m/s)'
          % (co[2], co[3] * 225, r.std(), len(ts), np.percentile(asp, 5), np.percentile(asp, 95)))
    return co[2]


def replay_takeoff(D, tm, rise, scale):
    """the firmware's AUTOLAND_BAROLRN on one ground takeoff. Pressure rise is taken
    from AHR2.Alt (no BARO in these logs) with any thrust compensation added back."""
    A = np.array(D['AHR2'])
    pa = D['pa_per_m']
    comp = param_at(D, 'BARO1_THST_SCALE', rise) or 0

    def raw_press(t):
        # altitude drop -> pressure rise, plus what the compensation already took out
        return -interp(A, t) * pa + comp * thr_frac(D, t)

    # baseline: last second parked, throttle under 5%, below 0.5 m/s
    tb = A[(A[:, 0] > rise - LRN_BASE_S) & (A[:, 0] < rise), 0]
    tb = tb[(interp(D['GPS'], tb, 2) < LRN_PARK_GS) & (thr_frac(D, tb) < LRN_PARK_THR)]
    res = dict(t=rise, n=0, dur=0, thr_max=0, meas=0, old=scale, new=scale, base_sd=0)
    if len(tb) < 2 or tb[-1] - tb[0] < LRN_BASE_MIN_S - 0.15:
        res['reason'] = 'no still baseline'
        return res
    pb = raw_press(tb)
    base = np.median(pb)
    res['base_sd'] = pb.std()
    # samples while still: below 0.8 m/s, forward acceleration (from GPS speed) under 1 m/s/s
    ts = A[(A[:, 0] >= rise) & (A[:, 0] < rise + LRN_MAX_DUR), 0]
    if len(ts) < 2:
        res['reason'] = 'run-up too short'
        return res
    gs = interp(D['GPS'], ts, 2)
    acc = np.gradient(gs, ts)
    acc_f = np.zeros_like(acc)
    for i in range(1, len(acc)):
        acc_f[i] = acc_f[i - 1] + 0.3 * (acc[i] - acc_f[i - 1])
    still = (gs < LRN_ROLL_GS) & (acc_f < LRN_ROLL_ACCEL)
    end = np.argmin(still) if not still.all() else len(ts)
    ts = ts[:end]
    if len(ts) < 2:
        res['reason'] = 'run-up too short'
        return res
    th = thr_frac(D, ts)
    dp = raw_press(ts) - base
    fit = th >= LRN_FIT_THR
    res.update(n=len(ts), dur=ts[-1] - ts[0], thr_max=th.max())
    if res['dur'] < LRN_MIN_DUR or th.max() < LRN_MIN_THR_MAX or th.max() - th.min() < LRN_MIN_SPAN or fit.sum() < 3:
        res['reason'] = 'run-up too short'
    elif res['base_sd'] >= LRN_MAX_BASE_SD:
        res['reason'] = 'too noisy'
    else:
        meas = (th[fit] * dp[fit]).sum() / (th[fit] ** 2).sum()
        res['meas'] = meas
        if abs(meas) > LRN_MAX_SCALE:
            res['reason'] = 'out of range'
        else:
            res['reason'] = 'accepted'
            res['new'] = meas if scale == 0 else scale + 0.5 * (meas - scale)
    res['samples'] = (th, dp)
    return res


def model_fits(results):
    """pooled run-up samples -> coefficients of the three models (Pa)"""
    th = np.concatenate([r['samples'][0] for r in results if 'samples' in r])
    dp = np.concatenate([r['samples'][1] for r in results if 'samples' in r])
    k = th >= LRN_FIT_THR
    lin = (th[k] * dp[k]).sum() / (th[k] ** 2).sum()
    X = np.column_stack([np.ones(k.sum()), th[k]])
    off, slope = np.linalg.lstsq(X, dp[k], rcond=None)[0]
    quad = ((th[k] ** 2) * dp[k]).sum() / (th[k] ** 4).sum()
    return dict(lin=lin, off=off, slope=slope, quad=quad, n=int(k.sum()))


def compare_models(logs_air, mf, pa):
    models = {
        'none': lambda t: 0 * t,
        'linear through 0 (BARO1_THST_SCALE %.0f)' % mf['lin']: lambda t: mf['lin'] * t,
        'linear with offset (%.0f + %.0f x thr)' % (mf['off'], mf['slope']):
            lambda t: np.where(t > 0.05, mf['off'] + mf['slope'] * t, 0),
        'throttle squared (%.0f x thr^2)' % mf['quad']: lambda t: mf['quad'] * t ** 2,
    }
    print('\nModel comparison: uncompensated airborne (baro - GPS) residual sd after each throttle correction; '
          'offset, drift and airspeed^2 fitted per log (pooled run-up fit: %u samples)' % mf['n'])
    for name, f in models.items():
        sds = []
        for ts, e, thr, asp in logs_air:
            ec = e + f(thr) / pa   # the compensation raises the baro altitude by model/pa
            X = np.column_stack([np.ones(len(ts)), ts - ts.mean(), asp ** 2])
            r = ec - X @ np.linalg.lstsq(X, ec, rcond=None)[0]
            sds.append(r.std())
        print('  %-44s mean sd %.2f m  (per log: %s)' % (name, np.mean(sds), ' '.join('%.2f' % s for s in sds)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--replay', action='store_true', help='replay AUTOLAND_BAROLRN on each ground takeoff')
    ap.add_argument('--models', action='store_true', help='compare run-up throttle models on the airborne error')
    ap.add_argument('logs', nargs='+')
    args = ap.parse_args()

    static, air, rows, replays, logs_air = [], [], [], [], []
    wcf_now = 0.0
    thst_now = 0.0
    scale = 0.0
    pa = 11.9
    for p in args.logs:
        print(p)
        D = load(p)
        if not D['AHR2']:
            print('  no AHR2 data')
            continue
        P = D['PARM']
        D['thr_ch'] = throttle_channel(P)
        gnd = P.get('BARO1_GND_PRESS') or 101325.0
        D['rho'] = gnd / (287.05 * 288.15)
        D['pa_per_m'] = pa = D['rho'] * 9.80665
        wcf_now = (P.get('BARO1_WCF_FWD', 0) or 0) if P.get('BARO1_WCF_ENABLE', 0) else 0.0
        thst_now = P.get('BARO1_THST_SCALE', 0) or 0
        print('  BARO1_THST_SCALE %s, BARO1_WCF_ENABLE %s, BARO1_WCF_FWD %s; throttle on SERVO%u'
              % (P.get('BARO1_THST_SCALE', 'n/a'), P.get('BARO1_WCF_ENABLE', 'n/a'), P.get('BARO1_WCF_FWD', 'n/a'),
                 D['thr_ch'][0]))
        for b in D['BTLN']:
            print('  BTLN (firmware learning): measured %.1f Pa, scale %.1f -> %.1f, result %u' % (b[3], b[4], b[5], b[7]))
        static += takeoff_rolls(D)
        a = airborne(D)
        if a is not None:
            air.append(a)
        rows += near_ground(D)
        if args.replay or args.models:
            for tm, rise in ground_takeoffs(D):
                r = replay_takeoff(D, tm, rise, scale)
                r['log'] = p
                replays.append(r)
                if r['reason'] == 'accepted':
                    scale = r['new']
        if args.models:
            ad = airborne_data(D)
            if ad is not None:
                logs_air.append(ad)
    print('\nSummary (negative = baro reads LOW, i.e. fuselage pressure above ambient)')
    if static:
        # each roll: the scale in use plus what was still left in the baro
        absolute = [scale_then + inc for _, scale_then, inc in static]
        print('  static run-up, per roll: %s' % ', '.join('%.0f %+.0f' % (s, i) for _, s, i in static))
        print('     -> BARO1_THST_SCALE %.0f Pa (mean of the rolls; current %.0f, increment %+.0f)'
              % (np.mean(absolute), thst_now, np.mean(absolute) - thst_now))
    if air:
        print('  airborne fits (uncompensated): median %+5.2f m per 100%% throttle (range %.1f to %.1f)'
              ' -> about %.0f Pa' % (np.median(air), min(air), max(air), -np.median(air) * pa))
    if rows:
        r = np.array(rows)
        e = np.median(r[:, 0])
        q = np.median(r[:, 1])
        inc = -e * pa / q
        print('  near ground at idle (all landings): %+5.2f m at q %.0f Pa (airspeed %.1f m/s)'
              % (e, q, np.sqrt(2 * q / 1.225)))
        print('     -> BARO1_WCF_FWD %.2f %+.2f = %.2f (set BARO1_WCF_ENABLE 1)' % (wcf_now, inc, wcf_now + inc))
    if replays:
        print('\nAUTOLAND_BAROLRN replay (starting from BARO1_THST_SCALE 0):')
        for r in replays:
            print('  %s %5.0f s: %-17s %3u samples %4.1f s, max thr %3.0f%%, baseline sd %4.1f Pa, '
                  'measured %5.1f Pa, scale %5.1f -> %5.1f'
                  % (r['log'].split('/')[-1].split('\\')[-1], r['t'], r['reason'], r['n'], r['dur'],
                     r['thr_max'] * 100, r['base_sd'], r['meas'], r['old'], r['new']))
    if args.models:
        ok = [r for r in replays if 'samples' in r and (r['samples'][0] >= LRN_FIT_THR).sum() >= 3]
        if ok and logs_air:
            compare_models(logs_air, model_fits(ok), pa)
        else:
            print('\nModel comparison: needs at least one run-up and one log with airborne GPS data')


if __name__ == '__main__':
    main()
