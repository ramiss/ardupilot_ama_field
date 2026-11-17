import sys, math, collections
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], robust_parsing=True)
d = collections.defaultdict(list); P = {}
while True:
    x = m.recv_match()
    if x is None: break
    t = x.get_type()
    if t == 'PARM': P[x.Name] = x.Value; continue
    if t in ('MODE','MSG','ATT','CTUN','NTUN','GPS','BAT','RCIN','RCOU','POS','TECS','PIDR','PIDP','PIDS','RFND','XKF4','STAT','EV','ERR','ARM','FNCE','ARSP','AETR','XKF3','XKFS','XKQ','PM','POWR','ORGN','CMD'):
        d[t].append(x.to_dict())
def T(r): return r['TimeUS']/1e6
print('PARAMS', {k: P.get(k) for k in ['FENCE_TYPE','FENCE_RADIUS','FENCE_ALT_MAX','FENCE_OPTIONS','FENCE_TOTAL','AIRSPEED_CRUISE','AIRSPEED_MIN','AIRSPEED_MAX','TRIM_THROTTLE','THR_MAX','TKOFF_TDRAG_ELEV','TKOFF_TDRAG_SPD1','TKOFF_GND_PITCH','PTCH_TRIM_DEG','RLL_RATE_P','PTCH_RATE_P','RC9_OPTION','RC10_OPTION','FLTMODE1','LEVEL_ROLL_LIMIT','ROLL_LIMIT_DEG','BATT_CAPACITY','SCALING_SPEED','ARSPD_TYPE','RNGFND1_MAX','STEER2SRV_P','STEER2SRV_I','STEER2SRV_TCONST','TKOFF_ROTATE_SPD']})
mm = {v:k for k,v in mavutil.mode_mapping_apm.items()}
print('MODES'); [print('  %.1f %s reason=%s' % (T(r), mavutil.mode_mapping_apm.get(r['ModeNum'], r['ModeNum']), r.get('Rsn'))) for r in d['MODE']]
print('MSG'); [print('  %.1f %s' % (T(r), r['Message'])) for r in d['MSG']]
print('EV/ERR/ARM'); [print('  %.1f %s %s' % (T(r), k, {a:b for a,b in r.items() if a not in ('mavpackettype','TimeUS')})) for k in ('EV','ERR','ARM') for r in d[k]]
print('GPS first/last', {k: d['GPS'][0].get(k) for k in ('Status','NSats','HDop','Lat','Lng')}, 'min sats', min(r['NSats'] for r in d['GPS']), 'max hdop', max(r['HDop'] for r in d['GPS']))
b = d['BAT']; print('BAT V start %.2f min %.2f end %.2f, max curr %.1f, used %.0f mAh' % (b[0]['Volt'], min(r['Volt'] for r in b), b[-1]['Volt'], max(r.get('Curr',0) for r in b), b[-1].get('CurrTot',0)))
