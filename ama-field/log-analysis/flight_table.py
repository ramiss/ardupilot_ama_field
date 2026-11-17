import sys, math, bisect
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1])
step = float(sys.argv[2]) if len(sys.argv) > 2 else 2
t0s = float(sys.argv[3]) if len(sys.argv) > 3 else 0; t1s = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
D = {k: [] for k in ('CTUN','POS','GPS','RCOU','RCIN','STAT','MODE','RFND','NTUN','TECS')}
home = None
while True:
    x = m.recv_match(type=list(D.keys()) + ['ORGN','CMD'])
    if x is None: break
    t = x.get_type()
    if t == 'ORGN' and x.Type == 1: home = (x.Lat, x.Lng)
    elif t in D: D[t].append(x.to_dict())
def at(k, t):
    L = D[k]; ts = [r['TimeUS'] for r in L]; i = bisect.bisect_right(ts, t*1e6) - 1
    return L[max(i, 0)] if L else {}
if home is None: home = (D['GPS'][0]['Lat'], D['GPS'][0]['Lng'])
print('home', home)
modes = [(r['TimeUS']/1e6, mavutil.mode_mapping_apm.get(r['ModeNum'])) for r in D['MODE']]
def mode_at(t):
    md = '?'
    for a, b in modes:
        if a <= t: md = b
    return md
t = max(D['CTUN'][0]['TimeUS']/1e6, t0s); tend = min(D['CTUN'][-1]['TimeUS']/1e6, t1s)
print('   t  mode     alt  gs   As   roll(nav)  pitch(nav)  thr  ch3in elev  rud  dist  brg  fly  rfnd')
while t <= tend:
    c = at('CTUN', t); p = at('POS', t); g = at('GPS', t); o = at('RCOU', t); i = at('RCIN', t); s = at('STAT', t); r = at('RFND', t)
    dn = (p['Lat'] - home[0]) * 111111; de = (p['Lng'] - home[1]) * 111111 * math.cos(math.radians(home[0]))
    print('%6.1f %-8s %5.1f %4.1f %4.1f %5.0f(%4.0f) %5.1f(%5.1f) %4.0f %5d %4d %4d %5.0f %4.0f %3d %5.2f' % (
        t, mode_at(t), p['RelHomeAlt'], g['Spd'], c['As'], c['Roll'], c['NavRoll'], c['Pitch'], c['NavPitch'], c['ThO'],
        i['C3'], o['C2'], o['C4'], math.hypot(dn, de), (math.degrees(math.atan2(de, dn)) + 360) % 360, s.get('isFlying', -1), r.get('Dist', -1)))
    t += step
