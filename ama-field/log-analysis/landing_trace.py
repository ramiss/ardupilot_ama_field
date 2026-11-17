import sys, bisect, math
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1]); t0, t1 = float(sys.argv[2]), float(sys.argv[3])
D = {k: [] for k in ('CTUN','POS','GPS','TECS','NTUN','STAT','RFND','RCOU','ATT')}
while True:
    x = m.recv_match(type=list(D))
    if x is None: break
    D[x.get_type()].append(x.to_dict())
def at(k, t):
    L = D[k]; ts = [r['TimeUS'] for r in L]; return L[max(bisect.bisect_right(ts, t*1e6)-1, 0)]
print('   t   alt  hdem   rf   wpdist  gs    As   spdem  thr  pitch(nav)  sink  stage')
t = t0
while t <= t1:
    c=at('CTUN',t); p=at('POS',t); g=at('GPS',t); e=at('TECS',t); n=at('NTUN',t); s=at('STAT',t); r=at('RFND',t)
    print('%5.1f %5.1f %5.1f %5.2f %6.0f %5.1f %5.1f %5.1f %4.0f %5.1f(%5.1f) %5.1f %d' % (t, p['RelHomeAlt'], e['hdem'], r['Dist'], n['Dist'], g['Spd'], c['As'], e['spdem'], c['ThO'], c['Pitch'], c['NavPitch'], -g['VZ'] if 'VZ' in g else 0, s['Stage']))
    t += 0.5
