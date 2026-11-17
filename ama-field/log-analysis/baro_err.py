import sys, bisect, statistics
from pymavlink import mavutil
for path in sys.argv[1:]:
    m = mavutil.mavlink_connection(path)
    S=[];P=[];G=[];R=[]
    while True:
        x = m.recv_match(type=['RFSS','POS','GPS','RCOU'])
        if x is None: break
        t=x.get_type()
        if t=='RFSS': S.append((x.TimeUS/1e6,x.D,x.Str,x.St))
        elif t=='POS': P.append((x.TimeUS/1e6,x.RelHomeAlt))
        elif t=='GPS': G.append((x.TimeUS/1e6,x.Spd))
        else: R.append((x.TimeUS/1e6,x.C3))
    pt=[p[0] for p in P]; gt=[g[0] for g in G]; rt=[r[0] for r in R]
    fly=[]; taxi=[]; still=[]
    for t,d,s,st in S:
        if st!=1 or s<150: continue
        b=P[max(bisect.bisect_right(pt,t)-1,0)][1]; gs=G[max(bisect.bisect_right(gt,t)-1,0)][1]
        thr=R[max(bisect.bisect_right(rt,t)-1,0)][1]
        err=b-(d-0.1)
        if gs>8 and d>0.5: fly.append(err)
        elif d<0.3 and 1.5<gs<6: taxi.append(err)
        elif d<0.3 and gs<0.3 and thr<1100: still.append(err)
    f=lambda L: ('%5.1f m (n=%d)' % (statistics.median(L),len(L))) if L else '   n/a'
    print('%-14s baro error: low flight near ground %s | taxiing %s | parked, motor off %s' % (path.split('/')[0], f(fly), f(taxi), f(still)))
