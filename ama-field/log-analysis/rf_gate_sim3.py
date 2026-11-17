import sys, bisect
from pymavlink import mavutil
path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
m = mavutil.mavlink_connection(path)
S=[]; P=[]
while True:
    x=m.recv_match(type=['RFSS','POS'])
    if x is None: break
    if x.get_type()=='RFSS': S.append((x.TimeUS/1e6, x.D, x.Str, x.St))
    else: P.append((x.TimeUS/1e6, x.RelHomeAlt))
pt=[p[0] for p in P]; MAX=6.0
def truth(t):  # baro reads ~1.5m low near the ground at speed; use rf when strong as truth proxy
    return P[max(bisect.bisect_right(pt,t)-1,0)][1]
for name, gate, smin in (('stock', False, 0), ('v5 gate', True, 0), ('v7 gate+strong bypass', 'bypass', 0)):
    cnt=0; last=-1; init=None; inr=False; eng=[]
    for t,d,s,st in S:
        if not (t0<=t<=t1): continue
        baro=truth(t)
        ok = st==1 and 0.02<d<MAX*0.95 and s>=smin
        if ok and gate and not (gate=='bypass' and s>=128) and abs(d-baro) > 2.5+0.2*abs(baro): ok=False
        if ok:
            if init is None: init=d
            if cnt<10:
                if d!=last and abs(init-d)>0.05*MAX: cnt+=1
                if abs(last-d)>MAX*0.2: cnt=0
            else: inr=True
            last=d
        else: cnt=0; inr=False
        if inr: eng.append((round(t,1), round(baro,1), round(d,2), s))
    weak=[e for e in eng if e[3]<125]
    print('%-22s in-range samples %3d, with weak (garbage-like) signal %3d, first: %s' % (name, len(eng), len(weak), eng[0] if eng else None))
    if weak: print('     weak examples:', weak[:5])
