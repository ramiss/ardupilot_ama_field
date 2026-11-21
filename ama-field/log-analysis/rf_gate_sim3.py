import sys, bisect
from pymavlink import mavutil

def rfss_quality(x):
    """signal quality 0-100% of an RFSS sample: the logged Q, or for older logs
    (TOFSense-F only, no Q) the MT-06 strength / 255 as its driver reports it"""
    q = getattr(x, 'Q', None)
    if q is not None:
        return q
    return min(100, x.Str * 100 // 255) if x.St == 1 else 0

path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
m = mavutil.mavlink_connection(path)
S=[]; P=[]; MAX=6.0
while True:
    x=m.recv_match(type=['RFSS','POS','PARM'])
    if x is None: break
    if x.get_type()=='RFSS': S.append((x.TimeUS/1e6, x.D, rfss_quality(x), x.St))
    elif x.get_type()=='PARM':
        if x.Name=='RNGFND1_MAX': MAX=x.Value
    else: P.append((x.TimeUS/1e6, x.RelHomeAlt))
pt=[p[0] for p in P]
def truth(t):  # baro reads ~1.5m low near the ground at speed; use rf when strong as truth proxy
    return P[max(bisect.bisect_right(pt,t)-1,0)][1]
for name, gate, smin in (('stock', False, 0), ('v5 gate', True, 0), ('v7 gate+strong bypass', 'bypass', 0)):
    cnt=0; last=-1; init=None; inr=False; eng=[]
    for t,d,q,st in S:
        if not (t0<=t<=t1): continue
        baro=truth(t)
        ok = st==1 and 0.02<d<MAX*0.95 and q>=smin
        if ok and gate and not (gate=='bypass' and q>=50) and abs(d-baro) > 2.5+0.2*abs(baro): ok=False
        if ok:
            if init is None: init=d
            if cnt<10:
                if d!=last and abs(init-d)>0.05*MAX: cnt+=1
                if abs(last-d)>MAX*0.2: cnt=0
            else: inr=True
            last=d
        else: cnt=0; inr=False
        if inr: eng.append((round(t,1), round(baro,1), round(d,2), q))
    weak=[e for e in eng if e[3]<50]
    print('%-22s in-range samples %3d, with weak (garbage-like) signal %3d, first: %s' % (name, len(eng), len(weak), eng[0] if eng else None))
    if weak: print('     weak examples:', weak[:5])
