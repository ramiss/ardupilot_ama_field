import json, math, sys
for d in sys.argv[1:]:
    r=json.load(open(d+'/result.json')); stop=r['stop']; hdg=math.radians(r['home'][3])
    tr=[p for p in r['track'] if p['phase']=='takeoff']
    t_loit=None
    for ph,txt in r['texts']:
        pass
    def xy(p):
        dn=(p['lat']-stop['lat'])*111111; de=(p['lon']-stop['lon'])*111111*math.cos(math.radians(stop['lat']))
        return dn*math.cos(hdg)+de*math.sin(hdg), -dn*math.sin(hdg)+de*math.cos(hdg)
    pts=[xy(p) for p in tr[len(tr)//3:]]  # steady loiter part
    sides=[s for a,s in pts]
    # circle fit
    import numpy as np
    A=np.array([[2*a,2*s,1] for a,s in pts]); b=np.array([a*a+s*s for a,s in pts])
    ca,cs,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=math.sqrt(c+ca*ca+cs*cs)
    print('%s: flown circle centre along=%.0f side=%.0f radius=%.0f | side range %.0f..%.0f (left/pits side negative)' % (d,ca,cs,R,min(sides),max(sides)))
    print('   msgs:', [t for _,t in r['texts'] if 'Takeoff' in t or 'Autoland' in t][:6])
