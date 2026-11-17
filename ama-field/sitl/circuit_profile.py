import json, math, sys
for d in sys.argv[1:]:
    r=json.load(open(d+'/result.json')); stop=r['stop']; hdg=math.radians(r['home'][3])
    def xy(p):
        dn=(p['lat']-stop['lat'])*111111; de=(p['lon']-stop['lon'])*111111*math.cos(math.radians(stop['lat']))
        return dn*math.cos(hdg)+de*math.sin(hdg), -dn*math.sin(hdg)+de*math.cos(hdg)
    al=[(p['t'],)+xy(p)+(p['rel'],math.hypot(p['vx'],p['vy'])) for p in r['track'] if p['phase']=='autoland' and p['armed']]
    print('==',d); last=-9
    for t,a,s,h,g in al:
        if t-last>=3 and h>0.3: last=t; print('  t=%4.0f along=%5.0f side=%5.0f alt=%5.1f gs=%4.1f' % (t,a,s,h,g))
    off=[h for t,a,s,h,g in al if abs(s)>8]
    print('  lowest alt while off the centerline (>8 m): %.1f' % min(off), '| farthest %.0f m' % max(math.hypot(a,s) for t,a,s,h,g in al))
