import json, math, sys
for d in sys.argv[1:]:
    r=json.load(open(d+'/result.json')); stop=r['stop']; hdg=math.radians(r['home'][3]); home=r['home']
    def xy(p):
        dn=(p['lat']-stop['lat'])*111111; de=(p['lon']-stop['lon'])*111111*math.cos(math.radians(stop['lat']))
        return dn*math.cos(hdg)+de*math.sin(hdg), -dn*math.sin(hdg)+de*math.cos(hdg)
    al=[p for p in r['track'] if p['phase']=='autoland' and p['armed']]
    texts=[t for _,t in r['texts']]
    # find exit time: first sample after which altitude < WP_ALT+3 and along > -200 heading to runway... use texts order not timed; approximate: final = points with along between -140 and 0 and alt < 16
    pts=[(xy(p),p['rel'],p['t']) for p in al]
    circ=[(a,s) for (a,s),h,t in pts]
    fin=[(a,s,h) for (a,s),h,t in pts if h<15]
    # overshoot: most negative side (pits side) during final descent
    pits_min=min(s for a,s,h in fin); circ_max=max(s for a,s in circ); back=min(a for a,s in circ)
    td=[x for x in fin if x[2]<0.4]
    hm=(home[0],home[1])
    print('%-5s base circle: back edge along %.0f, far side %.0f | final: worst pits-side %.1f m | touchdown along %.0f side %.1f | %s' % (d, back, circ_max, pits_min, td[0][0] if td else float('nan'), td[0][1] if td else float('nan'), [t for t in texts if 'Hit ground' in t or 'glide' in t]))
