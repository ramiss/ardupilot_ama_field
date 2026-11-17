import sys
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1])
P = {}
while True:
    x = m.recv_match(type=['PARM'])
    if x is None: break
    P.setdefault(x.Name, x.Value)
for k in sys.argv[2:]:
    print(k, P.get(k))
