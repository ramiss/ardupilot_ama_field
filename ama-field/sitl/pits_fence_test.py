#!/usr/bin/env python3
"""SITL: pits-side detection, arming block and fence bounce-back.

usage: pits_fence_test.py <outdir> <scenario> [MODE] [inst]
  scenario  fallback = first GPS fix 60 m left of the runway (the "pits")
            nogps    = no GPS at boot: arming must be refused
            blocked  = no pits offset: TAKEOFF must be refused
  MODE      CRUISE | FBWB | FBWA  mode to fly into the 600 m test fence
"""
import json, math, os, subprocess, sys, time
from pymavlink import mavutil

BIN = os.path.abspath(os.path.expanduser(os.environ.get('ARDUPLANE_SITL', 'build/sitl/bin/arduplane')))
DEFAULTS = os.path.abspath(os.path.join(os.environ.get('ARDUPILOT_ROOT', '.'), 'Tools/autotest/models/plane.parm'))
HOME = (-35.363261, 149.165230, 584.0, 353.0)
outdir, scenario = sys.argv[1], sys.argv[2]
FLYMODE = sys.argv[3] if len(sys.argv) > 3 else 'CRUISE'
INST = int(sys.argv[4]) if len(sys.argv) > 4 else 0
os.makedirs(outdir, exist_ok=True)
os.chdir(outdir)

# GPS glitch at boot = first good fix appears 60m to the LEFT of the runway (the "pits")
left = math.radians(HOME[3] - 90)
dlat = 60 * math.cos(left) / 111111.0
dlon = 60 * math.sin(left) / (111111.0 * math.cos(math.radians(HOME[0])))
extra = os.path.join(outdir, 'extra.parm')
with open(extra, 'w') as f:
    if scenario in ('fallback', 'nogps'):
        f.write('SIM_GPS1_GLTCH_X %.9f\nSIM_GPS1_GLTCH_Y %.9f\n' % (dlat, dlon))
    if scenario == 'nogps':
        f.write('SIM_GPS1_TYPE 0\n')
proc = subprocess.Popen([BIN, '-S', '--model', 'plane', '--speedup', '8', '--defaults', DEFAULTS + ',' + extra,
                         '--home', ','.join(str(x) for x in HOME), '-I%d' % INST, '--wipe'],
                        stdout=open('sitl_stdout.txt', 'w'), stderr=subprocess.STDOUT)
time.sleep(3)
m = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % (5760+10*INST), source_system=255, source_component=190)
m.wait_heartbeat(timeout=60)
track, texts = [], []
rc = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
last_rc = 0.0
phase = 'boot'

def pump(seconds, cond=None):
    global last_rc
    end = time.time() + seconds
    while time.time() < end:
        if time.time() - last_rc > 0.05:
            m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
            last_rc = time.time()
        msg = m.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'GLOBAL_POSITION_INT':
            track.append(dict(t=msg.time_boot_ms / 1000.0, lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                              rel=msg.relative_alt / 1000.0, vx=msg.vx / 100.0, vy=msg.vy / 100.0,
                              mode=m.flightmode, armed=bool(m.motors_armed()), phase=phase))
        elif t == 'STATUSTEXT':
            texts.append((phase, msg.severity, msg.text))
            if 'PreArm' not in msg.text:
                print('  MSG[%d]: %s' % (msg.severity, msg.text), flush=True)
        if cond and cond(msg):
            return True
    return False

def setp(name, val):
    for _ in range(5):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        if pump(1.0, lambda x: x.get_type() == 'PARAM_VALUE' and x.param_id == name and abs(x.param_value - val) < 1e-6):
            return
    raise RuntimeError('param set failed ' + name)

def set_mode(name, tries=10):
    mid = m.mode_mapping()[name]
    for _ in range(tries):
        m.set_mode(mid)
        if pump(0.5, lambda x: x.get_type() == 'HEARTBEAT' and x.custom_mode == mid and x.get_srcSystem() == 1):
            return True
    texts.append((phase, 0, 'SET_MODE FAILED ' + name))
    print('  set mode failed:', name, flush=True)
    return False

m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
if scenario == 'nogps':
    # no GPS yet: arming must be refused with the fix-point message
    setp('AUTOLAND_OPTIONS', 6)
    pump(10)
    for _ in range(5):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(1.5)
    refused = [t for _, _, t in texts if 'no GPS fix point' in t]
    print('ARM WITHOUT FIX POINT -> refused messages:', len(refused), refused[:1], '| armed:', m.motors_armed(), flush=True)
    setp('SIM_GPS1_TYPE', 1)
# wait for the GPS fix point to be saved (at the glitched "pits" position in the fallback scenario)
pump(40, lambda x: x.get_type() == 'STATUSTEXT' and 'fix point saved' in x.text)
if scenario in ('fallback', 'nogps'):
    setp('SIM_GPS1_GLTCH_X', 0); setp('SIM_GPS1_GLTCH_Y', 0)
    pump(15)   # let the EKF settle on the true position
params = dict(AUTOLAND_OPTIONS=6, AUTOLAND_SIDEGAP=30, AUTOLAND_WP_DIST=250, AUTOLAND_WP_ALT=25,
              TKOFF_ALT=50, TKOFF_DIST=300, WP_LOITER_RAD=30, RTL_ALTITUDE=60, THR_FAILSAFE=0, RTL_AUTOLAND=0,
              GROUND_STEER_ALT=5, LAND_FLARE_ALT=3, LAND_FLARE_SEC=2, RNGFND_LANDING=0, NAVL1_DAMPING=0.85,
              AIRSPEED_MIN=12, AIRSPEED_CRUISE=15,
              FENCE_ENABLE=1, FENCE_TYPE=2, FENCE_RADIUS=600, FENCE_ACTION=6, FENCE_RET_RALLY=0, FENCE_MARGIN=2)
for k, v in params.items():
    setp(k, v)

phase = 'ground'
set_mode('MANUAL')
for _ in range(120):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    if pump(1.0, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == 1 and
            (x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)):
        break
else:
    raise RuntimeError('arm failed')
pump(2)
arm = track[-1].copy()
# taxi 100m down the runway and stop (HOME stays at the arming point on the runway line)
rc[2] = 1220; pump(1.5); rc[2] = 1000
end = time.time() + 30
while time.time() < end:
    pump(0.5)
    if math.hypot(track[-1]['vx'], track[-1]['vy']) < 0.2:
        break
pump(2)
stop = track[-1].copy()

phase = 'takeoff'
took = set_mode('TAKEOFF', tries=4)
result = dict(scenario=scenario, arm=arm, stop=stop, fixpt_offset_m=60 if scenario == 'fallback' else 0,
              takeoff_allowed=took)
if took:
    pump(35)
    phase = 'cruise'
    rc[2] = 1600
    set_mode(FLYMODE)
    pump(75)                     # fly straight until fence breaches, turn back (watch for 2 cycles)
    phase = 'autoland'
    rc[2] = 1000
    ok = set_mode('AUTOLAND')
    if ok:
        pump(240, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == 1 and
             not (x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        pump(3)
    result['autoland_engaged'] = ok
    if ok and scenario == 'fallback':
        # re-arm on the runway where it landed (HOME moves here) and try TAKEOFF again
        phase = 'rearm'
        pump(3)
        for _ in range(60):
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            if pump(1.0, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == 1 and
                    (x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)):
                break
        result['rearm_takeoff_allowed'] = set_mode('TAKEOFF', tries=4)
        print('RE-ARMED ON RUNWAY, TAKEOFF allowed:', result['rearm_takeoff_allowed'], flush=True)
        pump(3)
result.update(track=track, texts=texts, home=HOME)
json.dump(result, open('result.json', 'w'))
proc.terminate()
print('done', flush=True)
