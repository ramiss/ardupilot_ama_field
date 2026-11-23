#!/usr/bin/env python3
"""SITL: arm on the runway (first GPS fix glitched 60 m left = "pits"), auto
TAKEOFF, loiter on the runway circle, fly away in GUIDED, then AUTOLAND.
Writes <outdir>/result.json (track + messages) for analyze_final.py /
circuit_profile.py / analyze_loiter.py.

usage: autoland_circuit_test.py <outdir> [key=val ...]
  bin=<arduplane sitl binary>   (default $ARDUPLANE_SITL or build/sitl/bin/arduplane)
  inst=N        SITL instance (run several in parallel)
  wind=m/s wdir=deg-from        simulated wind (runway heading is 353)
  l1=13 damp=0.85 lvl=10        NAVL1_PERIOD, NAVL1_DAMPING, LEVEL_ROLL_LIMIT
  wpd=85 wpalt=20 gap=5 tdo=25 arc=180 fnl=8 rog=3
                                AUTOLAND_WP_DIST/_WP_ALT/_SIDEGAP/_TD_OFS/_DSC_ARC/_FNL_HGT/_RO_GAIN
  fa=2 fs=1 ls=0.6              LAND_FLARE_ALT, LAND_FLARE_SEC, TECS_LAND_SINK
  pmin=-12                      TECS_PITCH_MIN
  alt=50 away=12                height/seconds of the GUIDED leg before AUTOLAND
                                (= the height AUTOLAND is selected at)
  rf=100 rfl=1                  SITL rangefinder, RNGFND_LANDING
  rfprof=tfluna                 rangefinder profile, sets RNGFND1_* and SIM_SONAR_* (RNGFND_LANDING 1):
                                mt06, tfluna, sonar, lidar, highmount (see RF_PROFILES)
  drop=0.1                      SIM_SONAR_DROP: rangefinder samples with no data
  rffail=eng                    the rangefinder dies 0.5 s after "Rangefinder engaged"
  rfmax=3                       RNGFND1_MAX (default 6); 3 = a small ToF sensor outdoors
  rfs=2.8                       SIM_SONAR_STRONG: rangefinder reports a strong signal
                                below 2.8 m, weak above (0/absent = no signal quality)
  glitch=0.05                   SIM_SONAR_GLITCH (with rfs: weak random 2-5 m readings)
  wcf=0.3                       SIM_BARO_WCF_FWD (baro position error at speed)
  slew=20                       TKOFF_THR_SLEW
  NAME=value                    any other parameter, in capitals (TKOFF_TDRAG_ELEV=60 ARSPD_USE=0)
  wipe=0                        keep the parameters saved by the previous run in <outdir>
  loiter=90                     stop after N s of takeoff loiter (loiter-only test)
  block=1                       after landing try TAKEOFF again (should be refused),
                                then disarm/re-arm and try again (should be accepted)
"""
import json, math, os, subprocess, sys, time
from pymavlink import mavutil

BIN = os.path.abspath(os.path.expanduser(dict(a.split('=') for a in sys.argv[2:]).get('bin', os.environ.get('ARDUPLANE_SITL', 'build/sitl/bin/arduplane'))))
DEFAULTS = os.path.abspath(os.path.join(os.environ.get('ARDUPILOT_ROOT', '.'), 'Tools/autotest/models/plane.parm'))
HOME = (-35.363261, 149.165230, 584.0, 353.0)
outdir = sys.argv[1]
opt = dict(a.split('=') for a in sys.argv[2:])
speedup = 8
os.makedirs(outdir, exist_ok=True)
os.chdir(outdir)
left = math.radians(HOME[3] - 90)
extra = os.path.join(outdir, 'extra.parm')
# simulated rangefinder profiles (rfprof=): RNGFND1_MIN/_MAX/_GNDCLR as configured,
# and how the simulated sensor behaves (SIM_SONAR_*). The SITL plane's sensor reads
# 0.01 m on its wheels at its origin: SIM_SONAR_POS_Z mounts it higher, here 0.11 m
# like the test aircraft (0.35 m for the high mount), and GNDCLR is set to that.
RF_PROFILES = {
    # name:      (min,  max, gndclr, Hz, strong (quality below this; 0 = none), garbage beyond, pos_z)
    'mt06':      (0.05, 6,   0.11,   10, 3,   1, -0.10),
    'tfluna':    (0.2,  8,   0.11,   50, 8,   0, -0.10),
    'sonar':     (0.3,  2,   0.11,   10, 0,   0, -0.10),
    'lidar':     (0.1,  40,  0.11,   20, 0,   0, -0.10),
    'highmount': (0.2,  8,   0.35,   50, 8,   0, -0.34),
}
with open(extra, 'w') as f:
    if opt.get('rfprof') in RF_PROFILES:
        rmin, rmax, clr, hz, strong, garb, posz = RF_PROFILES[opt['rfprof']]
        f.write('RNGFND1_TYPE 100\nRNGFND1_ORIENT 25\nRNGFND_LANDING 1\n'
                'RNGFND1_MIN %s\nRNGFND1_MAX %s\nRNGFND1_GNDCLR %s\n' % (rmin, rmax, clr))
        f.write('SIM_SONAR_HZ %s\nSIM_SONAR_STRONG %s\nSIM_SONAR_GARB %s\nSIM_SONAR_POS_Z %s\n'
                % (hz, strong, garb, posz))
        f.write('SIM_SONAR_GLITCH %s\nSIM_SONAR_DROP %s\nSIM_SONAR_FAIL 0\n' % (opt.get('glitch', 0), opt.get('drop', 0)))
    elif opt.get('rf'):
        f.write('RNGFND1_TYPE %s\nRNGFND1_MAX %s\nRNGFND1_MIN 0.05\nRNGFND1_ORIENT 25\nSIM_SONAR_GLITCH %s\n'
                % (opt['rf'], opt.get('rfmax', 6), opt.get('glitch', 0)))
        f.write('SIM_SONAR_STRONG %s\n' % opt.get('rfs', 0))
    if opt.get('wcf'): f.write('SIM_BARO_WCF_FWD %s\n' % opt['wcf'])
    f.write('SIM_WIND_SPD %s\nSIM_WIND_DIR %s\nSIM_WIND_TURB 0\n' % (opt.get('wind', 0), opt.get('wdir', 0)))
    # any other parameter, given in capitals: TKOFF_TDRAG_ELEV=60 ARSPD_USE=0
    for k, v in opt.items():
        if k.isupper():
            f.write('%s %s\n' % (k, v))
    # RC overrides never time out: at 8x speed a 0.4 s pause in this script would
    # otherwise hand control to the simulator's own RC input (mode switch = MANUAL)
    f.write('RC_OVERRIDE_TIME -1\n')
    pits_glitch = (60*math.cos(left)/111111.0, 60*math.sin(left)/(111111.0*math.cos(math.radians(HOME[0]))))
    f.write('SIM_GPS1_GLTCH_X %.9f\nSIM_GPS1_GLTCH_Y %.9f\n' % pits_glitch)
proc = subprocess.Popen([BIN, '-S', '--model', 'plane', '--speedup', str(speedup), '--defaults', DEFAULTS + ',' + extra,
                         '--home', ','.join(str(x) for x in HOME), '-I'+opt.get('inst','0')] + (['--wipe'] if opt.get('wipe', '1') == '1' else []),
                        stdout=open('sitl_stdout.txt', 'w'), stderr=subprocess.STDOUT)
time.sleep(3)
m = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % (5760+10*int(opt.get('inst','0'))), source_system=255, source_component=190)
m.wait_heartbeat(timeout=60)
track, texts = [], []
rc = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
last_rc = 0.0
phase = 'ground'
last_yaw = [0.0]
rf_failed = []

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
        if t == 'ATTITUDE':
            last_yaw[0] = math.degrees(msg.yaw) % 360
        if t == 'GLOBAL_POSITION_INT':
            track.append(dict(t=msg.time_boot_ms / 1000.0, lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                              rel=msg.relative_alt / 1000.0, vx=msg.vx / 100.0, vy=msg.vy / 100.0,
                              mode=m.flightmode, yaw=last_yaw[0], armed=bool(m.motors_armed()), phase=phase))
        elif t == 'STATUSTEXT':
            texts.append((phase, msg.text))
            print('  MSG:', msg.text, flush=True)
            if opt.get('rffail') == 'eng' and 'Rangefinder engaged' in msg.text and not rf_failed:
                # the sensor dies half a second (sim) after it was taken into use
                rf_failed.append(time.time() + 0.5 / speedup)
        if rf_failed and len(rf_failed) == 1 and time.time() >= rf_failed[0]:
            rf_failed.append(True)
            for _ in range(3):
                m.mav.param_set_send(m.target_system, m.target_component, b'SIM_SONAR_FAIL', 1.0,
                                     mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            texts.append((phase, 'TEST: rangefinder failed'))
            print('  TEST: rangefinder failed', flush=True)
        if cond and cond(msg):
            return True
    return False

def setp(name, val):
    for _ in range(5):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        if pump(1.0, lambda x: x.get_type() == 'PARAM_VALUE' and x.param_id == name and abs(x.param_value - val) < 1e-3):
            return
    raise RuntimeError('param set failed ' + name)

def set_mode(name):
    mid = m.mode_mapping()[name]
    for _ in range(10):
        m.set_mode(mid)
        if pump(0.5, lambda x: x.get_type() == 'HEARTBEAT' and x.custom_mode == mid and x.get_srcSystem() == 1):
            return True
    texts.append((phase, 'SET_MODE FAILED ' + name))
    return False

def offset(lat, lon, brg_deg, dist):
    b = math.radians(brg_deg)
    return (lat + dist * math.cos(b) / 111111.0, lon + dist * math.sin(b) / (111111.0 * math.cos(math.radians(lat))))

def upload_rally(lat, lon, alt):
    # mission protocol, mission_type RALLY
    for _ in range(3):
        m.mav.mission_count_send(m.target_system, m.target_component, 1, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        got = []
        def c(x):
            if x.get_type() in ('MISSION_REQUEST_INT', 'MISSION_REQUEST') and getattr(x, 'mission_type', 0) == 2:
                m.mav.mission_item_int_send(m.target_system, m.target_component, 0,
                                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                            mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT, 0, 0, 0, 0, 0, 0,
                                            int(lat * 1e7), int(lon * 1e7), alt, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if x.get_type() == 'MISSION_ACK' and getattr(x, 'mission_type', 0) == 2:
                got.append(x.type)
                return True
            return False
        if pump(5, c) and got and got[0] == 0:
            return True
    raise RuntimeError('rally upload failed')

m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
setp('AUTOLAND_OPTIONS', 6)
pump(40, lambda x: x.get_type() == 'STATUSTEXT' and 'fix point saved' in x.text)
setp('SIM_GPS1_GLTCH_X', 0); setp('SIM_GPS1_GLTCH_Y', 0)
pump(15)
params = dict(AUTOLAND_OPTIONS=int(opt.get('opts', 6)), AUTOLAND_SIDEGAP=int(opt.get('gap', 15)),
              AUTOLAND_WP_DIST=float(opt.get('wpd',100)), AUTOLAND_WP_ALT=float(opt.get('wpalt',12)), TKOFF_ALT=float(opt.get('tka',40)), TKOFF_DIST=120, WP_LOITER_RAD=30,
              RTL_ALTITUDE=60, RTL_RADIUS=0, THR_FAILSAFE=0, RTL_AUTOLAND=0, GROUND_STEER_ALT=5,
              LAND_FLARE_ALT=float(opt.get('fa',3)), LAND_FLARE_SEC=float(opt.get('fs',2)), TECS_LAND_SINK=float(opt.get('ls',0.25)), LAND_PF_ALT=6, RNGFND_LANDING=int(opt.get('rfl', 1 if opt.get('rfprof') in RF_PROFILES else 0)),
              NAVL1_DAMPING=float(opt.get('damp', 0.85)), TECS_SINK_MAX=float(opt.get('sink', 5)), NAVL1_PERIOD=float(opt.get('l1', 17)), ROLL_LIMIT_DEG=45, LEVEL_ROLL_LIMIT=float(opt.get('lvl',5)), AIRSPEED_CRUISE=float(opt.get('cruise',15)))
if opt.get('slow', '1') == '1':
    params.update(AIRSPEED_MIN=12, AIRSPEED_CRUISE=15)
if 'tdo' in opt: params['AUTOLAND_TD_OFS'] = int(opt['tdo'])
if 'slew' in opt: params['TKOFF_THR_SLEW'] = int(opt['slew'])
if 'arc' in opt: params['AUTOLAND_DSC_ARC'] = int(opt['arc'])
if 'fnl' in opt: params['AUTOLAND_FNL_HGT'] = float(opt['fnl'])
if 'pmin' in opt: params['TECS_PITCH_MIN'] = float(opt['pmin'])
if 'rog' in opt: params['AUTOLAND_RO_GAIN'] = float(opt['rog'])
if 'block' in opt: params['LAND_DISARMDELAY'] = 0
for k, v in params.items():
    setp(k, v)

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

# taxi out, stop on the "centerline"
rc[2] = 1250; pump(3.0); rc[2] = 1000
end = time.time() + 30
while time.time() < end:
    pump(0.5)
    if math.hypot(track[-1]['vx'], track[-1]['vy']) < 0.2:
        break
pump(2)
stop = track[-1].copy()
hdg = HOME[3]
# emulate arming in the pits: HOME 60m to the left of the runway line, abeam the takeoff spot
pits = offset(stop['lat'], stop['lon'], hdg - 90, 60)
rally = None
if opt.get('rally') == '1':
    # rally point 200m along the runway, 150m to the right (far side)
    a = offset(stop['lat'], stop['lon'], hdg, 200)
    rally = offset(a[0], a[1], hdg + 90, 150)
    upload_rally(rally[0], rally[1], 60)
    texts.append((phase, 'rally uploaded'))

phase = 'takeoff'
set_mode('TAKEOFF')
pump(35)      # climb + ~4 min of loiter (sim time)

if opt.get('loiter'):
    pump(float(opt['loiter']))
    json.dump(dict(opt=opt, home=HOME, stop=stop, track=track, texts=texts), open('result.json', 'w'))
    proc.terminate(); print('done', flush=True); sys.exit(0)
phase = 'away'
far = offset(stop['lat'], stop['lon'], hdg, 250)
far = offset(far[0], far[1], hdg + 90, 60)
set_mode('GUIDED')
for _ in range(3):
    m.mav.mission_item_int_send(m.target_system, m.target_component, 0,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0,
                                int(far[0] * 1e7), int(far[1] * 1e7), float(opt.get('alt', 50)))
    pump(0.5)
pump(float(opt.get('away', 12)))

phase = 'autoland'
ok = set_mode('AUTOLAND')
if ok:
    if opt.get('block'):
        pump(200, lambda x: x.get_type() == 'STATUSTEXT' and 'Hit ground' in x.text)
        pump(15)
        phase = 'retakeoff'
        r1 = set_mode('TAKEOFF')
        print('TAKEOFF after landing, still armed -> accepted:', r1, flush=True)
        for _ in range(10):
            m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
            if pump(1.0, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == 1 and not (x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)):
                break
        set_mode('MANUAL')
        for _ in range(30):
            m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            if pump(1.0, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == 1 and (x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)):
                break
        r2 = set_mode('TAKEOFF')
        print('TAKEOFF after disarm + re-arm -> accepted:', r2, flush=True)
    else:
        pump(240, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == 1 and
             not (x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        pump(3)
json.dump(dict(opt=opt, home=HOME, stop=stop, pits=pits, rally=rally, track=track, texts=texts,
               autoland_engaged=ok), open('result.json', 'w'))
# saved parameters are reused with wipe=0: put the "pits" GPS glitch back for the next boot
setp('SIM_GPS1_GLTCH_X', pits_glitch[0]); setp('SIM_GPS1_GLTCH_Y', pits_glitch[1])
pump(2)
proc.terminate()
print('done', flush=True)
