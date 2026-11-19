#!/usr/bin/env python3
"""SITL: baro thrust compensation (BARO1_THST_SCALE) on a fixed-wing, on the ground.

The aircraft is held still (no thrust from output 3) while the throttle output is
stepped, disarmed and armed. BARO.Alt is compared with the expected shift. Two passes:
  pass 1: SIM_BARO_THST 0, BARO1_THST_SCALE 50: the compensation alone. The baro
          altitude should rise by 50 Pa x throttle (about +4.4 m at full throttle at
          the SITL field's 584 m), and not move while disarmed.
  pass 2: SIM_BARO_THST 50, BARO1_THST_SCALE 50: error and compensation cancel.

usage: baro_thrust_test.py <outdir> [inst=N] [bin=...]
"""
import glob
import os
import subprocess
import sys
import time

import numpy as np
from pymavlink import mavutil

opt = dict(a.split('=') for a in sys.argv[2:])
BIN = os.path.abspath(os.path.expanduser(opt.get('bin', os.environ.get('ARDUPLANE_SITL', 'build/sitl/bin/arduplane'))))
DEFAULTS = os.path.abspath(os.path.join(os.environ.get('ARDUPILOT_ROOT', '.'), 'Tools/autotest/models/plane.parm'))
outdir = sys.argv[1]
os.makedirs(outdir, exist_ok=True)
os.chdir(outdir)
inst = int(opt.get('inst', 0))
with open('extra.parm', 'w') as f:
    f.write('LOG_DISARMED 1\nRC_OVERRIDE_TIME -1\nSIM_ENGINE_MUL 0\nSIM_ENGINE_FAIL 4\n'
            'BARO1_THST_SCALE 50\nSIM_BARO_RND 0.05\n')
proc = subprocess.Popen([BIN, '-S', '--model', 'plane', '--speedup', '4', '--defaults', DEFAULTS + ',extra.parm',
                         '-I%u' % inst, '--wipe'], stdout=open('sitl_stdout.txt', 'w'), stderr=subprocess.STDOUT)
time.sleep(3)
m = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % (5760 + 10 * inst), source_system=255, source_component=190)
m.wait_heartbeat(timeout=60)
rc = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
last_rc = [0.0]


def pump(seconds, cond=None):
    end = time.time() + seconds
    while time.time() < end:
        if time.time() - last_rc[0] > 0.05:
            m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
            last_rc[0] = time.time()
        msg = m.recv_match(blocking=True, timeout=0.05)
        if msg is not None and cond and cond(msg):
            return True
    return False


def setp(name, val):
    for _ in range(5):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        if pump(1.0, lambda x: x.get_type() == 'PARAM_VALUE' and x.param_id == name and abs(x.param_value - val) < 1e-3):
            return
    raise RuntimeError('param set failed ' + name)


def arm(state):
    for _ in range(60):
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1 if state else 0, 0 if state else 21196, 0, 0, 0, 0, 0)
        if pump(1.0, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == 1 and
                bool(x.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == state):
            return
    raise RuntimeError('arm/disarm failed')


m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
pump(40, lambda x: x.get_type() == 'STATUSTEXT' and 'is using GPS' in x.text)
pump(10)
m.set_mode(m.mode_mapping()['MANUAL'])
pump(2)
HOLD = 3.0    # wall seconds per step (12 s sim at 4x)
for sim_thst in (0, 50):
    setp('SIM_BARO_THST', sim_thst)
    for stick in (1000, 2000, 1000):
        rc[2] = stick
        pump(HOLD)
    arm(True)
    for pct in (0, 50, 100, 0):
        rc[2] = 1000 + 10 * pct
        pump(HOLD)
    arm(False)
    pump(HOLD)
proc.terminate()
time.sleep(2)

# analysis from the dataflash log: split into steps where pass (SIM_BARO_THST),
# armed state, RC throttle stick and throttle output are constant
log = sorted(glob.glob('logs/*.BIN'), key=os.path.getmtime)[-1]
lm = mavutil.mavlink_connection(log)
ev = []
while True:
    x = lm.recv_match(type=['BARO', 'RCOU', 'RCIN', 'ARM', 'PARM'])
    if x is None:
        break
    t = x.get_type()
    if t == 'PARM' and x.Name != 'SIM_BARO_THST':
        continue
    ev.append(x)
state = dict(thst=None, armed=0, out=1000, stick=1000)
segs = []
for x in ev:
    t = x.get_type()
    if t == 'PARM':
        state['thst'] = x.Value
    elif t == 'ARM':
        state['armed'] = x.ArmState
    elif t == 'RCOU':
        state['out'] = x.C3
    elif t == 'RCIN':
        state['stick'] = x.C3
    elif t == 'BARO' and x.I == 0 and state['thst'] is not None:
        key = (state['thst'], state['armed'], state['stick'], state['out'])
        if not segs or segs[-1][0] != key:
            segs.append((key, []))
        segs[-1][1].append(x.Alt)
p_field = 94500.0
rho = p_field / (287.05 * (288.15 - 0.0065 * 584))
print('expected change at full throttle: %+.2f m (50 Pa, air density %.3f)' % (50 / (rho * 9.80665), rho))
ref = {}
for (thst, armed, stick, out), alts in segs:
    if len(alts) < 15:
        continue
    alt = float(np.median(alts[len(alts) // 3:]))
    ref.setdefault(thst, alt)
    print('SIM_BARO_THST %2.0f  %-8s stick %3u%%  throttle out %3u%%  BARO.Alt %+6.2f m  change %+5.2f m'
          % (thst, 'armed' if armed else 'disarmed', (stick - 1000) / 10, (out - 1000) / 10, alt, alt - ref[thst]))
