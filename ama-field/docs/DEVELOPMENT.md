# AMA Field Mode – development notes

Building, testing in SITL, log analysis, and where the code lives.

---

## Building

The field mode is in ArduPlane itself, so build for **your** board:

```bash
git clone --recurse-submodules -b ama-field-4.7.2 https://github.com/<you>/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y   # once; then log out and back in
./waf configure --board <YourBoard>      # e.g. MatekF405-Wing, CubeOrange, FlyingRCF4WingMini
./waf plane
# -> build/<YourBoard>/bin/arduplane.apj   (Mission Planner: Install Firmware > Load custom firmware)
```

**Re-run `./waf configure` whenever you switch boards or edit a `hwdef.dat`.** waf
only reads hwdef changes at configure time.

On 1 MB boards, check the build fits. The custom code is small, but ArduPilot's
1 MB targets have little spare flash.

The prebuilt release binary is for the **FlyingRC F4Wing Mini** (the test
aircraft's board). It was built with the xPack `arm-none-eabi-gcc` 10.3.1-2.3
toolchain. ArduPilot's standard 10-2020-q4-major works the same. Flash use is about
875 KB of 983 KB.

### FlyingRC F4Wing Mini specifics

This board has no SD card, so its hwdef enables logging with no default backend. To
stream dataflash logs over MAVLink to an external logger (an ESP32 SD logger on
SERIAL3 in testing):

- `LOG_BACKEND_TYPE` 2
- `SERIAL3_PROTOCOL` 2
- `ARMING_SKIPCHK` 1024, so a missing logger doesn't block arming

Set `LOG_BACKEND_TYPE` 0 and reboot to turn logging off. The hwdef also compiles in
the TOFSense-F I2C rangefinder driver.

---

## SITL testing

```bash
./waf configure --board sitl && ./waf plane
export ARDUPILOT_ROOT=$PWD ARDUPLANE_SITL=$PWD/build/sitl/bin/arduplane
pip install pymavlink numpy

# full circuit: takeoff, runway circle, AUTOLAND in a 6 m/s crosswind
python3 ama-field/sitl/autoland_circuit_test.py /tmp/run1 wind=6 wdir=83 l1=13 lvl=10 \
    wpd=85 wpalt=20 gap=5 tdo=25 arc=90 rog=3 ls=0.6 fa=2 fs=1 alt=22
python3 ama-field/sitl/analyze_final.py /tmp/run1      # circle size, overshoot, touchdown
python3 ama-field/sitl/circuit_profile.py /tmp/run1    # height around the circuit
python3 ama-field/sitl/analyze_loiter.py /tmp/run1     # runway-circle geometry

# one takeoff per arming
python3 ama-field/sitl/autoland_circuit_test.py /tmp/run2 block=1 tdo=25 arc=90

# pits detection, arming block, fence bounce-back in FBWB
python3 ama-field/sitl/pits_fence_test.py /tmp/run3 fallback FBWB
```

- The SITL aircraft is ArduPilot's stock plane model. Try your own aircraft's speeds
  and tuning with the script options:
  - `wpd` / `wpalt` / `gap` / `tdo` / `arc` / `rog`: `AUTOLAND_*`
  - `l1`: `NAVL1_PERIOD`
  - `ls`: `TECS_LAND_SINK`
  - `fa` / `fs`: `LAND_FLARE_ALT` / `_SEC`
  - `wind` / `wdir`: wind
  - `alt`: AUTOLAND selection height
  - `rf=100 rfl=1`: simulated rangefinder with `RNGFND_LANDING` 1
- Run several at once with `inst=0`, `inst=1`, and so on.
- The SITL runway heading is 353°. The pits are simulated by moving the first GPS fix
  60 m to the left.
- SITL doesn't model tailwheel steering. Tune the ground roll from flight logs.

---

## Log analysis scripts

All in `ama-field/log-analysis/` (Python 3 + pymavlink):

| Script | Use |
|---|---|
| `flight_review.py LOG.BIN` | Modes, messages, events, GPS and battery summary. |
| `flight_table.py LOG.BIN [step] [t0] [t1]` | Timeline table: altitude, speeds, attitude, throttle, servos, distance. |
| `landing_trace.py LOG.BIN t0 t1` | Approach and flare trace: height vs demand, distance, speed, sink, pitch. |
| `baro_err.py LOG.BIN ...` | Baro error against strong rangefinder readings. Use it to check baro shielding. |
| `rf_gate_sim3.py LOG.BIN t0 t1` | Replays the rangefinder plausibility gate on a log. |
| `params_of.py LOG.BIN NAME ...` | Parameter values recorded in a log. |

These log messages are the most useful:

- `CTUN`, `NTUN`, `TECS`: height and navigation demands vs actual;
- `POS`, `GPS`, `ATT`;
- `RCIN` / `RCOU`;
- `PIDS`: ground steering;
- `RFND`, and `RFSS` for raw TOFSense-F samples with signal strength;
- `MSG`: every field-mode message is logged.

---

## Code changes

| File | Change |
|---|---|
| `ArduPlane/mode_autoland.cpp`, `mode.h` | The main part. It holds:<ul><li>the pits reference;</li><li>roll start and runway direction capture, including the veer fallback;</li><li>the runway circle;</li><li>circuit side, height selection and base circle sizing;</li><li>the level circuit, descent arc and glide path by distance;</li><li>touchdown aim offset;</li><li>touchdown detection for the rollout;</li><li>the new parameters `SIDEGAP`, `TD_OFS`, `DSC_ARC`, `RO_GAIN`.</li></ul> |
| `ArduPlane/mode_takeoff.cpp` | Pre-takeoff checks; integrator reset; one takeoff per arming; climb-out line hold; runway-circle loiter. |
| `ArduPlane/mode_rtl.cpp` | RTL to the runway circle. |
| `ArduPlane/fence.cpp` | Bounce-back to CRUISE, FBWB or FBWA. |
| `ArduPlane/Attitude.cpp` | Flare de-crab, and rollout steering gain. |
| `ArduPlane/altitude.cpp` | Rangefinder plausibility gate against baro, with a bypass for a strong signal. |
| `ArduPlane/commands_logic.cpp`, `AP_Arming_Plane.cpp` | Loiter-to-alt exit for the circuit; arming block until the pits reference exists. |
| `libraries/AP_RangeFinder/AP_RangeFinder_TOFSenseF_I2C.*` | Reports signal strength as `signal_quality_pct`, and adds the `RFSS` log. |
| `libraries/AP_HAL_ChibiOS/hwdef/FlyingRCF4WingMini/`, `Tools/bootloaders/FlyingRCF4WingMini_bl.*`, `Tools/AP_Bootloader/board_types.txt` | FlyingRC F4Wing Mini board (board ID 1241). |

Nothing changes unless `AUTOLAND_OPTIONS` has bits 1 and 2 set (value 6). Without
them the firmware behaves like stock ArduPlane 4.7.2, apart from the rangefinder gate
and the board-specific items.

### Commits on top of `Plane-4.7.2`

1. `AP_HAL_ChibiOS: add FlyingRCF4WingMini board`
2. `FlyingRCF4WingMini: enable MAVLink logging and TOFSense-F I2C rangefinder`
3. `AP_RangeFinder: TOFSense-F I2C signal quality and raw sample log`
4. `ArduPlane: rangefinder landing plausibility gate`
5. `ArduPlane: runway-circuit AUTOLAND, TAKEOFF and fence behaviour`
6. `AMA field mode: documentation, diagrams, example parameters and scripts`

Commits 3–5 are what another board needs; 1–2 are the test aircraft's board. To move
to a newer ArduPilot release, branch from the new tag and cherry-pick the commits.
The ArduPlane changes are concentrated in `mode_autoland.cpp`, so conflicts should
stay local.

---

## Version history

These are development builds, all on ArduPlane 4.7.2 and flight-tested on the
example aircraft:

| Build | Change |
|---|---|
| v1–v3 | Land at the roll start; circuit away from the pits; runway circle; pits from the first GPS fix; arming and takeoff blocks; fence turn-back to CRUISE. |
| v4 | Hold altitude to the base circle and descend on the circle. |
| v5 | Rangefinder plausibility gate; `RFSS` logging. |
| v6 | Veer fallback to the line-up heading; climb-out line hold; wind-sized runway circle with `AUTOLAND_SIDEGAP` + 10 m. |
| v7 | No dive between the base circle and the glide slope; strong rangefinder returns bypass the baro gate. |
| v8 | Fence bounce-back also from FBWB and FBWA. |
| v10 | Base circle sized for speed and wind (no overshoot on the turn to final). |
| v11 | Glide path by distance while lining up; line-up heading sampled at idle; 60 m circle cap. |
| v12 | `AUTOLAND_TD_OFS`. |
| v13 | Level circuit at the selected height; descend on final. |
| v14 | Flare de-crab (nose aligned with the runway). |
| v15 | `AUTOLAND_DSC_ARC`. |
| v16 | Integrator reset on TAKEOFF; `AUTOLAND_RO_GAIN`; one takeoff per arming. |

v9 was a ground throttle mapping for spring-centred throttle sticks, and was not
kept.
