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

Release binaries are built by CI for the boards in `ama-field/ci/boards.txt` (see
[Release CI](#release-ci)), with ArduPilot's standard 10-2020-q4-major toolchain.
Releases up to v16 were built by hand for the **FlyingRC F4Wing Mini** only, with
the xPack `arm-none-eabi-gcc` 10.3.1-2.3 toolchain; either toolchain works. On
that board v17 uses 875,280 of 983,040 bytes (CI toolchain), 2,784 bytes more than
v16 for the baro compensation, takeoff learning and rangefinder changes.

**Baro compensation in your own build.** The field mode expects baro thrust and
wind compensation (`AP_BARO_THST_COMP_ENABLED`, `HAL_BARO_WIND_COMP_ENABLED`).
Thrust compensation is off by default on every board, and most 1 MB boards turn
wind compensation off. The release builds add them with
[`ama-field/ci/field-mode.hwdef`](../ci/field-mode.hwdef); do the same:

```bash
./waf configure --board <YourBoard> --extra-hwdef=ama-field/ci/field-mode.hwdef
```

Without them the build has no baro compensation and its landings rely on the learned
`AUTOLAND_TD_OFS` alone (see [TUNING.md](TUNING.md#44-boards-without-baro-compensation)).
The FlyingRC F4Wing Mini and SITL hwdefs already have both.

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

## Release CI

[`.github/workflows/ama_field_release.yml`](../../.github/workflows/ama_field_release.yml)
builds ArduPlane for every board in
[`ama-field/ci/boards.txt`](../ci/boards.txt) and publishes the files on a GitHub
Release. It runs in ArduPilot's own build container
(`ardupilot/ardupilot-dev-chibios:v0.1.3`), five boards per job, with ccache.

| Trigger | Result |
|---|---|
| Push a tag `ama-field-v<N>` | Builds every board, then creates or updates the release for that tag. |
| *Actions → AMA field release → Run workflow* | Test build. Files go to the run's artifacts only; no release is made. Optionally enter a board list, for example `FlyingRCF4WingMini MatekF405-Wing`. |

### Adding a board

1. Check that the name is a folder under `libraries/AP_HAL_ChibiOS/hwdef/`. Use
   the plain flight-controller target, not a `-bdshot`, `-periph`, `-ODID` or
   `-PPPGW` variant.
2. Add it on its own line in `ama-field/ci/boards.txt`.
3. If its first flash needs anything other than "hold BOOT, plug in USB, flash
   `_with_bl.hex`" (see the board's `README.md` in its hwdef folder), add a line to
   [`ama-field/ci/board_notes.txt`](../ci/board_notes.txt).
4. Do a test build of just that board with *Run workflow*.

No workflow edit is needed. An unknown board name stops the run at the first job,
before anything is built.

### Baro compensation and trimmed builds for 1 MB boards

Every build gets [`field-mode.hwdef`](../ci/field-mode.hwdef), which turns on baro
thrust and wind compensation, through `./waf configure --board <B> --extra-hwdef=...`.
The board's own hwdef files are never edited. Per board, the first build that fits
is shipped:

1. the board's own hwdef ("stock") + `field-mode.hwdef`;
2. trim tier 1 + `field-mode.hwdef`;
3. trim tier 2 + `field-mode.hwdef`;
4. trim tier 2 alone. This build has no baro compensation; the learned
   `AUTOLAND_TD_OFS` takes up the larger float.

Steps 2–4 are only for a 1 MB board that isn't in `no_trim.txt`. Any other board
tries its own hwdef without `field-mode.hwdef` as step 2. `extract_features.py` on
every shipped binary decides the "Baro comp" column (yes = both compensations
compiled in). The trim tiers and `field-mode.hwdef` are combined into one overlay at
build time, because `--extra-hwdef` takes one file. On the test boards the
compensation, with `AUTOLAND_BAROLRN`, costs 2,976 bytes in a tier 2 build.

A ChibiOS build that is too big fails to link with "cannot move location counter
backwards (from <end of code> to <end of flash>)", from the crash log section at the
end of flash, rather than "region overflowed by". The script reads the overflow
from either message.

| Tier | Turns off |
|---|---|
| 1 | Features AMA field mode can't use: QuadPlane/VTOL (including ship landing and VTOL quicktune), deepstall landing, soaring, internal-combustion engine and EFI, parachute. |
| 2 | Tier 1, plus peripherals a novice field aircraft rarely has: ADSB, camera, mount/gimbal, gripper, generator, proximity and avoidance, beacon, visual odometry, terrain database and terrain following. |

- **Never trimmed:**
  - boards with more than 1 MB of flash;
  - boards listed in [`ama-field/ci/no_trim.txt`](../ci/no_trim.txt). That
    includes the FlyingRC F4Wing Mini, which must match the flight-tested build.
    Its hwdef already has both baro compensations, so `field-mode.hwdef` changes
    nothing on it. Compared with v16 its feature list differs only by those two.
- **If tier 2 without baro compensation still doesn't fit,** AMA field mode itself
  doesn't fit. The board is "not available", and the summary says how many bytes it
  was over.
- **Editing the tiers:** the overlays are generated from `TRIM_TIERS` in
  [`release_tools.py`](../ci/release_tools.py). Change the feature labels there
  and run `python3 ama-field/ci/release_tools.py overlays`.
  - Labels are feature labels from `Tools/scripts/build_options.py`. Everything
    that depends on a label is turned off with it.
  - The first CI job fails if the committed `.hwdef` files are out of date with
    this tree, if a tier names a baro compensation define, or if
    `field-mode.hwdef` turns on anything other than the two baro compensations.
- **Safety checks:**
  - `overlays` refuses to name anything that AMA field mode or a basic setup
    needs: AUTOLAND, fence, GPS, compass, baro, rangefinder, airspeed, logging,
    OSD, battery monitoring, RC protocols, MAVLink, bootloader flashing.
  - After a trimmed build, `probe` reads each feature's effective value from the
    board's real compile command, for the stock and the trimmed configuration. A
    board fails if a protected feature went from on to off.
  - The features that did go from on to off are the board's "removed" list in
    the release notes.
  - The only allowed exceptions are the battery-monitor backends that read from
    an EFI unit or a generator, and the baro compensation in step 4. That shows in
    the "Baro comp" column, not in the removed list.
- **No trim needed for core modes:** TAKEOFF, FBWA, FBWB, CRUISE, LOITER, RTL,
  MANUAL, GUIDED and Plane autotune aren't optional at compile time in 4.7.2, so
  no overlay can remove them.

### Cutting a release

1. Merge the changes into `ama-field-4.7.2`, and update the version history below.
2. Optionally, do a full test build with *Run workflow* and an empty board list.
3. Run the VS Code task **AMA field: Publish all builds** (*Terminal → Run Task*),
   or `python ama-field/ci/publish_release.py`. It checks the working tree is clean
   and `ama-field-4.7.2` is pushed, asks for the tag (it suggests the next
   `ama-field-v<N>`; `17` or `v17` is enough) and a one-line message, shows a
   summary and, once you confirm, creates and pushes the tag. `--dry-run` goes
   through everything without tagging. By hand, the same is:

   ```bash
   git tag -a ama-field-v17 -m "AMA field mode v17"
   git push origin ama-field-v17
   ```

4. Wait for the run to finish (roughly 30–60 minutes), then check the release
   page. The script prints both links.

Files are named `ArduPlane-<fw>_AMAFieldMode_<version>_<Board>.apj` and
`..._with_bl.hex`, where `<fw>` comes from `ArduPlane/version.h` and `<version>` is
the tag without `ama-field-`. `SHA256SUMS.txt` covers every file.

To redo a release, move the tag and push it again: run the task with the same tag
and confirm moving it, or by hand
(`git tag -f -a ama-field-v17 ...` then `git push -f origin ama-field-v17`). The
new run cancels any run still going for that tag, then replaces the release notes
and files.

### Reading the summary

Open the run in the *Actions* tab. The summary page shows:

- one table per batch (`build b01`, `b02`, ...);
- a combined table with every board, its status, its build (stock, tier 1 or
  tier 2), whether it has baro compensation, and flash used and free (the `.apj`
  image size and free flash, the same numbers Mission Planner reports);
- a collapsible list of the removed features for every trimmed board.

The same table and lists go into the release notes. Each batch artifact also
has `features/<Board>.txt` (`Tools/scripts/extract_features.py` on the shipped
build), and `probe/` with the stock and trimmed feature values.

A board that fails is listed as **failed** with a reason, and as "not available"
in the release. The release still goes out with every other board. Usual
reasons:

| Reason | Meaning |
|---|---|
| `does not fit even with tier 2 and no baro compensation: N bytes over` | A 1 MB board that is still N bytes too big after every step: AMA field mode itself doesn't fit. Making it fit needs a further tier, which is a deliberate decision, not a CI fix. |
| `does not fit: N bytes over, M without baro compensation (not trimmed: ...)` | A board that is never trimmed (more than 1 MB, or in `no_trim.txt`) no longer fits, even without `field-mode.hwdef`. |
| `tierN fits but the trim check failed` | The trimmed build would remove a protected feature, so it isn't shipped. Read `probe/<Board>.<step>.diff` in the batch artifact. |
| `build error` | Compile or configure error. Download the batch artifact (`ama-field-b0N`) and read `logs/<Board>.log`. |
| `build job did not finish` | The whole batch job failed or timed out. Look at that job's log. |

The run fails, and no release is made, only if no board built at all.

---

## SITL testing

```bash
./waf configure --board sitl && ./waf plane
export ARDUPILOT_ROOT=$PWD ARDUPLANE_SITL=$PWD/build/sitl/bin/arduplane
pip install pymavlink numpy

# full circuit: takeoff, runway circle, AUTOLAND in a 6 m/s crosswind
python3 ama-field/sitl/autoland_circuit_test.py /tmp/run1 wind=6 wdir=83 l1=13 lvl=10 \
    wpd=85 wpalt=20 gap=5 arc=90 rog=3 ls=0.6 fa=2 fs=1 alt=22
python3 ama-field/sitl/analyze_final.py /tmp/run1      # circle size, overshoot, touchdown
python3 ama-field/sitl/circuit_profile.py /tmp/run1    # height around the circuit
python3 ama-field/sitl/analyze_loiter.py /tmp/run1     # runway-circle geometry

# baro error and compensation: simulated prop-wash and airspeed error, starting values,
# takeoff learning on a held run-up; then the baro checks from the dataflash log
python3 ama-field/sitl/autoland_circuit_test.py /tmp/run4 thst=50 wcf=0.15 bts=0 bwcf=0.15 \
    lrn=1 slew=20 hold=3 wind=6 wdir=83 l1=13 lvl=10 wpd=85 wpalt=20 gap=5 arc=90 rog=3 \
    ls=0.6 fa=2 fs=1 alt=22
python3 ama-field/sitl/baro_check.py /tmp/run4          # BTLN, run-up, airborne error
# repeat with wipe=0 (same directory) to watch BARO1_THST_SCALE converge

# thrust compensation on the ground: altitude shift with throttle, nothing disarmed
python3 ama-field/sitl/baro_thrust_test.py /tmp/run5

# one takeoff per arming
python3 ama-field/sitl/autoland_circuit_test.py /tmp/run2 block=1 arc=90

# pits detection, arming block, fence bounce-back in FBWB
python3 ama-field/sitl/pits_fence_test.py /tmp/run3 fallback FBWB
```

The SITL build has both baro compensations (`libraries/AP_HAL_SITL/hwdef/sitl/hwdef.dat`).
Every run learns `AUTOLAND_TD_OFS` (`tdl=0` turns it off, `tdo=` sets a start value);
repeat runs with `wipe=0` to watch it converge. To test a board without baro
compensation, build with both off:

```bash
printf 'undef AP_BARO_THST_COMP_ENABLED\ndefine AP_BARO_THST_COMP_ENABLED 0\nundef HAL_BARO_WIND_COMP_ENABLED\ndefine HAL_BARO_WIND_COMP_ENABLED 0\n' > /tmp/nocomp.hwdef
./waf configure --board sitl --extra-hwdef=/tmp/nocomp.hwdef && ./waf plane
```

- The SITL aircraft is ArduPilot's stock plane model. Try your own aircraft's speeds
  and tuning with the script options:
  - `wpd` / `wpalt` / `gap` / `tdo` / `tdl` / `arc` / `rog`: `AUTOLAND_*`
  - `l1`: `NAVL1_PERIOD`
  - `ls`: `TECS_LAND_SINK`
  - `fa` / `fs`: `LAND_FLARE_ALT` / `_SEC`
  - `wind` / `wdir`: wind
  - `alt`: AUTOLAND selection height
  - `rf=100 rfl=1`: simulated rangefinder with `RNGFND_LANDING` 1; `rfmax=3` for a
    small ToF sensor's range, `rfs=2.8` for a strong signal below 2.8 m
    (`SIM_SONAR_STRONG`), `glitch=0.05` for weak random readings
  - `thst` / `wcf`: simulated baro error from throttle (`SIM_BARO_THST`, Pa at full
    throttle) and airspeed (`SIM_BARO_WCF_FWD`)
  - `bts` / `bwcf` / `lrn`: `BARO1_THST_SCALE`, `BARO1_WCF_FWD`, `AUTOLAND_BAROLRN`
  - `slew`: `TKOFF_THR_SLEW`; `hold=3`: hold the aircraft still for the first 3 s
    of TAKEOFF, because the SITL plane has no static friction and rolls at about
    10% throttle, too early for the baro learning
  - `wipe=0`: keep the parameters saved by the previous run in the same directory
- Run several at once with `inst=0`, `inst=1`, and so on.
- The test sets `RC_OVERRIDE_TIME` -1. At 8× speed, a pause of under half a second
  in the script would otherwise hand control to the simulator's own RC input,
  which has the mode switch in MANUAL.
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
| `baro_throttle.py [--replay] [--models] LOG.BIN ...` | Baro error from throttle and airspeed: the takeoff run-up, near-ground flight against strong rangefinder readings, and airborne fits. Suggests `BARO1_THST_SCALE` and `BARO1_WCF_FWD` (current + increment). `--replay` runs the `AUTOLAND_BAROLRN` learning on each takeoff; `--models` compares throttle models. See [TUNING.md](TUNING.md#4-barometer-compensation). |
| `baro_err.py LOG.BIN ...` | Baro error against strong rangefinder readings, by flight phase. |
| `rf_gate_sim3.py LOG.BIN t0 t1` | Replays the rangefinder plausibility gate on a log. |
| `rf_engage_replay.py LOG.BIN ...` | Rangefinder engagement height on each landing, the old (10 samples) and new (3 strong samples) rule, from the raw `RFSS` samples, with a 50 Hz estimate. |
| `params_of.py LOG.BIN NAME ...` | Parameter values recorded in a log. |

These log messages are the most useful:

- `CTUN`, `NTUN`, `TECS`: height and navigation demands vs actual;
- `POS`, `GPS`, `ATT`;
- `RCIN` / `RCOU`;
- `PIDS`: ground steering;
- `RFND`, and `RFSS` for raw TOFSense-F samples with signal strength;
- `BTLN`: the baro learning on each takeoff (samples, top throttle, measured, old
  and new `BARO1_THST_SCALE`, baseline noise, result). It is logged whatever
  `LOG_BITMASK` is;
- `TDLN`: the touchdown offset learning on each landing (touchdown along and across
  the runway line, float, impact, old and new `AUTOLAND_TD_OFS`, accelerometer or
  rangefinder, result). Also logged whatever `LOG_BITMASK` is;
- `MSG`: every field-mode message is logged.

`RFSS` is a streaming message, so with MAVLink logging it is limited to
`LOG_MAV_RATEMAX` (10 Hz on the example aircraft). The driver polls at 50 Hz and
logs every sample below 5 m, 10 Hz above. To see every sample, for example on a
bench test with a hand moved under the sensor, set `LOG_MAV_RATEMAX` 0 for that
test.

---

## Code changes

| File | Change |
|---|---|
| `ArduPlane/mode_autoland.cpp`, `mode.h` | The main part. It holds:<ul><li>the pits reference;</li><li>roll start and runway direction capture, including the veer fallback;</li><li>the runway circle;</li><li>circuit side, height selection and base circle sizing;</li><li>the level circuit, descent arc and glide path by distance;</li><li>touchdown aim offset (`TD_OFS`);</li><li>touchdown detection for the rollout;</li><li>the baro learning on the takeoff run-up (`BAROLRN`, `BTLN` log);</li><li>the touchdown offset learning on each landing (`TDLRN`, `TDLN` log);</li><li>the new parameters `SIDEGAP`, `TD_OFS`, `DSC_ARC`, `RO_GAIN`, `BAROLRN`, `TDLRN`.</li></ul> |
| `ArduPlane/mode_takeoff.cpp` | Pre-takeoff checks; integrator reset; one takeoff per arming; climb-out line hold; runway-circle loiter; starts the baro learning. |
| `libraries/AP_Baro/AP_Baro.cpp`, `AP_Baro.h` | Thrust compensation (`BARO1_THST_SCALE`) on a plane without VTOL motors uses the forward throttle output, 0 to 1 after the throttle slew, reverse ignored, nothing while disarmed. AP_Baro reads it from `SRV_Channels`, as AP_Airspeed and the battery monitor already do, so no vehicle code has to feed it. An accessor lets ArduPlane update the parameter. |
| `libraries/AP_HAL_ChibiOS/hwdef/FlyingRCF4WingMini/hwdef.dat`, `libraries/AP_HAL_SITL/hwdef/sitl/hwdef.dat`, `ama-field/ci/field-mode.hwdef` | Turn on `AP_BARO_THST_COMP_ENABLED` and `HAL_BARO_WIND_COMP_ENABLED` (wind compensation is off in `minimize_common.inc`). |
| `ArduPlane/mode_rtl.cpp` | RTL to the runway circle. |
| `ArduPlane/fence.cpp` | Bounce-back to CRUISE, FBWB or FBWA. |
| `ArduPlane/Attitude.cpp` | Flare de-crab, and rollout steering gain. |
| `ArduPlane/altitude.cpp`, `Plane.h` | Rangefinder plausibility gate against baro, with a bypass for a strong signal. A strong signal on 3 new samples in a row sets the rangefinder in range at once; other sensors and weak samples keep the stock 10 samples. |
| `ArduPlane/commands_logic.cpp`, `AP_Arming_Plane.cpp` | Loiter-to-alt exit for the circuit; arming block until the pits reference exists. |
| `libraries/AP_RangeFinder/AP_RangeFinder_TOFSenseF_I2C.*` | Reports signal strength as `signal_quality_pct`, and adds the `RFSS` log (every sample below 5 m, 10 Hz above). Polls at 50 Hz, the MT-06's output rate. |
| `libraries/SITL/*`, `libraries/AP_Baro/AP_Baro_SITL.cpp`, `libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp` | Test only: `SIM_BARO_THST` (baro error proportional to the throttle output), `SIM_SONAR_STRONG` (signal quality for the SITL rangefinder, and weak glitches with `SIM_SONAR_GLITCH`). |
| `libraries/AP_HAL_ChibiOS/hwdef/FlyingRCF4WingMini/`, `Tools/bootloaders/FlyingRCF4WingMini_bl.*`, `Tools/AP_Bootloader/board_types.txt` | FlyingRC F4Wing Mini board (board ID 1241). |

Nothing changes unless `AUTOLAND_OPTIONS` has bits 1 and 2 set (value 6). Without
them the firmware behaves like stock ArduPlane 4.7.2, apart from the rangefinder
gate and in-range rule, the baro thrust compensation on a fixed-wing motor (only
with `BARO1_THST_SCALE` set), and the board-specific items.

**`AUTOLAND_TDLRN`** (index 11, every build) measures the touchdown of each AUTOLAND
aimed at the roll start. After the flare starts, the touchdown is the rangefinder at
its ground clearance; without one, it is where the EKF (GPS-aided) sink rate falls
below 0.3 m/s and stays there for 1.5 s, at the first accelerometer spike (0.25 g
above 1 g) in that time if any. The accelerometer alone is not enough: in SITL the
flare pull-up reaches 0.8 g above 1 g, more than a soft touchdown, but the sink
continues after it. A bounce (EKF climb rate above 0.7 m/s, or the rangefinder 0.3 m
up) waits for the next contact; 2 s down confirms. Stick input, a mode change, or
more than 15 m off the runway line rejects the landing. The float (aim point to
touchdown) moves `AUTOLAND_TD_OFS` half way toward it (as is from 0; 0–50 m) with
`set_and_notify`, and it is saved at disarm. It doesn't touch the landing in
progress: the aim point is fixed when AUTOLAND starts. The bounce check uses the EKF
climb rate rather than the baro, because the uncompensated part of the baro airspeed
error decays as the aircraft slows and would look like a climb.
`AUTOLAND_TD_OFS` defaults to `AUTOLAND_TD_OFS_DEFAULT` in `mode_autoland.cpp`: 20 m
with both baro compensations, 30 m without, from the test aircraft's measured float
(36–48 m uncompensated in logs 15–18, 25–28 m compensated in SITL), so an untrained
aircraft lands near the roll start instead of far down the runway.
`AUTOLAND_BAROLRN` (index 10) exists only with thrust compensation.

### Commits on top of `Plane-4.7.2`

1. `AP_HAL_ChibiOS: add FlyingRCF4WingMini board`
2. `FlyingRCF4WingMini: enable MAVLink logging and TOFSense-F I2C rangefinder`
3. `AP_RangeFinder: TOFSense-F I2C signal quality and raw sample log`
4. `ArduPlane: rangefinder landing plausibility gate`
5. `ArduPlane: runway-circuit AUTOLAND, TAKEOFF and fence behaviour`
6. `AMA field mode: documentation, diagrams, example parameters and scripts`

v17 adds:

7. `AP_Baro: thrust compensation from the forward motor on a fixed-wing`, with the
   hwdef changes
8. `ArduPlane: learn BARO1_THST_SCALE on the takeoff run-up`
9. `ArduPlane: learn AUTOLAND_TD_OFS from each AUTOLAND touchdown`
10. `ArduPlane: engage the rangefinder after 3 strong samples`, with the 50 Hz
    TOFSense-F polling
11. `CI: baro compensation on every board, before the trim tiers`
12. `AMA field: baro compensation docs, parameters and scripts`

Commits 3–5 and 7–10 are what another board needs; 1–2 are the test aircraft's board. To move
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
| v17 (SITL-tested, not yet flown) | Baro thrust compensation from the forward motor, with wind compensation, on every board; `AUTOLAND_BAROLRN` learns `BARO1_THST_SCALE` on each takeoff run-up; `AUTOLAND_TDLRN` learns `AUTOLAND_TD_OFS` from each AUTOLAND touchdown; rangefinder in range after 3 strong samples; TOFSense-F polled at 50 Hz. First release built by CI, for every board in `boards.txt`. |

v9 was a ground throttle mapping for spring-centred throttle sticks, and was not
kept.
