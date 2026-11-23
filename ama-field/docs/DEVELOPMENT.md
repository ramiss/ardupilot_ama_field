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

**Or build exactly as the release CI does**, from Windows or Linux, with the VS Code
task **AMA field: Build locally** (*Terminal → Run Task*). It asks for the boards,
default `FlyingRCF4WingMini`. You can also run
`python ama-field/ci/build_local.py [BOARD ...]`. It builds the working copy,
uncommitted changes included. It uses the CI's Docker image
(`ardupilot/ardupilot-dev-chibios:v0.1.3`) and `ama-field/ci/build_boards.sh`, so the
overlays, trim tiers and file names are the same as a release. On Windows it runs in
WSL, and Docker is needed in WSL or on Linux.

The build happens in a Linux clone, `~/ama-field-build`, which keeps the submodules
and the compiler cache. The first run clones it and builds everything, which takes
a while; after that, only what changed is rebuilt. The results go to
`ama-field-out/` in the repo root, as in CI:
`files/ArduPlane-4.7.2_AMAFieldMode_local-<sha>[-dirty]_<board>.apj` and
`_with_bl.hex`, plus `results/`, `features/` and the build logs in `logs/`.

On 1 MB boards, check the build fits. The custom code is small, but ArduPilot's
1 MB targets have little spare flash.

Release binaries are built by CI for the boards in `ama-field/ci/boards.txt` (see
[Release CI](#release-ci)), with ArduPilot's standard 10-2020-q4-major toolchain.
Releases up to v16 were built by hand for the **FlyingRC F4Wing Mini** only, with
the xPack `arm-none-eabi-gcc` 10.3.1-2.3 toolchain; either toolchain works. Flash
use on that board is in the version history below. The tightest board in
`boards.txt`, the JHEMCU F405 Wing, fitted at stock with 484 bytes free in v17.

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

### Trimmed builds for 1 MB boards

Every board is built from its own hwdef first ("stock"). A 1 MB board that doesn't
fit, and isn't listed in `no_trim.txt`, is rebuilt with trim tier 1, then trim tier
2, through `./waf configure --board <B> --extra-hwdef=...`, stopping at the first
that fits. The board's own hwdef files are never edited. 2 MB boards are never
trimmed.

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
- **If tier 2 still doesn't fit,** AMA field mode itself
  doesn't fit. The board is "not available", and the summary says how many bytes it
  was over.
- **Editing the tiers:** the overlays are generated from `TRIM_TIERS` in
  [`release_tools.py`](../ci/release_tools.py). Change the feature labels there
  and run `python3 ama-field/ci/release_tools.py overlays`.
  - Labels are feature labels from `Tools/scripts/build_options.py`. Everything
    that depends on a label is turned off with it.
  - The first CI job fails if the committed `.hwdef` files are out of date with
    this tree.
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
    an EFI unit or a generator.
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
  tier 2), and flash used and free (the `.apj`
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
| `does not fit even with tier 2: N bytes over` | A 1 MB board that is still N bytes too big after every step: AMA field mode itself doesn't fit. Making it fit needs a further tier, which is a deliberate decision, not a CI fix. |
| `does not fit: N bytes over (not trimmed: ...)` | A board that is never trimmed (more than 1 MB, or in `no_trim.txt`) no longer fits. |
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

# full circuit: takeoff, runway circle, AUTOLAND in a 6 m/s crosswind, with the
# even circle descent and the gentle final
python3 ama-field/sitl/autoland_circuit_test.py /tmp/run1 wind=6 wdir=83 l1=13 lvl=10 \
    wpd=85 wpalt=20 gap=5 arc=180 fnl=8 rog=3 ls=0.6 fa=2 fs=2 alt=22 tdo=25 rfprof=mt06
python3 ama-field/sitl/analyze_final.py /tmp/run1      # circle size, overshoot, touchdown
python3 ama-field/sitl/circuit_profile.py /tmp/run1    # height around the circuit
python3 ama-field/sitl/analyze_loiter.py /tmp/run1     # runway-circle geometry

# one takeoff per arming
python3 ama-field/sitl/autoland_circuit_test.py /tmp/run2 block=1 arc=180 fnl=8

# pits detection, arming block, fence bounce-back in FBWB
python3 ama-field/sitl/pits_fence_test.py /tmp/run3 fallback FBWB
```

- The SITL aircraft is ArduPilot's stock plane model. Try your own aircraft's speeds
  and tuning with the script options:
  - `wpd` / `wpalt` / `gap` / `tdo` / `arc` / `fnl` / `rog`: `AUTOLAND_*`
  - `pmin`: `TECS_PITCH_MIN`
  - `l1`: `NAVL1_PERIOD`
  - `ls`: `TECS_LAND_SINK`
  - `fa` / `fs`: `LAND_FLARE_ALT` / `_SEC`
  - `wind` / `wdir`: wind
  - `alt`: AUTOLAND selection height
  - `rf=100 rfl=1`: simulated rangefinder with `RNGFND_LANDING` 1; `rfmax=3` for a
    small ToF sensor's range, `rfs=2.8` for a strong signal below 2.8 m
    (`SIM_SONAR_STRONG`), `glitch=0.05` for random 0.5-5 m spikes
  - `rfprof=mt06|tfluna|sonar|lidar|highmount`: a simulated sensor profile, which
    sets `RNGFND1_MIN`/`_MAX`/`_GNDCLR`, `RNGFND_LANDING` 1 and the test-only
    `SIM_SONAR_HZ` (sample rate), `SIM_SONAR_STRONG`, `SIM_SONAR_GARB` (random
    "valid" readings beyond the real range, like the MT-06) and `SIM_SONAR_POS_Z`
    (mounting height). `drop=0.1` (`SIM_SONAR_DROP`) for samples with no data,
    `rffail=eng` for a sensor that dies 0.5 s after it engages (`SIM_SONAR_FAIL`)
  - `python3 ama-field/sitl/rf_matrix.py RUNDIR ...` tabulates rangefinder runs
    against the simulator's truth: engagement height, flare source, touchdown
    detection tier and delay, touchdown point, and landing-height steps
  - `wcf`: simulated baro error from airspeed (`SIM_BARO_WCF_FWD`, stock SITL)
  - `slew`: `TKOFF_THR_SLEW`
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
| `baro_err.py LOG.BIN ...` | Baro error against strong rangefinder readings, by flight phase. See [TUNING.md](TUNING.md#41-the-barometer-must-read-true-height). |
| `rf_gate_sim3.py LOG.BIN t0 t1` | Replays the rangefinder plausibility gate on a log (quality from `RFSS.Q`, `RNGFND1_MAX` from the log). |
| `rf_engage_replay.py [--scan] LOG.BIN ...` | Rangefinder engagement on each landing from the raw `RFSS` samples: the stock rule, v17's 3 strong samples and the AUTOLAND-final consistency rule, with a check that each engagement is sustained. `--scan` stress-tests the rules over the whole flight. Sensor agnostic (`RFSS.Q`, or MT-06 strength / 255 in older logs). |
| `params_of.py LOG.BIN NAME ...` | Parameter values recorded in a log. |

These log messages are the most useful:

- `CTUN`, `NTUN`, `TECS`: height and navigation demands vs actual;
- `POS`, `GPS`, `ATT`;
- `RCIN` / `RCOU`;
- `PIDS`: ground steering;
- `RFND`, and `RFSS` for raw rangefinder samples (TOFSense-F and Benewake I2C):
  type, distance as reported (also for invalid samples), raw strength, sample status
  and the 0-100% quality ArduPilot gets. Every valid sample below `RNGFND1_MAX`,
  10 Hz otherwise;
- `ALRF` (10 Hz on an AUTOLAND final): rangefinder distance, status, quality,
  height, in range, consistent samples, hold state, correction, the landing height
  and what the stock correction rule would give;
- `TDET`: touchdown detection (tier, delay, landing height, vertical speed, last
  good rangefinder reading);
- `TDLN`: the touchdown report on each landing (touchdown along and across the
  runway line, float from the aim point, impact, the `AUTOLAND_TD_OFS` it was aimed
  with, accelerometer or rangefinder, pilot input on the final, result). Logged
  whatever `LOG_BITMASK` is;
- `MSG`: every field-mode message is logged.

`RFSS` is a streaming message, so with MAVLink logging it is limited to
`LOG_MAV_RATEMAX` (10 Hz on the example aircraft). The driver polls at 50 Hz and
logs every valid sample below `RNGFND1_MAX`, 10 Hz otherwise. To see every sample, for example on a
bench test with a hand moved under the sensor, set `LOG_MAV_RATEMAX` 0 for that
test.

---

## Code changes

| File | Change |
|---|---|
| `ArduPlane/mode_autoland.cpp`, `mode.h` | The main part. It holds:<ul><li>the pits reference;</li><li>roll start and runway direction capture, including the veer fallback;</li><li>the runway circle;</li><li>circuit side, height selection and base circle sizing;</li><li>the level circuit, the even circle descent (`circle_descent_height()`) and the glide path by distance;</li><li>touchdown aim offset (`TD_OFS`);</li><li>touchdown detection for the rollout;</li><li>the touchdown report on each landing (`TDLN` log);</li><li>the new parameters `SIDEGAP`, `TD_OFS`, `DSC_ARC`, `RO_GAIN`, `FNL_HGT`.</li></ul> |
| `libraries/AP_L1_Control/AP_L1_Control.*` | `set_period_scale()`: a multiplier (0.3–1) on `NAVL1_PERIOD` for straight-line tracking only; loiters are unchanged. AUTOLAND sets 0.6 on the final until the flare (`AUTOLAND_FINAL_L1_SCALE`) and 1 everywhere else and on leaving the mode. The base circle ends `AUTOLAND_SIDEGAP` (5 m) off the line; at the full period the test aircraft took 50–70 m of an 85–98 m final to come within 1 m of it (logs 19 and 23). In SITL the change keeps the final within about 1.6 m of the line instead of 2–3 m, on it from 15–50 m out. |
| `ArduPlane/takeoff.cpp`, `Plane.h` | `takeoff_airborne_check()`: 1.5 m above the baro height at the start of the takeoff and 5 m/s over the ground, `highest_airspeed` is raised to `TKOFF_ROTATE_SPD` / `TKOFF_TDRAG_SPD1`, so the tail hold (which otherwise cycles with its pitch check), the ground pitch and the takeoff roll limit end as they would at those speeds. Without an airspeed sensor the EKF's synthetic estimate read 4.5–7 m/s up to 39 m on the test aircraft's headwind takeoff (log 23). |
| `ArduPlane/mode_takeoff.cpp` | Pre-takeoff checks; integrator reset; one takeoff per arming; climb-out line hold; runway-circle loiter. |
| `ArduPlane/mode_rtl.cpp` | RTL to the runway circle. |
| `ArduPlane/fence.cpp` | Bounce-back to CRUISE, FBWB or FBWA. |
| `ArduPlane/Attitude.cpp` | Flare de-crab, and rollout steering gain. |
| `ArduPlane/altitude.cpp`, `Plane.h` | Rangefinder plausibility gate against baro, with a bypass for a strong signal (all modes). On an AUTOLAND final only (`rangefinder_autoland_rules()`): engagement by consistency with the EKF vertical speed, the rangefinder height measured from the wheels (`RNGFND1_GNDCLR` subtracted), the correction held below the minimum range or faded out after a failure, and the `ALRF` log. Everywhere else the stock 10-sample rule (v17's 3-strong-samples path is removed). |
| `ArduPlane/commands_logic.cpp`, `AP_Arming_Plane.cpp` | Loiter-to-alt exit for the circuit; arming block until the pits reference exists. |
| `libraries/AP_RangeFinder/AP_RangeFinder_TOFSenseF_I2C.*` | Reports signal strength as `signal_quality_pct` (strength / 255) and logs `RFSS`. Polls at 50 Hz, the MT-06's output rate. |
| `libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TFMiniPlus.*` | Reports `signal_quality_pct` from the strength (provisional mapping from the TF-Luna manual's repeatability table: 0 below 100 or from 32768, 100 × log10(strength / 100) otherwise), averages only valid samples, and logs `RFSS`. |
| `libraries/AP_RangeFinder/AP_RangeFinder_Backend.*` | `log_raw_sample()`: the shared `RFSS` log. |
| `libraries/SITL/*`, `libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp` | Test only: for the SITL rangefinder `SIM_SONAR_STRONG` (signal quality), `SIM_SONAR_GARB`, `SIM_SONAR_HZ`, `SIM_SONAR_DROP`, `SIM_SONAR_FAIL`, and `SIM_SONAR_GLITCH` spikes. |
| `libraries/AP_HAL_ChibiOS/hwdef/FlyingRCF4WingMini/`, `Tools/bootloaders/FlyingRCF4WingMini_bl.*`, `Tools/AP_Bootloader/board_types.txt` | FlyingRC F4Wing Mini board (board ID 1241). |

Nothing changes unless `AUTOLAND_OPTIONS` has bits 1 and 2 set (value 6). Without
them the firmware behaves like stock ArduPlane 4.7.2, apart from the rangefinder
gate and in-range rule, and the board-specific items.

**The circle descent** (`AUTOLAND_DSC_ARC` with `AUTOLAND_FNL_HGT`, in
`circle_descent_height()` and the LOITER exit in `navigate()`):

- The plan is by degrees of turn still to go: circuit height down to
  `AUTOLAND_FNL_HGT` at 60°, linear in angle from the start point, then level.
- Outside the LAND flight stage TECS lags its height demand by `TECS_HDEM_TCONST`,
  and on entering LAND it drops that lag ([AP_TECS.cpp](../../libraries/AP_TECS/AP_TECS.cpp),
  `_update_height_demand`). v15–v17 fed the arc plan straight in, so the aircraft
  trailed it by sink rate × 3 s, and the target stepped that far below it at the
  start of the final: 4.6–6.7 m in the logs and SITL, then a dive at up to 4.7 m/s.
  The plan is now read ahead by ground speed × `TECS_HDEM_TCONST` along the circle,
  so the lagged demand follows the plan itself.
- The descent starts when the aircraft itself (not the read-ahead target) reaches
  `AUTOLAND_DSC_ARC`, so 180 works on a half-turn arrival from downwind. At that
  moment the read-ahead target is already part way down; the 3 s lag turns that
  step into the same even sink from the start point. It only arms within 3 m of
  circuit height: selected higher, the stock loiter-to-alt spirals down to circuit
  height first (SITL from 100 m: about 2.9 m/s to 21 m, then the normal profile;
  before this, the profile armed on the first pass and the aircraft spiralled straight
  down to `AUTOLAND_FNL_HGT` and circled there). It starts from the height the
  aircraft is at, and reports a planned sink above 2.5 m/s.
- It starts only with the aircraft within 15 m (or a quarter radius) of the circle
  and turning its way: joining it is often an S-turn, during which the degrees to go
  mean nothing. Established past the start point, it starts there if 60% of the
  descent arc is left; otherwise one more lap. It never climbs back (the circle part
  of the target is latched), and an extra lap holds that height until clear of the
  exit.
- At the exit, more than 2.5 m above `AUTOLAND_FNL_HGT` flies another lap (at most
  two). The final glides from `AUTOLAND_FNL_HGT` at the exit point (usually a little
  before `land_start`) to the aim; `land_start`'s height is put on that line, so the
  stock approach-stage slope continues it.
- `AUTOLAND_FNL_HGT` 0 keeps the old continuous glide.

**The touchdown report** measures the touchdown of each AUTOLAND aimed at the roll
start, and suggests the `AUTOLAND_TD_OFS` that would have put it on the roll start
(the current value plus the distance past it). After the flare starts, the touchdown is the rangefinder tiers of
`touchdown_update()`; without a rangefinder, it is where the EKF (GPS-aided) sink rate
falls below 0.3 m/s and stays there for 1.5 s, at the first accelerometer spike
(0.25 g above 1 g) in that time if any. A bounce (EKF climb rate above 0.7 m/s, or the
rangefinder losing the ground) waits for the next contact; 2 s down confirms. It is
reported and logged (`TDLN`, with `PIn` for elevator or throttle-stick input), and
nothing is changed: v17's `AUTOLAND_TDLRN` (index 11) moved `AUTOLAND_TD_OFS` half
way to each measured float, and on the crash flights it followed the baro error from
20 to 0. Index 10 (v17's `AUTOLAND_BAROLRN`) and 11 are left unused.
`AUTOLAND_TD_OFS` defaults to 25 m (`AUTOLAND_TD_OFS_DEFAULT`), what v16 landed with
on the vented test aircraft.

### Commits on top of `Plane-4.7.2`

1. `AP_HAL_ChibiOS: add FlyingRCF4WingMini board`
2. `FlyingRCF4WingMini: enable MAVLink logging and TOFSense-F I2C rangefinder`
3. `AP_RangeFinder: TOFSense-F I2C signal quality and raw sample log`
4. `ArduPlane: rangefinder landing plausibility gate`
5. `ArduPlane: runway-circuit AUTOLAND, TAKEOFF and fence behaviour`
6. `AMA field mode: documentation, diagrams, example parameters and scripts`

v17 added the baro compensation and learning, the touchdown learning and the
rangefinder handling; v18 removes the first two again and keeps the rest:

7. `ArduPlane: sensor-agnostic rangefinder engagement and touchdown on AUTOLAND`, with the 50 Hz
   TOFSense-F polling
8. `ArduPlane: report the AUTOLAND touchdown point`
9. `ArduPlane: even circle descent and gentle final on AUTOLAND`

Commits 3–5 and 7–9 are what another board needs; 1–2 are the test aircraft's board. To move
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
| v17 (flown; crashed on the third landing) | Baro thrust compensation from the forward motor, with wind compensation, on every board; `AUTOLAND_BAROLRN` learns `BARO1_THST_SCALE` on each takeoff run-up; `AUTOLAND_TDLRN` learns `AUTOLAND_TD_OFS` from each AUTOLAND touchdown; rangefinder handling for any sensor on the AUTOLAND final (engagement by consistency with the descent at any sample rate, correction held below the minimum range, tiered touchdown detection including a blind zone, wheel-height reference); signal quality and `RFSS` for Benewake I2C (TF-Luna, if type 25 detects it); TOFSense-F polled at 50 Hz. First release built by CI, for every board in `boards.txt`. |
| v18 (SITL-tested, not yet flown) | Removes v17's baro compensation and its learning (the vented fuselage needs none; on the unvented test aircraft the compensation read 3–4 m high on the approach and the third landing flew into the ground). The touchdown point is reported, no longer learned; `AUTOLAND_TD_OFS` default 25 m. Even circle descent from `AUTOLAND_DSC_ARC` to `AUTOLAND_FNL_HGT` (new, 8 m) with TECS lag compensation, level for the last 60°, another lap if high, and a gentle final from the circle exit: no step or dive at the start of the final (SITL: peak sink below 10 m 2.0–3.0 m/s, against 4.65). Final tracks the runway line with 0.6 × `NAVL1_PERIOD` (on the centreline sooner). TAKEOFF confirms the pits side (*"Takeoff: pits 57m left, circle on right"*). Tail hold, ground pitch and takeoff roll limit released 1.5 m up, not only at the estimated airspeed (SITL, thresholds out of reach: released 0.2 s after liftoff; stock held the ground pitch to 22 m). Keeps v17's rangefinder handling and TF-Luna support. FlyingRC F4Wing Mini: 878,816 bytes used, 104,224 free. |

v9 was a ground throttle mapping for spring-centred throttle sticks, and was not
kept.
