# AMA Field Mode – setup and tuning guide

This guide takes any runway-capable ArduPlane aircraft from "flies well in FBWA" to
"auto takeoff and one-switch landing at my field". Section 2 shows how to choose
starting values. Section 4 sets up the barometer compensation. Sections 5–8 cover
what to change for each symptom you see.

The [Freewing Pandora example](../../README.md#example-aircraft) shows one finished
setup and is quoted throughout as a reference point. It is a 1.2 m foam
taildragger cruising at 14 m/s. Expect your numbers to differ.

---

## 1. Before you start

- The aircraft is **calibrated** (accelerometers, level, compass, radio, servo
  directions, battery) and **autotuned** in FBWA.
- `AIRSPEED_CRUISE`, `AIRSPEED_MIN`, `AIRSPEED_MAX` and `TRIM_THROTTLE` are right.
  Check that FBWB holds height and speed in a straight line.
- You can fly the aircraft by hand, and land it by hand, on this runway.
- Logging works. Every step below is easier with a dataflash log of each flight
  (see [DEVELOPMENT.md](DEVELOPMENT.md#log-analysis-scripts)).
- A **pilot override** is one switch away. That means MANUAL or FBWA, not just
  another automatic mode.

Then load [`ama_field_core.param`](../params/ama_field_core.param) and choose
values as below.

---

## 2. Choosing starting values

### 2.1 Field values

These depend on your site, not the aircraft. Pace out or measure the field on a
satellite map.

| Parameter | How to choose it | Pandora example |
|---|---|---|
| `AUTOLAND_WP_ALT` | Tallest obstacle near the circuit plus at least 5 m. This is the lowest the aircraft will be anywhere except the final approach. | 20 m (15 m trees) |
| `AUTOLAND_WP_DIST` | Length of the final approach, measured back from the touchdown aim. Start at about **6–8 s at landing speed** (`TECS_LAND_ARSPD`, or about 1.2 × `AIRSPEED_MIN`). Make it longer for fast or slippery aircraft, and shorter if trees off the end of the runway force it. | 85 m at 15 m/s |
| `AUTOLAND_SIDEGAP` | Gap between the extended centreline and the base circle. Use 5 m for a small, precise aircraft, and 10–15 m if the turn to final overshoots or the aircraft is fast. | 5 m |
| `TKOFF_ALT` | Height of the runway circle after takeoff. It must be well above `AUTOLAND_WP_ALT`. | 50 m |
| `CRUISE_ALT_FLOOR` | Floor height for FBWB and CRUISE. It is usually the same as `AUTOLAND_WP_ALT`. | 20 m |
| `FENCE_RADIUS` | Large enough that the whole circuit fits inside with margin (see 2.3), and small enough to keep a novice close. | 240 m |
| `FENCE_ALT_MAX` | Club or AMA ceiling, with margin. | 100 m |

### 2.2 Aircraft values

| Parameter | Starting value | How to refine it |
|---|---|---|
| `BARO1_THST_SCALE` | **0**: the first auto takeoff measures it (`AUTOLAND_BAROLRN` 1) | Learned again on every auto takeoff. See 4.1. |
| `BARO1_WCF_ENABLE` / `BARO1_WCF_FWD` | **1** / **0** | Set from the first landings. See 4.2. |
| `AUTOLAND_TD_OFS` | Leave at the default, **20** (30 on a board without baro compensation) | Learned on every AUTOLAND you don't touch (`AUTOLAND_TDLRN` 1). See 4.3. |
| `AUTOLAND_DSC_ARC` | **90** | Use 90 for most aircraft: the glide starts on the last quarter of the final turn, so the final is shallow. Use 0 to hold height until lined up (for example, obstacles inside the final turn). Use 120–180 for slippery aircraft that gain speed on a steep final. |
| `AUTOLAND_RO_GAIN` | **1** (stock) | Raise it 0.5–1 at a time if the aircraft swings off as it slows on the rollout. Taildraggers usually need 2–3, tricycles 1–2. |
| `TKOFF_TDRAG_ELEV` | Taildragger: 30–60 (up elevator holds the tail down). Tricycle: 0, or -10 to -20 to hold the nosewheel down. | See 5. |
| `TKOFF_THR_SLEW` | 20–30 %/s | Gives the stationary run-up the baro learning needs (4.1), and a gentler start to the roll. |
| `TKOFF_TDRAG_SPD1` | Taildragger: just below `TKOFF_ROTATE_SPD` | A tail that lifts early loses tailwheel steering. |
| `TKOFF_ROTATE_SPD` | About `AIRSPEED_MIN` | |
| `TKOFF_GND_PITCH` | The aircraft's sitting pitch on its wheels (read it on the HUD) | |
| `STEER2SRV_P` | Stock procedure: the turning-circle diameter in metres. It often needs more for a light tailwheel at speed. | See 5 and 8. |
| `STEER2SRV_I` | **0** | An integrator winds up while taxiing and starts the roll with the rudder over. This firmware also resets it when TAKEOFF is selected. |
| `STEER2SRV_MINSPD` | 2–3 | |
| `LEVEL_ROLL_LIMIT` | 10 | Lets the climb-out correct crosswind drift just after liftoff. |
| `NAVL1_PERIOD` | 12–15 for small, light aircraft; 15–20 for larger ones | If the runway circle or base circle is flown wider than commanded, lower it. |
| `LAND_FLARE_SEC` / `LAND_FLARE_ALT` | 1.5 / 2 | See 7. |
| `TECS_LAND_SINK` | 0.5 | See 7. |
| `LAND_PITCH_DEG` | Taildragger 6–10, tricycle 3–5 | |
| `TECS_LAND_ARSPD` | About 1.1–1.2 × `AIRSPEED_MIN` | |

### 2.3 Checking the circuit fits the fence

The base circle radius is chosen automatically:

```
R = max(WP_LOITER_RAD, min(60, (V + W)² / (9.81 × tan(0.8 × ROLL_LIMIT_DEG))))
```

Here V is about the cruise airspeed and W is the wind estimate, limited to 4–10 m/s.

Example: V = 14 m/s, W = 4 m/s and `ROLL_LIMIT_DEG` = 45 give R ≈ 45 m.

The circuit then reaches about:

- **behind the roll start**: `AUTOLAND_WP_DIST` + R + `AUTOLAND_TD_OFS` (20 m by
  default), plus the transit if you select AUTOLAND further out. `AUTOLAND_TDLRN` can
  raise `AUTOLAND_TD_OFS` to 50 m (4.3): allow for that, or set `AUTOLAND_TDLRN` 0 if
  the fence is tight;
- **to the side**: `AUTOLAND_SIDEGAP` + 2 × R.

That point must be inside the fence by at least the fence overshoot (50–60 m for a
small aircraft). A breach during AUTOLAND abandons the landing.

The runway circle uses the same R. Its near edge is `AUTOLAND_SIDEGAP` + 10 m from
the centreline.

### 2.4 Rangefinder or not

The field mode doesn't need one.

**With a downward rangefinder**: any ArduPilot rangefinder works. Configure it, set
`RNGFND_LANDING` 1, and set its three limits correctly, because the field mode
uses only these, the status and (if reported) the signal quality:

- `RNGFND1_MIN` and `RNGFND1_MAX`: the sensor's specified range.
- `RNGFND1_GNDCLR`: what it reads standing on its wheels, measured.

**Mounting.** If the airframe allows it, mount the sensor so it sees the ground on
the wheels: lens at least `RNGFND1_MIN` + 0.05 m above the ground. Set
`RNGFND1_GNDCLR` to the measured reading. Touchdown is then detected from the
reading reaching the ground clearance. If it can't (`RNGFND1_GNDCLR` below
`RNGFND1_MIN`; you see *"Rangefinder can't see ground on wheels: blind-zone
touchdown"* at arming), touchdown is detected from the range ending at the bottom,
with the baro height under 1.5 m and the sink stopped.

**How it is used on the AUTOLAND final:**

- **Engagement.** The rangefinder is taken into use once its readings have followed
  the aircraft's own descent for at least 0.2 s. The descent is the EKF vertical
  speed. Random "valid" readings beyond a small sensor's range don't follow it.
  - With a signal quality of 50% or more, 3 samples are enough and the check is
    looser.
  - Weak readings, or a sensor without a quality, need 6 samples in a row: 0.2 s at
    50 Hz, 0.5 s at 10 Hz. Random readings can agree with the descent by chance
    once or twice, never five times running.
  - Below 20% quality a sample doesn't count.

  *"Rangefinder engaged at"* tells you the height.
- **The plausibility gate** also rejects weak readings that disagree badly with the
  baro.
- **Below the minimum range** the last correction is held, so the height doesn't
  step back to the baro.
- **If the sensor stops** working higher up, the correction fades out over 3 s.

**Range decides who flares.** For the rangefinder to decide the flare, the flare
has to start inside its *usable* range, which outdoors in sun over grass can be well
short of `RNGFND1_MAX`. With an MT-06 (strong returns below about 2.5–3 m), keep
`LAND_FLARE_ALT`, and `LAND_FLARE_SEC` × the final sink rate, at or below about
2–2.5 m. A shallow final (`AUTOLAND_DSC_ARC` 90, about 2 m/s sink) does that with
`LAND_FLARE_SEC` 1. On a steep final the flare starts above the sensor's range and
the rangefinder only refines its end. If `RNGFND1_MAX` is below `LAND_FLARE_ALT` +
1 m you see *"Rangefinder range below flare height: flare decided by baro"* at
arming.

**Measure the outdoor range, don't assume it.**

1. On the first flight, fly low passes at idle over the runway at stepped heights
   (2, 4, 6, 8 m).
2. In the log, `RFSS` shows each raw sample with its quality (`Q`), and `ALRF` shows
   the rangefinder on each AUTOLAND final.
3. The height up to which `RFSS` still shows valid samples with a good quality is
   your usable range.

**TF-Luna (Benewake) over I2C.** On a board with no free UART:

| | |
|---|---|
| Wiring | Pin 5 (configuration input) to ground for I2C mode. Pin 2 SDA, pin 3 SCL, 5 V and ground. |
| Parameters | `RNGFND1_TYPE` 25 (BenewakeTFmini-I2C), `RNGFND1_ADDR` 16 (0x10), `RNGFND1_MIN` 0.2, `RNGFND1_MAX` 8, `RNGFND1_ORIENT` 25, `RNGFND1_GNDCLR` measured, `RNGFND_LANDING` 1. Reboot. |
| Check | At boot, the message *"TFMiniPlus: found fw version …"*. Then a bench check: the rangefinder distance on Mission Planner's Status tab follows a hand or a board moved under the sensor. |
| Not detected? | The type 25 driver speaks the TFmini Plus command frames over I2C. The TF-Luna manual documents a register map for I2C mode and the command frames for serial mode only, so type 25 may not detect it. Then a TF-Luna I2C driver is needed. |
| Range | Rated 0.2–8 m at 90% reflectivity and 0.2–2.5 m at 10% (both at 90 klux). Green grass reflects strongly in near-infrared, so 4–6 m in sun is plausible, but measure it (above). |
| Blind zone | 0.2 m minimum range: on most aircraft it can't see the ground on its wheels, and touchdown uses blind-zone detection. |
| Quality | Benewake's strength is mapped to 0–100% (provisional): 0 below 100 (unreliable) and at 32768 and above (overexposed); 100 × log10(strength / 100) otherwise, so 50% at 316 and 100% from 1000. |

**Without one**: `RNGFND_LANDING` 0. The flare starts from baro height either way.
The main difference is that `AUTOLAND_RO_GAIN` starts when ArduPilot detects the
landing, which can be several seconds after touchdown.

---

## 3. Tuning order

Work through these in order, making **one change per flight** and keeping the logs.

| Flight | Do | Check |
|---|---|---|
| 1 | TAKEOFF, then fly FBWB and land manually | The roll tracks straight. *"Autoland direction="* is within a few degrees of the runway heading. The runway circle is on the far side. *"Baro: throttle scale ..."* during the roll (4.1). |
| 2 | Fly toward the fence in FBWB | It turns back and announces *"FBWB resumed"*. Note how far past the fence it went. |
| 3 | AUTOLAND from circuit height, with a hand on the override | The circuit stays on the far side, is level until the final turn, and lines up without crossing the centreline. Correct the line with aileron or rudder if needed (the landing is still learned). Take over if it is going to land short (that landing isn't learned). *"Autoland: touchdown ... TD_OFS ..."* after the touchdown (4.3). |
| 4+ | Set `BARO1_WCF_FWD` (4.2), then refine the flare and rollout. The touchdown point follows by itself (4.3). | Use the tables below. |

---

## 4. Barometer compensation

Air coming in through the cowling pressurises the whole fuselage, so the baro reads
**low**: mostly with throttle (prop wash), and a little with airspeed. On the test
aircraft it was about 4 m low at full throttle standing still and about 2 m low at
idle on the approach. Uncompensated, takeoff heights happen higher than set, TECS
adds throttle as the baro "sinks", and the flare starts high and floats.

Foam over the chip doesn't help: the pressure is the same throughout the fuselage.
The fix is two stock ArduPilot corrections, which this firmware enables on every
board and makes work on a fixed-wing forward motor. Do this before tuning the flare
and touchdown point.

Baro thrust compensation applies to the first barometer only. On a board with more
than one, leave `BARO_PRIMARY` at 0.

### 4.1 Throttle part (`BARO1_THST_SCALE`): automatic

With `AUTOLAND_BAROLRN` 1 (the default), every TAKEOFF started on the ground
measures it. Line up (point the aircraft straight down the runway by hand; see
[Lining up](../../README.md#3-takeoff-and-the-runway-circle)), stop, and select
TAKEOFF with the motor at idle. The first
second or two of the roll is a static run-up: the throttle rises while the aircraft
still stands still, so any change in the baro reading is the throttle error.

- **When TAKEOFF is selected**, the firmware takes the parked baro reading over the
  last second at idle as the baseline. It needs at least 0.5 s standing still.
- **Until the aircraft moves** (0.8 m/s or 1 m/s² forward), it records the pressure
  rise against throttle and fits a straight line through zero to the samples at 25%
  throttle and above.
- **It accepts the result** if there is at least 1 s of run-up, the throttle reached
  35% and covered at least 20%, the parked baseline was steady (under 4 Pa), and
  the result is under 150 Pa.
- **The first time** (`BARO1_THST_SCALE` 0) the measurement is used as it is. After
  that, each takeoff moves the value half way to the new measurement, so one odd
  takeoff can't move it far.
- **The new value is used straight away**, for the rest of the roll and the climb,
  and saved at disarm. You see *"Baro: throttle scale 47 Pa (measured 44, was 50)"*,
  and the details are in the log as `BTLN`.

`TKOFF_THR_SLEW` 20–30 %/s gives the run-up it needs: at 20 %/s the throttle passes
35% in under 2 s. A light tricycle on a smooth runway may start rolling almost at
once. It then reports *"Baro: run-up too short, not learned"* and changes nothing.
Use the manual run-up below instead, and set `AUTOLAND_BAROLRN` 0 if you prefer.

**Manual ground run-up**: an optional check, or the method with learning off.

1. Restrain the aircraft firmly and keep clear of the prop.
2. In FBWA or MANUAL, with the hatches and cowl as flown, watch the altitude in
   Mission Planner while you hold about 0, 50 and 100% throttle for a few seconds
   each.
3. `BARO1_THST_SCALE` ≈ the drop at 100% throttle in metres × 12 Pa/m. With
   compensation already on, add the drop that is left × 12 to the current value.
4. Repeat until the altitude stays within about ±0.3 m.

Tractor and pusher aircraft differ, and so do cowls and hatches: measure every
airframe.

### 4.2 Airspeed part (`BARO1_WCF_FWD`)

Set `BARO1_WCF_ENABLE` 1. The airspeed part is the error left at idle in forward
flight.

**With a rangefinder**: after any landing, run
`ama-field/log-analysis/baro_throttle.py` on the log. The final approach and flare
at idle, below 3 m, are the measurement. It prints the error for each landing and a
suggested value. By hand:

- `BARO1_WCF_FWD` += −(remaining error in m) × 12 / (0.6 × V²), with V the approach
  airspeed in m/s;
- that is about +0.1 for every metre the baro still reads low at 14 m/s.

Repeat over 2–3 landings until the remaining error is within ±0.3 m. Optionally,
fly idle low passes at two speeds to check that the error scales with speed².

**Without a rangefinder**: `baro_throttle.py` also prints an airborne fit of the
baro against GPS height ("airborne fit ... at 15 m/s airspeed"). It is less precise
than the rangefinder method, so take it in steps of no more than 0.1. Don't use the
touchdown point for this: with `AUTOLAND_TDLRN` on, `AUTOLAND_TD_OFS` takes up a long
or short touchdown (4.3).

Negative values are valid, for an airframe whose baro reads high in forward flight.

### 4.3 Touchdown point (`AUTOLAND_TD_OFS`): automatic

The landing aims `AUTOLAND_TD_OFS` short of the roll start. Whatever the baro
compensation leaves, plus the float after the flare, decides where it really touches
down. With `AUTOLAND_TDLRN` 1 (the default), every AUTOLAND measures that.

- **The touchdown**, with a rangefinder, is its first reading on the ground. Without
  one, it is where the sink stops for good: the GPS-aided sink rate stays below
  0.3 m/s for 1.5 s. It is placed at the first wheel impact on the accelerometer
  there, if there is one. The baro isn't used, because its remaining error is what
  makes the landing float. A bounce waits for the next contact. Two seconds down
  confirms it.
- **The float** is the distance from the aim point to the touchdown, along the
  runway line.
- **Aileron and rudder corrections are fine**: they change the direction, not the
  float.
- **The throttle stick can rest anywhere from 0 to 60 %.** In AUTOLAND it does
  nothing below half way (stock `THROTTLE_NUDGE`); above that it adds throttle in
  proportion. The learning ignores 0–60 %, so wherever the stick rests, any
  reduction, and a bump while using the rudder don't matter.
- **It is not learned** if, between the start of the final and 2 s after the
  touchdown, you move the elevator stick (over 10 %), raise the throttle stick above
  60 %, or change mode. It is also not learned if the touchdown is more than 15 m
  off the runway line.
- **Each landing moves it half way** to the float it measured, limited to 0–50 m, so
  one gusty landing can't move it far. (If `AUTOLAND_TD_OFS` is 0, the first
  measurement is used as it is.)
- **The new value is used from the next AUTOLAND** and saved at disarm. You see
  *"Autoland: touchdown +8m from roll start, TD_OFS 24 (was 20)"*. The log has the
  details as `TDLN`. *"Autoland: touchdown not seen, TD_OFS not learned"* means the
  aircraft disarmed before a touchdown was confirmed. Nothing changes.

Without a rangefinder the measurement is less exact. A flat stretch of the float,
sinking at under 0.3 m/s, can be taken as the touchdown. In SITL it was up to 7 m
early, which measures the float short, so the next landing is a little long: the
safe direction. With a rangefinder it was within 2 m.

**Start from the default**: 20 m, or 30 m on a board without baro compensation. It
is a typical float, so even the first landing should be within about 10 m of the
roll start, short or long, instead of a long way down the runway.

- **Where it comes from:** the test aircraft floated 36–48 m past the aim with the baro
  uncompensated, and 25–28 m in SITL with it compensated.
- **A clean, slow aircraft** that barely floats lands a little short at first.
- **Land at the roll start itself instead:** set 0, and it lands long the first time
  and learns from there.

Wind changes the float from day to day; the half-way update averages it out. To tune
by hand instead, set `AUTOLAND_TDLRN` 0 and change `AUTOLAND_TD_OFS` by the error in
metres (+ if long, − if short).

The flare parameters (7) still decide how the aircraft arrives. After you change
them, the float changes too, and `AUTOLAND_TD_OFS` follows over the next landings.

### 4.4 Boards without baro compensation

A 1 MB board that only fits without baro compensation is marked "Baro comp: no" in
the release notes. It has no `BARO1_THST_SCALE`, `BARO1_WCF_*` or
`AUTOLAND_BAROLRN`. Its baro error is not corrected, so the flare starts high and
the aircraft floats further. `AUTOLAND_TD_OFS` defaults to 30 m on these builds, and
`AUTOLAND_TDLRN` (4.3) works the same and takes up the rest. The test aircraft
floated 36–48 m without baro compensation, close to the 50 m learning limit. An
aircraft that floats further than that still lands somewhat long.

---

## 5. Takeoff

| Symptom | Change |
|---|---|
| Tail pops up or the aircraft hops when the tail is released | Raise `TKOFF_TDRAG_SPD1` toward `TKOFF_ROTATE_SPD`. Check that `TKOFF_GND_PITCH` equals the sitting attitude. |
| Veers during the roll (you see *"roll veered"*) | Raise `STEER2SRV_P` by 0.5–1 at a time. Check the tail is held down until near rotate speed. |
| Weaves left and right on the roll or when taxiing | Lower `STEER2SRV_P`. |
| Rudder already deflected when the roll starts | `STEER2SRV_I` 0. Stop fully before selecting TAKEOFF. |
| Drifts sideways just after liftoff | `LEVEL_ROLL_LIMIT` 10–15. |
| *"Autoland direction="* wrong by more than 5° | First, line up straighter: the aircraft must point exactly down the runway when you select TAKEOFF, because that is the direction it takes off and lands in. Then calibrate the compass away from metal. Line up and **stop** before TAKEOFF so the idle heading is sampled. |
| Runway circle too close to the runway, or crosses it on the first lap | Raise `AUTOLAND_SIDEGAP`, or lower `NAVL1_PERIOD`. |
| *"Takeoff blocked: pits side unknown"* | It was powered up on or near the runway line. Power cycle in the pits. |

---

## 6. Circuit and approach

| Symptom | Change |
|---|---|
| Too low over obstacles on downwind or base | Raise `AUTOLAND_WP_ALT`, select AUTOLAND higher, or lower `AUTOLAND_DSC_ARC` so the descent starts later. |
| Overshoots the centreline on the turn to final | Lower `NAVL1_PERIOD`, or raise `AUTOLAND_SIDEGAP`. |
| Final too steep, arrives fast | Raise `AUTOLAND_DSC_ARC`, or lengthen `AUTOLAND_WP_DIST`. |
| Final too shallow, drags in low over obstacles | Lower `AUTOLAND_DSC_ARC`, or shorten `AUTOLAND_WP_DIST`. |
| Circuit too big for the fence | Shorten `AUTOLAND_WP_DIST`. Lower `NAVL1_PERIOD` to fly the circle tighter. |
| Slow on final, sinks early | Raise `TECS_LAND_ARSPD`. |

---

## 7. Flare and touchdown

| Symptom | Change |
|---|---|
| Floats at 1–2 m, slows and drops in | Raise `TECS_LAND_SINK` in 0.1–0.2 steps. |
| Touches down fast and rolls long | Lower `TECS_LAND_SINK`. Make the final shallower (`AUTOLAND_DSC_ARC`). |
| Touches down flat, or bounces on the mains (taildragger) | Raise `LAND_PITCH_DEG` (8–10). |
| Flares far too high | Check the baro compensation first (4.2). Then lower `LAND_FLARE_SEC`. |
| Flares too late, hits hard | Raise `LAND_FLARE_SEC` or `LAND_FLARE_ALT`. |
| Lands long or short of the roll start | `AUTOLAND_TDLRN` corrects it over the next landings (4.3). If it keeps moving a lot from landing to landing, check `BARO1_WCF_FWD` (4.2) and the flare: `LAND_FLARE_SEC` / `LAND_FLARE_ALT` and `TECS_LAND_SINK`. |
| Touches down crabbed | Check *"Autoland direction="* was right. The de-crab needs a working rudder. |

---

## 8. Rollout

| Symptom | Change |
|---|---|
| Swings off the runway as it slows | Raise `AUTOLAND_RO_GAIN` (e.g. 2 → 3). |
| Weaves on the rollout | Lower `AUTOLAND_RO_GAIN`, or `STEER2SRV_P`. |
| Rollout steering starts late (no rangefinder) | Expected. It starts when ArduPilot detects the landing. |

A taildragger will still weathervane into a strong crosswind at walking speed, even
at full rudder.

---

## 9. Moving to a second aircraft or another field

- **Same aircraft type, another airframe**: the field and aircraft values carry over
  as a start. Calibrate and autotune again. The takeoffs re-learn
  `BARO1_THST_SCALE`; re-check `BARO1_WCF_FWD` and `TECS_LAND_SINK`, because
  barometer error varies between airframes.
- **Same aircraft, another field**: only the field values change (2.1). Nothing about
  the runway is stored between flights.
- **Different aircraft type**: start again from section 2 with
  [`ama_field_core.param`](../params/ama_field_core.param).
