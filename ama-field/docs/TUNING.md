# AMA Field Mode – setup and tuning guide

This guide takes any runway-capable ArduPlane aircraft from "flies well in FBWA" to
"auto takeoff and one-switch landing at my field". Section 2 shows how to choose
starting values. Section 4 checks the barometer and sets the touchdown point. Sections 5–8 cover
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
| `AUTOLAND_SIDEGAP` | Buffer between the runway centreline and the field mode's two circles, on the side away from the pits: the AUTOLAND base circle is this far out (the turn onto final ends there, so it can't overshoot toward the pits), and the runway circle for TAKEOFF, RTL (without a rally point) and the fence return is this + 10 m out (at least 15 m). Other loiters (LOITER mode, mission and GUIDED loiters) aren't affected. Use 5 m for a small, precise aircraft, and 10–15 m if the turn to final overshoots toward the pits or the aircraft is fast. Don't use 0: the turn then overshoots across the centreline toward the pits. | 5 m |
| `TKOFF_ALT` | Height of the runway circle after takeoff. It must be well above `AUTOLAND_WP_ALT`. | 50 m |
| `CRUISE_ALT_FLOOR` | Floor height for FBWB and CRUISE. It is usually the same as `AUTOLAND_WP_ALT`. | 20 m |
| `FENCE_RADIUS` | Large enough that the whole circuit fits inside with margin (see 2.3), and small enough to keep a novice close. | 240 m |
| `FENCE_ALT_MAX` | Club or AMA ceiling, with margin. | 100 m |

### 2.2 Aircraft values

| Parameter | Starting value | How to refine it |
|---|---|---|
| `AUTOLAND_TD_OFS` | The default, **25** | Adjust by hand from the touchdown reports. See 4.2. |
| `AUTOLAND_DSC_ARC` | **180** | Where the aircraft starts the even circle descent, in degrees still to turn. Arriving from downwind the turn onto final is about 180°, so 180 starts it as that turn begins. The sink rate is about (circuit height − `AUTOLAND_FNL_HGT`) ÷ the time to fly (`AUTOLAND_DSC_ARC` − 60)° of the circle: on a 21 m circuit, 8 m final height and a 54 m circle at 16 m/s, 180 gives about 1.8 m/s and 150 about 2.4 m/s; below that it can't come down in the turn ("Autoland: descent N m/s, raise AUTOLAND_DSC_ARC") and flies another lap. A value bigger than the arrival turn (240) starts where the aircraft is established on the circle. 0 holds height until lined up. See 6. |
| `AUTOLAND_FNL_HGT` | **8** | Height at the start of the final. With `AUTOLAND_WP_DIST` it sets the final's angle: 8 m over 85 m is about 5.4°, 1.5 m/s at 16 m/s. Lower is gentler but lower over the ground before the aim point. See 6. |
| `TECS_PITCH_MIN` | **-12** | Limits nose-down pitch in the automatic modes, so nothing on the approach can become a dive (about 3.3 m/s at 16 m/s). Steeper if the aircraft can't descend fast enough elsewhere in AUTO. |
| `AUTOLAND_RO_GAIN` | **1** (stock) | Raise it 0.5–1 at a time if the aircraft swings off as it slows on the rollout. Taildraggers usually need 2–3, tricycles 1–2. |
| `TKOFF_TDRAG_ELEV` | Taildragger: 30–60 (up elevator holds the tail down). Tricycle: 0, or -10 to -20 to hold the nosewheel down. | See 5. |
| `TKOFF_THR_SLEW` | 20–30 %/s | A gentler start to the roll. |
| `TKOFF_TDRAG_SPD1` | Taildragger: just below `TKOFF_ROTATE_SPD` | A tail that lifts early loses tailwheel steering. |
| `TKOFF_ROTATE_SPD` | About `AIRSPEED_MIN` | |
| `TKOFF_GND_PITCH` | The aircraft's sitting pitch on its wheels (read it on the HUD) | |
| `STEER2SRV_P` | Stock procedure: the turning-circle diameter in metres. It often needs more for a light tailwheel at speed. | See 5 and 8. |
| `STEER2SRV_I` | **0** | An integrator winds up while taxiing and starts the roll with the rudder over. This firmware also resets it when TAKEOFF is selected. |
| `STEER2SRV_MINSPD` | 2–3 | |
| `LEVEL_ROLL_LIMIT` | 10 | Lets the climb-out correct crosswind drift just after liftoff. |
| `NAVL1_PERIOD` | 12–15 for small, light aircraft; 15–20 for larger ones | If the runway circle or base circle is flown wider than commanded, lower it. |
| `LAND_FLARE_SEC` / `LAND_FLARE_ALT` | 2 / 2 | See 7. |
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

- **behind the roll start**: `AUTOLAND_WP_DIST` + R + `AUTOLAND_TD_OFS` (25 m by
  default), plus the transit if you select AUTOLAND further out;
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
2–3 m. The gentle final (`AUTOLAND_FNL_HGT` 8, about 1.5 m/s sink) does that with
`LAND_FLARE_SEC` 2. On a steep final the flare starts above the sensor's range and
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
| 1 | TAKEOFF, then fly FBWB and land manually | The roll tracks straight. *"Autoland direction="* is within a few degrees of the runway heading. The runway circle is on the far side. The baro reads true height (4.1). |
| 2 | Fly toward the fence in FBWB | It turns back and announces *"FBWB resumed"*. Note how far past the fence it went. |
| 3 | AUTOLAND from circuit height, with a hand on the override | The circuit stays on the far side and level until the descent start, comes down the circle evenly, levels off and lines up without crossing the centreline. The final is a gentle glide, not a dive. Correct the line with aileron or rudder if needed. Take over if it is going to land short. *"Autoland: touchdown ...m from roll start"* after the touchdown (4.2). |
| 4+ | Set `AUTOLAND_TD_OFS` from the touchdown reports (4.2), then refine the flare and rollout. | Use the tables below. |

---

## 4. Barometer and touchdown point

### 4.1 The barometer must read true height

Every height on the circuit, the final and the flare decision comes from the baro.
Check it before tuning anything on the landing.

On a closed foam fuselage the prop wash coming in through the cowling pressurises
the whole inside, so the baro reads **low** with throttle: about 4 m low at full
throttle on the test aircraft. Foam over the chip doesn't help, because the pressure
is the same throughout the fuselage.

- **Check it** with a ground run-up: restrain the aircraft firmly, keep clear of the
  prop, and in FBWA or MANUAL, with the hatches and cowl as flown, watch the altitude
  in Mission Planner while you hold about 0, 50 and 100% throttle for a few seconds
  each. It should stay within about ±0.5 m.
- **If it drops**, vent the fuselage. Any fuselage that air can get into needs an
  exit of at least 1 square inch, ideally more, behind the flight controller,
  wherever the motor is (README,
  [Fuselage vent](../../README.md#fuselage-vent-required)). On the example aircraft
  one opening was enough.
- **In flight**, with a rangefinder, the log's `ALRF.Corr` on each AUTOLAND final is
  how far the baro height is above the rangefinder's (positive = baro reads high).
  More than about 1 m either way needs looking at.

Leave `BARO1_THST_SCALE` and `BARO1_WCF_ENABLE` at 0: the field mode relies on a
vented fuselage, not on software compensation.

### 4.2 Touchdown point (`AUTOLAND_TD_OFS`)

The landing aims `AUTOLAND_TD_OFS` short of the roll start; the float after the
flare carries the aircraft on toward it. Every AUTOLAND reports where it touched
down, along the runway line from the roll start: *"Autoland: touchdown +4m from roll
start"* (+ past it, − before it). The log has the details as `TDLN`. Nothing is
changed automatically.

- **The touchdown**, with a rangefinder, is its first reading on the ground. Without
  one, it is where the sink stops for good: the GPS-aided sink rate stays below
  0.3 m/s for 1.5 s, placed at the first wheel impact on the accelerometer there if
  there is one. A bounce waits for the next contact. Two seconds down confirms it.
- **The suggestion.** Each report is followed by *"Autoland: suggest AUTOLAND_TD_OFS 29
  (now 25)"*: the current value plus the reported distance, which would have put that
  landing on the roll start. Average it over several landings and set that: wind
  changes the float from day to day.
- **Use clean landings.** Elevator input, or the throttle stick above 60 %, on the
  final changes where it touches down. `TDLN.PIn` is 1 on those landings. Aileron and
  rudder corrections don't matter much.

It isn't set automatically: a touchdown moved by wind, a height error or a pilot
correction isn't the aircraft's normal float, so set it from several clean landings.

**Start from the default**, 25 m. On a steeper final the example aircraft's wheels
met the ground within about 5 m of the aim point with it. The gentle final (6) arrives slower
and floats less: in SITL it touched down about 20 m shorter with the same
`AUTOLAND_TD_OFS`. Watch the first reports and lower it if it lands short, and make
sure the ground before the runway is clear.

The flare parameters (7) decide how the aircraft arrives. After you change them,
check the touchdown reports again.

---

## 5. Takeoff

| Symptom | Change |
|---|---|
| Tail pops up or the aircraft hops when the tail is released | Raise `TKOFF_TDRAG_SPD1` toward `TKOFF_ROTATE_SPD`. Check that `TKOFF_GND_PITCH` equals the sitting attitude. |
| Veers during the roll (you see *"roll veered"*) | Raise `STEER2SRV_P` by 0.5–1 at a time. Check the tail is held down until near rotate speed. |
| Weaves left and right on the roll or when taxiing | Lower `STEER2SRV_P`. |
| Rudder already deflected when the roll starts | `STEER2SRV_I` 0. Stop fully before selecting TAKEOFF. |
| Drifts sideways just after liftoff | `LEVEL_ROLL_LIMIT` 10–15. |
| Pitch hunts on the climb-out with the elevator stepping (no airspeed sensor) | The airspeed estimate is reading low, so the tail hold and ground pitch stay on. The field mode releases them 1.5 m up (*"Takeoff: airborne, tail hold released"*). The low estimate also raises the control gains until the wind estimate settles; an airspeed sensor removes both. |
| *"Autoland direction="* wrong by more than 5° | First, line up straighter: the aircraft must point exactly down the runway when you select TAKEOFF, because that is the direction it takes off and lands in. Then calibrate the compass away from metal. Line up and **stop** before TAKEOFF so the idle heading is sampled. |
| Runway circle too close to the runway, or crosses it on the first lap | Raise `AUTOLAND_SIDEGAP`, or lower `NAVL1_PERIOD`. |
| *"Takeoff blocked: pits side unknown"* | It was powered up on or near the runway line. Power cycle in the pits. |

---

## 6. Circuit and approach

| Symptom | Change |
|---|---|
| Too low over obstacles on downwind or base | Raise `AUTOLAND_WP_ALT`, select AUTOLAND higher, or lower `AUTOLAND_DSC_ARC` so the descent starts later (faster sink). |
| Too low over obstacles on the base circle | Lower `AUTOLAND_DSC_ARC` (it stays higher for longer, then sinks faster; not below about 150 on a 20 m circuit), or raise `AUTOLAND_FNL_HGT`. |
| Circle descent sinks too fast | Raise `AUTOLAND_DSC_ARC` (an earlier start, the same height to lose over more of the circle), or lower the circuit height. |
| *"Autoland: Nm high on the final turn, another lap"* every time, or *"Autoland: descent N m/s, raise AUTOLAND_DSC_ARC"* | The descent arc is too short for the height to lose, or it joins the circle too late: raise `AUTOLAND_DSC_ARC`, raise `AUTOLAND_FNL_HGT`, or select AUTOLAND lower. The extra lap is flown at `AUTOLAND_FNL_HGT`. |
| Overshoots the centreline on the turn to final | Lower `NAVL1_PERIOD`, or raise `AUTOLAND_SIDEGAP`. |
| Weaves left and right on the final | The final tracks the runway line with 0.6 × `NAVL1_PERIOD`; raise `NAVL1_PERIOD` by 1–2. |
| Final too steep, arrives fast | Lower `AUTOLAND_FNL_HGT`, or lengthen `AUTOLAND_WP_DIST`. |
| Final too shallow, drags in low over obstacles | Raise `AUTOLAND_FNL_HGT`, or shorten `AUTOLAND_WP_DIST`. |
| Dives on the final (sink well above the planned rate) | Check `AUTOLAND_FNL_HGT` is set (0 gives one continuous glide, which starts the final high) and the baro (4.1). `TECS_PITCH_MIN` -12 limits it. |
| Circuit too big for the fence | Shorten `AUTOLAND_WP_DIST`. Lower `NAVL1_PERIOD` to fly the circle tighter. |
| Slow on final, sinks early | Raise `TECS_LAND_ARSPD`. |

---

## 7. Flare and touchdown

| Symptom | Change |
|---|---|
| Floats at 1–2 m, slows and drops in | Raise `TECS_LAND_SINK` in 0.1–0.2 steps. |
| Touches down fast and rolls long | Lower `TECS_LAND_SINK`. Make the final gentler (`AUTOLAND_FNL_HGT`). |
| Touches down flat, or bounces on the mains (taildragger) | Raise `LAND_PITCH_DEG` (8–10). |
| Flares far too high | Check the baro first (4.1). Then lower `LAND_FLARE_SEC`. |
| Flares too late, hits hard | Raise `LAND_FLARE_SEC` or `LAND_FLARE_ALT`. |
| Lands long or short of the roll start | Set `AUTOLAND_TD_OFS` to the average suggested value (4.2). If it keeps moving a lot from landing to landing, check the baro (4.1) and the flare: `LAND_FLARE_SEC` / `LAND_FLARE_ALT` and `TECS_LAND_SINK`. |
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
  as a start. Calibrate and autotune again. Check the baro on a run-up (4.1),
  and `AUTOLAND_TD_OFS` and `TECS_LAND_SINK` from the first landings.
- **Same aircraft, another field**: only the field values change (2.1). Nothing about
  the runway is stored between flights.
- **Different aircraft type**: start again from section 2 with
  [`ama_field_core.param`](../params/ama_field_core.param).
