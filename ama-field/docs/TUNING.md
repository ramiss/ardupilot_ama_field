# AMA Field Mode – setup and tuning guide

This guide takes any runway-capable ArduPlane aircraft from "flies well in FBWA" to
"auto takeoff and one-switch landing at my field". Section 2 shows how to choose
starting values. Sections 3–7 cover what to change for each symptom you see.

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
| `AUTOLAND_TD_OFS` | **0**. The aircraft will land somewhat long, which is the safe direction. | After a landing, measure how far past the roll start it touched down, then add that. |
| `AUTOLAND_DSC_ARC` | **90** | Use 90 for most aircraft: the glide starts on the last quarter of the final turn, so the final is shallow. Use 0 to hold height until lined up (for example, obstacles inside the final turn). Use 120–180 for slippery aircraft that gain speed on a steep final. |
| `AUTOLAND_RO_GAIN` | **1** (stock) | Raise it 0.5–1 at a time if the aircraft swings off as it slows on the rollout. Taildraggers usually need 2–3, tricycles 1–2. |
| `TKOFF_TDRAG_ELEV` | Taildragger: 30–60 (up elevator holds the tail down). Tricycle: 0, or -10 to -20 to hold the nosewheel down. | See 4. |
| `TKOFF_TDRAG_SPD1` | Taildragger: just below `TKOFF_ROTATE_SPD` | A tail that lifts early loses tailwheel steering. |
| `TKOFF_ROTATE_SPD` | About `AIRSPEED_MIN` | |
| `TKOFF_GND_PITCH` | The aircraft's sitting pitch on its wheels (read it on the HUD) | |
| `STEER2SRV_P` | Stock procedure: the turning-circle diameter in metres. It often needs more for a light tailwheel at speed. | See 4 and 7. |
| `STEER2SRV_I` | **0** | An integrator winds up while taxiing and starts the roll with the rudder over. This firmware also resets it when TAKEOFF is selected. |
| `STEER2SRV_MINSPD` | 2–3 | |
| `LEVEL_ROLL_LIMIT` | 10 | Lets the climb-out correct crosswind drift just after liftoff. |
| `NAVL1_PERIOD` | 12–15 for small, light aircraft; 15–20 for larger ones | If the runway circle or base circle is flown wider than commanded, lower it. |
| `LAND_FLARE_SEC` / `LAND_FLARE_ALT` | 1.5 / 2 | See 6. |
| `TECS_LAND_SINK` | 0.5 | See 6. |
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

- **behind the roll start**: `AUTOLAND_TD_OFS` + `AUTOLAND_WP_DIST` + R, plus the
  transit if you select AUTOLAND further out;
- **to the side**: `AUTOLAND_SIDEGAP` + 2 × R.

That point must be inside the fence by at least the fence overshoot (50–60 m for a
small aircraft). A breach during AUTOLAND abandons the landing.

The runway circle uses the same R. Its near edge is `AUTOLAND_SIDEGAP` + 10 m from
the centreline.

### 2.4 Rangefinder or not

The field mode doesn't need one.

**With a downward rangefinder**: configure it, set `RNGFND_LANDING` 1, and check
`RFND` in a log that it reads the ground correctly on the rollout. The plausibility
gate in this firmware rejects readings that disagree with the baro, unless the
sensor reports a strong signal, so bad readings at approach height don't trigger an
early flare.

**Without one**: `RNGFND_LANDING` 0. The flare starts from baro height either way.
The main difference is that `AUTOLAND_RO_GAIN` starts when ArduPilot detects the
landing, which can be several seconds after touchdown.

---

## 3. Tuning order

Work through these in order, making **one change per flight** and keeping the logs.

| Flight | Do | Check |
|---|---|---|
| 1 | TAKEOFF, then fly FBWB and land manually | The roll tracks straight. *"Autoland direction="* is within a few degrees of the runway heading. The runway circle is on the far side. |
| 2 | Fly toward the fence in FBWB | It turns back and announces *"FBWB resumed"*. Note how far past the fence it went. |
| 3 | AUTOLAND from circuit height, with a hand on the override | The circuit stays on the far side, is level until the final turn, and lines up without crossing the centreline. Take over if it is going to land short or off the runway. |
| 4+ | Refine the flare, touchdown point and rollout | Use the tables below. |

---

## 4. Takeoff

| Symptom | Change |
|---|---|
| Tail pops up or the aircraft hops when the tail is released | Raise `TKOFF_TDRAG_SPD1` toward `TKOFF_ROTATE_SPD`. Check that `TKOFF_GND_PITCH` equals the sitting attitude. |
| Veers during the roll (you see *"roll veered"*) | Raise `STEER2SRV_P` by 0.5–1 at a time. Check the tail is held down until near rotate speed. |
| Weaves left and right on the roll or when taxiing | Lower `STEER2SRV_P`. |
| Rudder already deflected when the roll starts | `STEER2SRV_I` 0. Stop fully before selecting TAKEOFF. |
| Drifts sideways just after liftoff | `LEVEL_ROLL_LIMIT` 10–15. |
| *"Autoland direction="* wrong by more than 5° | Calibrate the compass away from metal. Line up and **stop** before TAKEOFF so the idle heading is sampled. |
| Runway circle too close to the runway, or crosses it on the first lap | Raise `AUTOLAND_SIDEGAP`, or lower `NAVL1_PERIOD`. |
| *"Takeoff blocked: pits side unknown"* | It was powered up on or near the runway line. Power cycle in the pits. |

---

## 5. Circuit and approach

| Symptom | Change |
|---|---|
| Too low over obstacles on downwind or base | Raise `AUTOLAND_WP_ALT`, select AUTOLAND higher, or lower `AUTOLAND_DSC_ARC` so the descent starts later. |
| Overshoots the centreline on the turn to final | Lower `NAVL1_PERIOD`, or raise `AUTOLAND_SIDEGAP`. |
| Final too steep, arrives fast | Raise `AUTOLAND_DSC_ARC`, or lengthen `AUTOLAND_WP_DIST`. |
| Final too shallow, drags in low over obstacles | Lower `AUTOLAND_DSC_ARC`, or shorten `AUTOLAND_WP_DIST`. |
| Circuit too big for the fence | Shorten `AUTOLAND_WP_DIST`. Lower `NAVL1_PERIOD` to fly the circle tighter. |
| Slow on final, sinks early | Raise `TECS_LAND_ARSPD`. |

---

## 6. Flare and touchdown

| Symptom | Change |
|---|---|
| Floats at 1–2 m, slows and drops in | Raise `TECS_LAND_SINK` in 0.1–0.2 steps. |
| Touches down fast and rolls long | Lower `TECS_LAND_SINK`. Make the final shallower (`AUTOLAND_DSC_ARC`). |
| Touches down flat, or bounces on the mains (taildragger) | Raise `LAND_PITCH_DEG` (8–10). |
| Flares far too high | Lower `LAND_FLARE_SEC`. Shield the baro from airflow (foam). |
| Flares too late, hits hard | Raise `LAND_FLARE_SEC` or `LAND_FLARE_ALT`. |
| Lands long or short of the roll start | Change `AUTOLAND_TD_OFS` by the error in metres (+ if long, − if short). |
| Touches down crabbed | Check *"Autoland direction="* was right. The de-crab needs a working rudder. |

---

## 7. Rollout

| Symptom | Change |
|---|---|
| Swings off the runway as it slows | Raise `AUTOLAND_RO_GAIN` (e.g. 2 → 3). |
| Weaves on the rollout | Lower `AUTOLAND_RO_GAIN`, or `STEER2SRV_P`. |
| Rollout steering starts late (no rangefinder) | Expected. It starts when ArduPilot detects the landing. |

A taildragger will still weathervane into a strong crosswind at walking speed, even
at full rudder.

---

## 8. Moving to a second aircraft or another field

- **Same aircraft type, another airframe**: the field and aircraft values carry over
  as a start. Calibrate and autotune again. Re-check `AUTOLAND_TD_OFS` and
  `TECS_LAND_SINK`, because barometer error varies between airframes.
- **Same aircraft, another field**: only the field values change (2.1). Nothing about
  the runway is stored between flights.
- **Different aircraft type**: start again from section 2 with
  [`ama_field_core.param`](../params/ama_field_core.param).
