# ArduPlane AMA Field Mode

**Auto takeoff, fly in FBWB, one switch to land – without ever crossing the flight
line.**

A set of extensions to **ArduPlane 4.7.2** for flying at a typical club (AMA) field:
one runway, with the pits and pilots' line along one side of it. The usual rule at
such a field is that nothing flies on the pilot side of the runway. Stock ArduPlane
modes don't know where the pits are. They will loiter, turn onto final or come home
right over the pilot line. This firmware learns the field layout at power-up and
takeoff, then keeps every automatic part of the flight on the far side of the runway.

The goal is a setup that a new pilot can fly safely:

- **TAKEOFF** from where you lined up. It holds the runway line in a crosswind and
  climbs to a holding circle beside the runway, away from the pits.
- **FBWB** as the everyday flying mode. It holds altitude and won't go below a floor
  height. If it hits the fence it turns back toward the field and hands control
  back to the pilot.
- **AUTOLAND** on one switch. It flies a level circuit on the far side, descends
  only once it's turning onto final, and kicks out the crab in the flare. It touches
  down where the takeoff roll started, then holds the runway heading until it stops.

The behaviour is built into ArduPlane, so it works on **any board and any airframe
that takes off from a runway**. That means a conventional plane with a rudder, GPS
and compass; a rangefinder is optional. Development and flight testing used one
aircraft, a Freewing Pandora. Its settings are included only as a
[worked example](#example-aircraft).

> **Experimental firmware.** Everything here was tested in ArduPilot SITL and on one
> airframe over about 20 flights. Fly every change with a thumb on the mode switch
> and a MANUAL fallback, and follow your club's and the AMA's safety code.

The original ArduPilot README is in [README-ArduPilot.md](README-ArduPilot.md).

---

## Download firmware

Ready-to-flash firmware for about 30 fixed-wing flight controllers is on the
**[latest release](https://github.com/ramiss/ardupilot_ama_field/releases/latest)**.

1. **Find your board.** The release notes have a table of every board. File names
   end in ArduPilot's name for the board, such as `MatekF405-Wing` or `CubeOrange`.
   It must match your flight controller exactly. If you're not sure of the name,
   look up your board on ArduPilot's
   [autopilot hardware list](https://ardupilot.org/plane/docs/common-autopilots.html).
2. **Pick the file:**
   - `.apj` if the board already runs ArduPilot. Flash it with Mission Planner:
     *Install Firmware → Load custom firmware*.
   - `_with_bl.hex` if it runs INAV or Betaflight. Flash it once over DFU; the
     release notes explain how.
3. Your board isn't listed, or is marked "not available"? Build it yourself; see
   [DEVELOPMENT.md](ama-field/docs/DEVELOPMENT.md#building).

Only the FlyingRC F4Wing Mini has been flight-tested. The other boards compile but
haven't flown this firmware. Ground-test first, and fly with a MANUAL/FBWA override
switch.

---

## Contents

1. [The field it is designed for](#1-the-field-it-is-designed-for)
2. [A flight, step by step](#2-a-flight-step-by-step)
3. [Takeoff and the runway circle](#3-takeoff-and-the-runway-circle)
4. [The AUTOLAND circuit](#4-the-autoland-circuit)
5. [Fence bounce-back](#5-fence-bounce-back)
6. [What your aircraft needs](#6-what-your-aircraft-needs)
7. [Getting started](#7-getting-started)
8. [Parameter reference](#8-parameter-reference)
9. [Messages you will hear and see](#9-messages-you-will-hear-and-see)
10. [Limitations and safety notes](#10-limitations-and-safety-notes)
11. [Example aircraft](#example-aircraft)
12. [More documentation](#more-documentation)

---

## 1. The field it is designed for

![Field layout: runway, flight line, pits, takeoff and runway circle](ama-field/docs/images/field-takeoff.svg)

The design assumes:

- **One runway** used in one direction per flight. The direction is whichever way
  you line up for takeoff.
- **Pits and pilots on one side of the runway**, behind a flight line. The firmware
  learns which side from where the aircraft is powered up.
- **A circular fence** around HOME (`FENCE_RADIUS`) that the whole circuit fits
  inside.
- Optional **obstacles** (trees, buildings) on the far side or at the ends. A
  minimum circuit height clears them.

Nothing about the field has to be entered as coordinates. The firmware learns the
field in two moments:

| When | What it learns | How |
|---|---|---|
| Power-up in the pits | **Which side the pits are on** | The first good GPS fix is saved as the pits reference. Arming waits for it. |
| Takeoff roll | **Runway position and direction**, and the **touchdown point** | It saves the spot where the aircraft last stood still and the compass heading lined up there. It also uses the GPS track of the roll. |

So the same firmware and parameters work at any field. Power up in the pits, then
take off with TAKEOFF.

---

## 2. A flight, step by step

1. **Power up in the pits**, behind the flight line. Wait for *"Autoland: GPS fix
   point saved"*.
2. **Arm**, taxi or carry the aircraft to the runway, line it up on the centreline
   and **stop**.
3. **Select TAKEOFF.** It rolls, lifts off and climbs out along the runway line,
   then circles on the **runway circle** beside the runway, away from the pits.
4. **Fly in FBWB.** The stick sets climb/descent and speed; altitude is held when
   you let go. The fence turns it back automatically.
5. **Flip AUTOLAND** at about the circuit height you want. It flies the circuit,
   lands near where the takeoff roll began, and disarms after `LAND_DISARMDELAY`.
6. To fly again, **disarm and re-arm**. A second auto takeoff on the same arming is
   refused.

---

## 3. Takeoff and the runway circle

**Before the roll**
- TAKEOFF is refused on the ground if there is no compass heading or the pits side
  is unknown. It is also refused if this arming has already flown.
- When TAKEOFF is accepted, the steering and attitude integrators are reset. Taxiing
  can't leave the rudder wound up at the start of the roll.

**The roll and climb-out**
- The roll steers the heading you lined up on.
- The **roll start** is the last point where the armed aircraft stood still. It is
  saved as the touchdown reference.
- The **runway direction** is the GPS track of the roll. If the roll veered more
  than 8° off the lined-up heading, the compass heading measured at idle is used
  instead, so a swerve can't misalign the landing.
- Climb-out **tracks the runway line** itself, not just a point ahead. A crosswind
  can't drift the climb over the pits.

**The runway circle**
- At `TKOFF_ALT` the aircraft loiters on a circle **abeam the roll start, on the
  side away from the pits**.
- The near edge is `AUTOLAND_SIDEGAP` + 10 m from the centreline.
- The radius is sized from cruise speed, the wind estimate and the bank limit, so
  the circle it actually flies stays on its side of the runway. The centre is
  recalculated every few seconds as the wind estimate settles.
- The same circle is the **RTL** destination and the **fence-return** target.

---

## 4. The AUTOLAND circuit

![AUTOLAND circuit, top view](ama-field/docs/images/autoland-circuit.svg)

1. **Level at the selected height.** The circuit height is the height when AUTOLAND
   is selected. It is limited to between `AUTOLAND_WP_ALT` (the minimum, for
   obstacles) and 0.25 × `AUTOLAND_WP_DIST` (the maximum, so the final isn't too
   steep). Selected higher than that, it holds altitude to the circle and comes down
   on it. Selected lower than `AUTOLAND_WP_ALT`, it climbs.
2. **Base circle on the far side.** It is centred `AUTOLAND_WP_DIST` behind the
   touchdown aim, `AUTOLAND_SIDEGAP` from the extended centreline. The radius is
   sized for speed and wind, up to 60 m, so the turn onto final ends lined up rather
   than overshooting across the centreline toward the pits.
3. **Descent starts on the final turn.** It begins `AUTOLAND_DSC_ARC` degrees before
   the aircraft is lined up (90° = the last quarter of the turn). Set it to 0 to stay
   level until lined up.
4. **Final** follows a glide path by distance. This replaces the stock behaviour of
   diving at `TECS_SINK_MAX` straight after the turn.
5. **Flare.** The rudder points the nose down the runway, removing the crosswind
   crab, instead of steering the ground track. The aim point is `AUTOLAND_TD_OFS`
   short of the roll start, which allows for the flare float.
6. **Rollout.** Steering is multiplied by `AUTOLAND_RO_GAIN` from touchdown until the
   aircraft stops, so it holds the runway heading as the rudder loses authority.

![AUTOLAND height profile](ama-field/docs/images/autoland-profile.svg)

The height profile is the key safety behaviour. Stock AUTOLAND starts descending as
soon as it is selected, so it can arrive low over trees on the downwind leg. Here
the circuit stays at or above `AUTOLAND_WP_ALT` until the aircraft is turning onto
final, which is the only time it is over the cleared approach.

---

## 5. Fence bounce-back

![Fence bounce-back](ama-field/docs/images/fence-bounce.svg)

With `FENCE_ACTION` 6 (Guided) and `FENCE_RET_RALLY` 0:

- **In FBWB, FBWA or CRUISE**, a breach switches to GUIDED and turns toward the
  runway circle. Once back inside the fence and heading home (within 20°), it returns
  to the mode the pilot was flying: *"Fence: turned back, FBWB resumed"*. A novice
  who flies too far gets turned around and keeps flying.
- **In TAKEOFF, AUTOLAND or RTL**, a breach stays in GUIDED, circling the runway
  circle, until the pilot changes mode. A breach during AUTOLAND abandons the
  landing, so size the circuit to fit inside the fence.

Expect the aircraft to go 50–60 m past the fence before it has turned (small foam
aircraft at ~14 m/s; more for faster aircraft).

---

## 6. What your aircraft needs

| Need | Why |
|---|---|
| **Takeoff from a runway or grass strip** (wheels) | The touchdown point and runway direction are captured from the takeoff roll, which must be 15–250 m long. Hand or bungee launches capture nothing, and AUTOLAND then refuses with *"no takeoff point captured"*. |
| **GPS** | Pits reference, roll start, runway direction. |
| **Compass** (healthy, calibrated) | Lined-up heading. TAKEOFF is refused without it. |
| **Rudder**, ideally with a **steerable nose or tail wheel** | Ground steering on the roll, flare de-crab and rollout steering. A flying wing without a rudder gets the circuit, but none of the steering behaviour. |
| **Any ArduPilot board that runs ArduPlane 4.7** | The features are in ArduPlane itself, not in a board definition, and add only a little flash. |
| Downward **rangefinder** (optional) | Refines touchdown detection and the final metre of the flare. See [section 10](#10-limitations-and-safety-notes). |
| Airspeed sensor (optional) | Not needed; the test aircraft had none. |

**Airframe types**

| Type | Notes |
|---|---|
| Taildragger | Tested. Hold the tail down with `TKOFF_TDRAG_ELEV` (positive) and release it just below rotate speed with `TKOFF_TDRAG_SPD1`. A tail that lifts early loses tailwheel steering. |
| Tricycle | Use `TKOFF_TDRAG_ELEV` 0, or negative to hold the nosewheel down, and a lower `LAND_PITCH_DEG` (about 3–5°) so it lands on the mains. Expect less rollout steering trouble. |
| Belly lander / skids | Not supported: the roll capture needs a wheeled takeoff roll. |
| Hand launch, bungee, VTOL | Not supported: no takeoff roll to capture. |

**Included extras from the test aircraft.** These are not needed for the field mode:

- A board definition for the **FlyingRC F4Wing Mini**, which is not in upstream
  ArduPilot. It has MAVLink logging to an external logger enabled.
- Signal strength reporting and a raw-sample log (`RFSS`) for **TOFSense-F / MicoAir
  MT-06** I2C rangefinders.
- A **rangefinder plausibility gate** in ArduPlane. It rejects readings that disagree
  with baro height, unless the sensor reports a strong signal. This applies to any
  rangefinder and protects against bad readings at approach height.

---

## 7. Getting started

1. **Flash the firmware** for your board from the
   [latest release](#download-firmware), or build it yourself (see
   [DEVELOPMENT.md](ama-field/docs/DEVELOPMENT.md#building)).
2. **Set up the aircraft as normal**:
   - calibrations, radio and servo directions;
   - `AIRSPEED_CRUISE`, `AIRSPEED_MIN`, `TRIM_THROTTLE`;
   - **autotune** in FBWA;
   - fly it in FBWA and FBWB until it is well trimmed.
3. **Load [`ama-field/params/ama_field_core.param`](ama-field/params/ama_field_core.param)**.
   It switches the mode on (`AUTOLAND_OPTIONS` 6) with conservative starting values.
   Then work through every line marked FIELD or AIRFRAME.
4. **Put TAKEOFF, FBWB and AUTOLAND within reach.** For example:
   - TAKEOFF and FBWB on the mode switch;
   - AUTOLAND on its own switch (`RCx_OPTION` 183);
   - MANUAL or FBWA on another switch as the pilot's way out.
5. **Tune in order**: takeoff, circuit, flare and touchdown, then rollout. Use
   [TUNING.md](ama-field/docs/TUNING.md). It explains how to choose starting values
   for your airframe and what to change for each symptom.

---

## 8. Parameter reference

### New parameters

| Parameter | Default | Range | Typical | What it does |
|---|---|---|---|---|
| `AUTOLAND_SIDEGAP` | 0 | 0–100 m | 5–15 | Gap between the extended centreline and the near edge of the base circle. The runway circle uses this + 10 m. |
| `AUTOLAND_TD_OFS` | 0 | -50–100 m | 0–30 | Aims the landing this far short of the roll start to absorb the flare float. Raise it if the aircraft lands long. |
| `AUTOLAND_DSC_ARC` | 0 | 0–270° | 0–90 | Degrees before the end of the final turn at which the descent starts. 0 = level until lined up. |
| `AUTOLAND_RO_GAIN` | 1.0 | 1–5 | 1–3 | Steering multiplier from touchdown until stopped. |

### Existing parameters with a new meaning when `AUTOLAND_OPTIONS` = 6

| Parameter | Meaning |
|---|---|
| `AUTOLAND_OPTIONS` | Bit 1 (land at the roll start) + bit 2 (circuit away from the pits) = **6** enables everything here. With other values the firmware behaves like stock ArduPlane 4.7.2. |
| `AUTOLAND_WP_ALT` | **Minimum** circuit height (obstacle clearance). |
| `AUTOLAND_WP_DIST` | Final length: base circle distance behind the touchdown aim. It also sets the circuit height cap (0.25 ×). |

### Stock parameters the mode relies on

| Parameter | Setting | Why |
|---|---|---|
| `FENCE_ACTION` / `FENCE_RET_RALLY` | 6 / 0 | Guided return to the runway circle, and the bounce-back. |
| `FENCE_TYPE` / `FENCE_RADIUS` / `FENCE_ALT_MAX` | 3 / field | Circle plus ceiling. The circuit must fit inside. |
| `RTL_AUTOLAND` / `RALLY_TOTAL` | 0 / 0 | RTL goes to the runway circle. |
| `CRUISE_ALT_FLOOR` | field | FBWB and CRUISE won't descend below it. |
| `TKOFF_ALT` | field | Height of the runway circle after takeoff. |
| `TKOFF_TDRAG_ELEV` / `_SPD1`, `TKOFF_ROTATE_SPD`, `TKOFF_GND_PITCH` | airframe | Takeoff roll. |
| `STEER2SRV_P` / `_I` / `_MINSPD` | airframe | Ground steering on roll, flare and rollout. `_I` 0 recommended. |
| `NAVL1_PERIOD` | airframe | Lower values track the circle and final more tightly. Too high and loiters fly wide. |
| `LAND_FLARE_SEC` / `_ALT`, `LAND_PITCH_DEG`, `TECS_LAND_SINK`, `TECS_LAND_ARSPD` | airframe | Flare and touchdown. |

Every parameter, with how to choose its value, is covered in
[TUNING.md](ama-field/docs/TUNING.md).

---

## 9. Messages you will hear and see

| Message | Meaning |
|---|---|
| Autoland: GPS fix point saved | Pits reference stored. Arming is now allowed. |
| PreArm: Autoland: no GPS fix point yet | Waiting for the first good GPS fix. |
| Takeoff blocked: no compass heading | TAKEOFF refused: the compass is not healthy. |
| Takeoff blocked: pits side unknown | TAKEOFF refused: it was powered up on the runway line, or the GPS fix is too close to it. Power cycle in the pits. |
| Takeoff blocked: disarm and re-arm first | TAKEOFF refused: this arming has already flown. |
| Autoland direction= N | Runway direction captured. Check it matches the runway. |
| Autoland: roll veered X, using line-up hdg N | The roll swerved more than 8°, so the lined-up compass heading is used. |
| Autoland: no target, roll Nm | Roll too short or long. AUTOLAND is unavailable this flight. |
| Autoland: target set Nm from home | Touchdown reference captured. |
| Takeoff: loiter abeam takeoff point | TAKEOFF moved to the runway circle. |
| Fence: turned back, MODE resumed | Fence bounce-back complete. |
| Autoland: circuit at Nm, descend on final | Circuit height chosen. |
| Autoland: circuit on left/right side | Circuit side (away from the pits). |
| Autoland: on circle, descending | It was selected above the height cap, and is now coming down on the base circle. |
| Autoland: no takeoff point captured | AUTOLAND refused. |

---

## 10. Limitations and safety notes

- **It only knows the field from this power-up and this takeoff.** Always power up in
  the pits and take off with TAKEOFF. If you hand-launch, or take off manually, there
  is no touchdown reference.
- **A barometer reads low in moving air** close to the ground. On the test aircraft
  it was 2–3 m low at landing speed. The flare starts from baro height, so the
  aircraft flares high and floats; `AUTOLAND_TD_OFS` compensates. Shield the baro
  with open-cell foam and block airflow into the FC bay.
- **Rangefinders**: a small ToF sensor outdoors over grass may read reliably only
  below 3–4 m. ArduPilot needs about 1 s of steady readings, so it engages at 1–2 m
  and refines the end of the flare rather than starting it. Expect roughly the same
  landings with no rangefinder; only the touchdown detection for `AUTOLAND_RO_GAIN`
  comes later.
- **Compass interference** from motor current shifts the heading. The lined-up
  heading is sampled with the motor below 20 % for this reason.
- **The fence overshoots** by roughly one turn diameter. Place the fence well inside
  the area you really need to protect.
- **A breach during AUTOLAND abandons the landing.** Keep the circuit inside the
  fence: roughly `AUTOLAND_TD_OFS` + `AUTOLAND_WP_DIST` + 2 × circle radius behind
  the roll start, and up to about 2 × radius + `AUTOLAND_SIDEGAP` to the side.
- **LOITER mode** circles wherever it is selected, including over the pits. Only the
  automatic modes described here know about the pits side.
- SITL can't model tailwheel steering. Ground handling was tuned from flight logs.

---

## Example aircraft

All flight testing was done on one aircraft:

| | |
|---|---|
| Airframe | Freewing Pandora, a 1.2 m foam high-wing taildragger, on 3S. Rudder and tailwheel share one output. |
| Flight controller | FlyingRC F4Wing Mini (STM32F405) |
| Sensors | u-blox GPS, external compass, no airspeed sensor, MicoAir MT-06 rangefinder (I2C) |
| Radio | ExpressLRS, EdgeTX with a spring-centred throttle |
| Field | Grass runway, pits along one side, trees beyond the far end |

Its settings are in
[`ama-field/params/examples/freewing_pandora_recommended.param`](ama-field/params/examples/freewing_pandora_recommended.param).
These values were right for that aircraft at that field, and they are there to show
what a finished setup looks like. They are **not defaults**: your aircraft's speed,
weight, drag and steering will need different values. Here is how its key values
compare with the core file:

| Parameter | Pandora (example) | Why on that aircraft |
|---|---|---|
| `AUTOLAND_WP_ALT` / `WP_DIST` | 20 / 85 | Trees beyond one end; a short final for a draggy foam aircraft. |
| `AUTOLAND_TD_OFS` | 25 | The baro read 2–3 m low, so it flared high and floated about 25 m. |
| `AUTOLAND_DSC_ARC` | 90 | A level circuit gave a ~16° final and a fast touchdown. |
| `AUTOLAND_RO_GAIN` | 3 | The tailwheel lost authority as it slowed. |
| `TKOFF_TDRAG_ELEV` / `_SPD1` | 60 / 9 | Releasing the tail at 5 m/s lifted it early and lost steering. |
| `STEER2SRV_P` / `_I` | 6 / 0 | A light tailwheel needed about twice the nominal gain. I wound up while taxiing. |
| `TECS_LAND_SINK` / `LAND_PITCH_DEG` | 0.6 / 8 | 0.4 floated, 0.8 landed fast. Tail-low touchdown. |
| `NAVL1_PERIOD` | 13 | 17 flew circles about twice too wide in wind. |

A full parameter dump from one flight is also there
(`freewing_pandora_2026-11-15_full_dump.param`). It contains that aircraft's
calibration values. Use it for reference only, and never load it onto another
aircraft.

---

## More documentation

- **[TUNING.md](ama-field/docs/TUNING.md)**: choosing starting values for your
  airframe and field, the tuning order, and what to change for each symptom.
- **[DEVELOPMENT.md](ama-field/docs/DEVELOPMENT.md)**: building, SITL test scripts,
  log analysis scripts, the code changes, commits and version history.
