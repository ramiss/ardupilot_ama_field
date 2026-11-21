# ArduPlane AMA Field Mode

**Auto takeoff, fly in FBWB, one switch to land – without ever crossing the flight
line.**

A set of extensions added to, and re-compiled as, **ArduPlane 4.7.2** for flying as a new pilot in training at a typical club (AMA) field:
The 
one runway, with the pits and pilots' line along one side of it. The usual rule at
such a field is that nothing flies on the pilot side of the runway. Stock ArduPlane
modes don't know where the pits are. They will loiter, turn onto final or come home
right over the pilot line. This firmware learns the field layout at power-up and
takeoff, then keeps every automatic part of the flight on the far side of the runway.

The goal is a setup that a new pilot can fly safely:

- **AUTO TAKEOFF** from where you lined up (pointed the aircraft straight down the runway - on one switch.
  by hand). It holds the runway line in a crosswind and
  climbs to a holding circle beside the runway, away from the pits.
- **FBWB** Once ready, the pilot flips into FBWB as the everyday flying mode. It holds altitude and won't go below a floor
  height. If it hits the fence it turns back toward the field and hands control
  back to the pilot.
- **AUTOLAND** on one switch. It flies a level circuit on the far side, descends
  only once it's turning onto final, and kicks out the crab in the flare. It touches
  down where the takeoff roll started, then holds the runway heading until it stops.

The inherent behaviour is built into ArduPlane, this custom version essentially controls where the plane is allowed to fly, so it works on **any board and any airframe
that takes off from a runway**. That means a conventional plane with a rudder, GPS
and compass; a rangefinder is optional and will increase landing accuracy. Development and flight testing used one
aircraft, a Freewing Pandora. Its settings are included only as a
[worked example](#example-aircraft).

> **Experimental firmware.** Everything here was tested in ArduPilot SITL and on one
> airframe over about 20 flights. Experienced pilots should set up and test the aircraft before handing to a new pilot. Initial setup should test every change with a thumb on the mode switch
> and a MANUAL fallback, and follow your club's and the AMA's safety code.

**Minimum hardware**

- **An ArduPilot flight controller that runs ArduPlane 4.7**, with its barometer. See list of boards and compiled builds on the Releases tab.
  Every ArduPilot board has one; the field mode corrects it for prop wash and
  airspeed.
- **GPS**: for the pits reference, the roll start (landing point) and the runway
  direction.
- **Compass**: required. ArduPlane can fly without one, but the field mode can't.
  The heading the aircraft is lined up on at idle is the takeoff heading and the
  check on the runway direction, so TAKEOFF is refused without a healthy compass.
- **A wheeled aircraft with a rudder** that takes off from a runway or grass strip.
  A steerable tail or nose wheel is best.
- Optional: a downward rangefinder for a more precise flare and touchdown. An airspeed
  sensor isn't needed.

Details are in [What your aircraft needs](#6-what-your-aircraft-needs).

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

Some 1 MB boards don't fit the full firmware, so they ship a **trimmed** build with
less-used features removed, such as QuadPlane, soaring and camera/gimbal support.
The release notes list exactly what each trimmed board is missing. A board that
only fits without baro compensation is marked "Baro comp: no"; its landings rely on
the learned `AUTOLAND_TD_OFS`. See [TUNING.md](ama-field/docs/TUNING.md#44-boards-without-baro-compensation).

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
  you line up for takeoff: the aircraft must point straight down the runway, because
  the landing comes in along that same line.
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
2. **Arm**, taxi or carry the aircraft to the runway, **line it up**, pointing it
   straight down the centreline by hand, and **stop**. Any angle to the runway
   becomes the landing direction too (see [Lining up](#3-takeoff-and-the-runway-circle)).
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

> **Lining up.** Before selecting TAKEOFF, point the aircraft by hand (or by taxiing)
> exactly straight down the runway centreline, then stop. The firmware takes the
> direction the aircraft points at idle as the takeoff heading. The roll follows
> it, and the landing direction comes from the roll and that heading. Nothing
> corrects it to the runway: an aircraft lined up 10° off rolls 10° off, and
> AUTOLAND then lands 10° across the runway. The final still ends at the roll
> start, but 100 m out it is about 17 m to the side of the centreline, and the
> rollout carries on across the runway at the same angle.

**Before the roll**
- TAKEOFF is refused on the ground if there is no compass heading or the pits side
  is unknown. It is also refused if this arming has already flown.
- When TAKEOFF is accepted, the steering and attitude integrators are reset. Taxiing
  can't leave the rudder wound up at the start of the roll.

**The roll and climb-out**
- The roll steers the heading you lined up on, so line up straight (see *Lining
  up* above).
- The **roll start** is the last point where the armed aircraft stood still. It is
  saved as the touchdown reference.
- The **runway direction** is the GPS track of the roll. If the roll veered more
  than 8° off the lined-up heading, the compass heading measured at idle is used
  instead, so a swerve can't misalign the landing. A crooked line-up still does:
  both directions come from it.
- Climb-out **tracks the runway line** itself, not just a point ahead. A crosswind
  can't drift the climb over the pits.

**Baro learning on the run-up.** The first second or two of the roll is a free
static run-up: the throttle comes up while the aircraft still stands still. With
`AUTOLAND_BAROLRN` 1 (the default), the firmware measures how far the baro reading
moves as the throttle rises. It updates `BARO1_THST_SCALE` straight away, so the
rest of the roll and the climb use corrected altitude, and saves it at disarm. You
see *"Baro: throttle scale 47 Pa (measured 44, was 50)"*. If the aircraft rolls
almost at once, nothing is learned: *"Baro: run-up too short, not learned"*. See
[TUNING.md](ama-field/docs/TUNING.md#4-barometer-compensation).

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
   crab, instead of steering the ground track. The touchdown aim is
   `AUTOLAND_TD_OFS` short of the roll start. Stock ArduPilot already aims the glide
   path short of that by the flare distance, so the flare ends there. The float
   after the flare carries the aircraft on, and `AUTOLAND_TDLRN` measures it on
   every landing and sets `AUTOLAND_TD_OFS` to it, so the touchdown moves onto the
   roll start.
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
| Downward **rangefinder** (optional) | Refines touchdown detection and the flare. Any ArduPilot rangefinder works: the field mode uses only its `RNGFND1_MIN`, `RNGFND1_MAX` and `RNGFND1_GNDCLR`, its status and, if it reports one, its signal quality. **Range matters:** a sensor that really reaches 4 m or more outdoors (a Benewake TF-Luna, or a longer-range lidar or radar) lets the rangefinder decide the flare on any final; a small ToF sensor like the MicoAir MT-06 only engages at about 2.5–3 m. A **minimum range** above the sensor's height on the wheels (the TF-Luna's 0.2 m) is handled by blind-zone touchdown detection. See [TUNING.md 2.4](ama-field/docs/TUNING.md#24-rangefinder-or-not) and [section 10](#10-limitations-and-safety-notes). |
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
- Signal quality reporting and a raw-sample log (`RFSS`) for **TOFSense-F / MicoAir
  MT-06** and **Benewake I2C (`RNGFND1_TYPE` 25, TFmini Plus; TF-Luna, if it is
  detected)** rangefinders. The TOFSense-F is polled at 50 Hz.
- A **rangefinder plausibility gate** in ArduPlane. It rejects readings that disagree
  with baro height, unless the sensor reports a strong signal. This applies to any
  rangefinder and protects against bad readings at approach height.
- **Rangefinder handling on the AUTOLAND final, for any sensor:**
  - it is taken into use once its readings have followed the aircraft's own descent
    for 0.2 s (6 samples if the signal is weak or unreported: 0.5 s at 10 Hz);
  - the rangefinder correction is held as the aircraft goes below the sensor's
    minimum range;
  - touchdown is detected from the ground clearance, or from the blind zone below
    the minimum range.

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
5. **Tune in order**: takeoff (the first auto takeoff also measures the baro
   throttle error), baro airspeed compensation, circuit, flare and touchdown, then
   rollout. Follow the [quick start](#quick-start) below,
   with [TUNING.md](ama-field/docs/TUNING.md) for how to choose starting values for
   your airframe and what to change for each symptom.

### Quick start

For an aircraft that is already set up and flying well in FBWA/FBWB. Only the AMA
field settings are listed; everything else is stock ArduPlane.

**1. Turn it on**

- Load [`ama_field_core.param`](ama-field/params/ama_field_core.param). It sets
  `AUTOLAND_OPTIONS` 6 (land at the roll start, circuit away from the pits), the
  fence bounce-back (`FENCE_ACTION` 6, `FENCE_RET_RALLY` 0), RTL to the runway circle
  (`RTL_AUTOLAND` 0, no rally points) and the baro compensation.
- Put AUTOLAND on a switch (`RCx_OPTION` 183) and TAKEOFF and FBWB on the mode
  switch.

**2. Fit the circuit to the field**

| Parameter | Set to | Typical |
|---|---|---|
| `AUTOLAND_WP_ALT` | Tallest obstacle near the circuit + 5 m. The circuit never goes lower before the final. | 20 m |
| `AUTOLAND_WP_DIST` | Final length, about 6–8 s at landing speed. | 80–120 m |
| `AUTOLAND_SIDEGAP` | Gap between the centreline and the base circle. | 5–15 m |
| `AUTOLAND_DSC_ARC` | Degrees of the final turn the descent starts on. | 90 |
| `CRUISE_ALT_FLOOR` | FBWB/CRUISE floor, usually = `AUTOLAND_WP_ALT`. | 20 m |
| `FENCE_RADIUS`, `FENCE_ALT_MAX` | Big enough for `AUTOLAND_WP_DIST` + `AUTOLAND_TD_OFS` (up to 50 m) + base circle radius behind the roll start, and `AUTOLAND_SIDEGAP` + 2 × radius to the side, plus the fence overshoot. | 240–300 m |

The base circle radius is sized automatically from speed, wind and `ROLL_LIMIT_DEG`
([TUNING.md 2.3](ama-field/docs/TUNING.md#23-checking-the-circuit-fits-the-fence)).

**3. Leave the learned values at their defaults**

| Parameter | Default | Learned from |
|---|---|---|
| `BARO1_THST_SCALE` | 0 | Every auto takeoff run-up (`AUTOLAND_BAROLRN` 1) |
| `AUTOLAND_TD_OFS` | 20 m (30 m on a board without baro compensation) | Every AUTOLAND touchdown you don't touch (`AUTOLAND_TDLRN` 1) |
| `BARO1_WCF_ENABLE` / `BARO1_WCF_FWD` | 1 / 0 | Set `BARO1_WCF_FWD` from `baro_throttle.py` after the first landings (step 6) |

The airspeed baro compensation needs EKF3 (`AHRS_EKF_TYPE` 3, the default). On a
board with two barometers keep `BARO_PRIMARY` 0: the thrust compensation applies to
BARO1.

**4. Airframe settings the field mode relies on**

| Parameter | Set to | Why |
|---|---|---|
| `TKOFF_THR_SLEW` | 20–30 %/s | Gives the stationary run-up that measures `BARO1_THST_SCALE`. |
| `STEER2SRV_I` | 0 | Stops the integrator winding up while taxiing. |
| `TKOFF_TDRAG_ELEV` / `_SPD1` | Taildragger: 30–60 / just below `TKOFF_ROTATE_SPD`. Tricycle: 0 or negative. | Holds the steering wheel on the ground for the roll. |
| `AUTOLAND_RO_GAIN` | 1, raised 0.5–1 at a time | Rollout steering as the rudder loses authority. |
| `LAND_FLARE_SEC` / `_ALT`, `TECS_LAND_SINK`, `LAND_PITCH_DEG` | Core file values, then per airframe | The flare. The touchdown point is learned around it. |
| `RNGFND_LANDING` | 1 with a downward rangefinder, else 0 | Optional, any sensor. Set `RNGFND1_MIN`, `_MAX` and `_GNDCLR` (measured on the wheels) correctly: the field mode uses them to decide engagement and touchdown. |

**5. Fly it**

- **Power up in the pits**, behind the flight line. The first GPS fix is the pits
  reference; arming waits for *"Autoland: GPS fix point saved"*.
- **Line up, stop at idle, select TAKEOFF.** Point the aircraft exactly straight
  down the runway by hand: the direction it points becomes the takeoff and landing
  direction, and any angle to the runway lands it across the runway by the same
  angle. The roll start becomes the landing point, and the run-up measures the baro
  (*"Baro: throttle scale ..."*). One ground takeoff per arming.
- **Fly FBWB.** The fence turns the aircraft back and resumes the mode.
- **Flip AUTOLAND** at any height. It flies a level circuit on the far side, descends
  on the final turn and lands at the roll start (*"Autoland: touchdown ...,
  TD_OFS ..."*).
- **Leave the elevator alone, and the throttle stick below 60 %**, from the final
  until a few seconds after touchdown if you want the landing learned. The throttle
  stick doesn't need centring: in AUTOLAND it does nothing below half way (stock
  ArduPlane throttle nudge), so idle is the natural place for it, and 0–60 % is
  ignored by the learning. Above half it adds throttle and lengthens the landing.
  Aileron and rudder corrections are fine. Elevator input, the throttle stick above
  60 %, or a mode change just skips the learning for that landing.

**6. After the first landings**

- Run `ama-field/log-analysis/baro_throttle.py` on the logs and set `BARO1_WCF_FWD`
  to the suggested value ([TUNING.md 4.2](ama-field/docs/TUNING.md#42-airspeed-part-baro1_wcf_fwd)).
- `BARO1_THST_SCALE` and `AUTOLAND_TD_OFS` settle by themselves over 3–4 flights.
  After a flare change, `AUTOLAND_TD_OFS` follows over the next landings.
- Symptom-by-symptom tuning of the circuit, flare and rollout is in
  [TUNING.md](ama-field/docs/TUNING.md); the
  [Pandora example](#example-aircraft) shows one finished setup.

---

## 8. Parameter reference

### New parameters

| Parameter | Default | Range | Typical | What it does |
|---|---|---|---|---|
| `AUTOLAND_SIDEGAP` | 0 | 0–100 m | 5–15 | Gap between the extended centreline and the near edge of the base circle. The runway circle uses this + 10 m. |
| `AUTOLAND_DSC_ARC` | 0 | 0–270° | 0–90 | Degrees before the end of the final turn at which the descent starts. 0 = level until lined up. |
| `AUTOLAND_RO_GAIN` | 1.0 | 1–5 | 1–3 | Steering multiplier from touchdown until stopped. |
| `AUTOLAND_BAROLRN` | 1 | 0–1 | 1 | Learn `BARO1_THST_SCALE` on each auto takeoff's run-up, apply it straight away and save it at disarm. 0 = off. |
| `AUTOLAND_TD_OFS` | 20 (30 without baro comp) | -50–100 m | 10–40 | Aims the landing this far short of the roll start to absorb the float after the flare. The default is a typical float, so an untrained aircraft lands near the roll start. Learned by `AUTOLAND_TDLRN`. |
| `AUTOLAND_TDLRN` | 1 | 0–1 | 1 | Measure where each AUTOLAND touches down and move `AUTOLAND_TD_OFS` half way toward the float it measured (0–50 m), used from the next landing and saved at disarm. Not learned after elevator input, the throttle stick above 60 %, or a mode change; aileron and rudder corrections are allowed. 0 = off. |

### Baro compensation

These are stock ArduPilot parameters, enabled in every build of this firmware
unless a board only fits without them. In this firmware thrust compensation also
works on a fixed-wing forward motor: stock ArduPilot only uses it with VTOL
motors.

| Parameter | Default | Typical | What it does |
|---|---|---|---|
| `BARO1_THST_SCALE` | 0 | 30–60 Pa | Pressure error at full throttle, from the prop wash pressurising the fuselage. Learned on takeoff by `AUTOLAND_BAROLRN`. |
| `BARO1_WCF_ENABLE` | 0 | 1 | Turns on the airspeed part. |
| `BARO1_WCF_FWD` | 0 | 0–0.3 | Pressure error as a fraction of the dynamic pressure, in forward flight. Set from a landing log with `baro_throttle.py`. |

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
| `TKOFF_THR_SLEW` | 20–30 | A gentle throttle ramp gives the stationary run-up that `AUTOLAND_BAROLRN` measures. |
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
| Baro: throttle scale N Pa (measured M, was O) | The takeoff run-up measured the baro thrust error; `BARO1_THST_SCALE` is now N, saved at disarm. |
| Baro: run-up too short, not learned | The aircraft rolled before the throttle had come up far enough. Nothing changed. |
| Baro: run-up too noisy / out of range, not learned | The parked baro reading was unsteady (wind gusts, someone moving the aircraft), or the result was implausible. Nothing changed. |
| Baro: no still baseline, not learned | TAKEOFF was selected without half a second standing still at idle first. Nothing changed. |
| Rangefinder engaged at Nm | The rangefinder is now used for the flare and touchdown. |
| Rangefinder can't see ground on wheels: blind-zone touchdown | At arming: `RNGFND1_GNDCLR` is below `RNGFND1_MIN`, so touchdown is detected from the range ending at the bottom (information only). |
| Rangefinder range below flare height: flare decided by baro | At arming: `RNGFND1_MAX` is less than `LAND_FLARE_ALT` + 1 m (information only). |
| Autoland: touchdown +Nm from roll start, TD_OFS N (was O) | Where this landing touched down (+ past the roll start); `AUTOLAND_TD_OFS` for the next landing, saved at disarm. |
| Autoland: elevator/throttle input / mode change, TD_OFS not learned | During the final, flare or first 2 s of the rollout you moved the elevator stick, raised the throttle stick above 60 %, or changed mode. Nothing changed. Aileron and rudder corrections, and the throttle stick anywhere from 0 to 60 %, don't stop the learning. |
| Autoland: touchdown Nm off the runway line / +Nm from roll start, TD_OFS not learned | The touchdown was more than 15 m to the side, or implausibly far from the aim. Nothing changed. |
| Autoland: touchdown not seen, TD_OFS not learned | It disarmed before a touchdown was confirmed. Nothing changed. |

---

## 10. Limitations and safety notes

- **It only knows the field from this power-up and this takeoff.** Always power up in
  the pits and take off with TAKEOFF. If you hand-launch, or take off manually, there
  is no touchdown reference.
- **The barometer reads low with throttle and airspeed.** Air coming in through the
  cowling pressurises the whole fuselage. Most of it is prop wash; airspeed adds a
  smaller part. On the test aircraft the baro read about 4 m low at full throttle
  standing still, and about 2 m low at idle on the approach.
  - Foam over the baro chip doesn't fix it, because the pressure is the same
    throughout the fuselage.
  - The fix is `BARO1_THST_SCALE` (learned on every auto takeoff) and
    `BARO1_WCF_FWD`; see [TUNING.md](ama-field/docs/TUNING.md#4-barometer-compensation).
    Thrust compensation works on fixed-wing aircraft in this firmware.
  - Uncompensated, it is a small positive feedback loop in TECS: more throttle,
    the baro reads lower, TECS thinks the aircraft is low and adds more throttle.
    It also makes the takeoff heights happen higher than set, and the flare start
    high and float.
- **Rangefinders: range matters.**
  - **Outdoor range is less than rated.** `RNGFND1_MAX` is only the configured
    limit; the *usable* range outdoors, in sun over grass, is often much shorter.
    The MT-06 gives strong returns only below about 2.5–3 m, so it engages at about
    2.5–3 m: enough to decide a flare at about 2 m on a shallow final
    (`AUTOLAND_DSC_ARC` 90).
  - **A longer-range sensor decides the flare on any final.** A sensor that really
    reaches 4 m or more outdoors (a TF-Luna, rated 8 m at 90% reflectivity and
    2.5 m at 10%, or a lidar or radar) does. Measure your sensor's real range on
    the first flight (`RFSS` / `ALRF` in the log); don't assume it.
  - **Engagement is the same for every sensor:** readings that follow the
    aircraft's descent for 0.2 s, with or without a signal quality (a slow sensor
    without a strong signal needs 6 samples, 0.5 s at 10 Hz).
  - **Below the minimum range:** a sensor whose minimum range is above its height on
    the wheels (the TF-Luna's 0.2 m) is blind on the ground. Touchdown is then
    detected from the range ending at the bottom, with the baro height and the sink
    stopping.
  - **No rangefinder:** expect roughly the same landings; only the touchdown
    detection for `AUTOLAND_RO_GAIN` comes later.
- **The runway direction is only as good as your line-up.** Point the aircraft
  straight down the runway by hand before TAKEOFF; any angle carries into the
  landing, which then comes in across the runway (see
  [Lining up](#3-takeoff-and-the-runway-circle)).
- **Compass interference** from motor current shifts the heading. The lined-up
  heading is sampled with the motor below 20 % for this reason.
- **The fence overshoots** by roughly one turn diameter. Place the fence well inside
  the area you really need to protect.
- **A breach during AUTOLAND abandons the landing.** Keep the circuit inside the
  fence: roughly `AUTOLAND_WP_DIST` + `AUTOLAND_TD_OFS` + circle radius behind
  the roll start (the learning can raise `AUTOLAND_TD_OFS` to 50 m), and up to about
  2 × radius + `AUTOLAND_SIDEGAP` to the side.
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
| `BARO1_THST_SCALE` / `BARO1_WCF_FWD` | 50 / 0.15 | Starting values: the baro read about 4 m low at full throttle on the run-up, and about 2 m low at idle on the approach. |
| `AUTOLAND_TD_OFS` | 20 | The firmware default, then learned. Without baro compensation (v16) it floated 36–48 m past the aim; 25 set by hand landed 14 m short to 23 m long. |
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
