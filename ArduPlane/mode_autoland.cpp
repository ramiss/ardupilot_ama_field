#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

#if MODE_AUTOLAND_ENABLED

// This is added to the target altitude to make sure the true target is exceeded.
// This should be larger than the expected steady state error.
constexpr float fast_climb_extra_alt = 10;

// AUTOLAND_TD_OFS default: about the float after the flare, so an untrained
// aircraft lands near the roll start rather than far down the runway. Test
// aircraft (Freewing Pandora, fuselage vented): 25m touches down a few metres
// before the roll start.
#define AUTOLAND_TD_OFS_DEFAULT 25

// the last part of the final turn is flown level at the final's start height
// (AUTOLAND_FNL_HGT), so the aircraft is settled before the final starts
static constexpr float AUTOLAND_LEVEL_ARC_DEG = 60.0f;
// shortest circle descent: AUTOLAND_DSC_ARC is raised to at least this
static constexpr float AUTOLAND_MIN_DSC_ARC_DEG = AUTOLAND_LEVEL_ARC_DEG + 30.0f;
// leave the circle only within this height above the final's start height,
// otherwise fly one more lap
static constexpr float AUTOLAND_EXIT_HIGH_M = 2.5f;
// the base circle ends AUTOLAND_SIDEGAP to the far side of the runway line; on the
// final, track the line with this fraction of NAVL1_PERIOD so the aircraft is on
// the centreline early instead of converging over most of the final
static constexpr float AUTOLAND_FINAL_L1_SCALE = 0.6f;

/*
  mode AutoLand parameters
 */
const AP_Param::GroupInfo ModeAutoLand::var_info[] = {
    // @Param: WP_ALT
    // @DisplayName: Final approach WP altitude
    // @Description: This is the target altitude above HOME for final approach waypoint
    // @Range: 0 200
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_ALT", 1, ModeAutoLand, final_wp_alt, 55),

    // @Param: WP_DIST
    // @DisplayName: Final approach WP distance
    // @Description: This is the distance from Home that the final approach waypoint is set. The waypoint point will be in the opposite direction of takeoff (the direction the plane is facing when the plane sets its takeoff heading)
    // @Range: 0 700
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WP_DIST", 2, ModeAutoLand, final_wp_dist, 400),

    // @Param: DIR_OFF
    // @DisplayName: Landing direction offset from takeoff
    // @Description: The captured takeoff direction after ground course is established in autotakeoffsis offset by this amount to create a different landing direction and approach.However,if TKOFF_OPTION bit1 is set, the takeoff(landing) direction is captured immediately via compass heading upon arming, then this offset is NOT applied.
    // @Range: -360 360
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("DIR_OFF", 3, ModeAutoLand, landing_dir_off, 0),

    // @Param: OPTIONS
    // @DisplayName: Autoland mode options
    // @Description: Enables optional autoland mode behaviors
    // @Bitmask: 0: When set if there is a healthy compass in use the compass heading will be captured at arming and used for the AUTOLAND mode's initial takeoff direction instead of capturing ground course in NAV_TAKEOFF or Mode TAKEOFF or other modes., 1: Land at the takeoff roll start point instead of HOME. The last point where the armed aircraft was stationary before taking off becomes the landing target, and the landing direction is the bearing from that point to the position 3m above it after liftoff. Overrides bit 0. If the roll veers more than 8 degrees from the heading the aircraft was lined up on (sampled with the motor at idle), the lined-up compass heading is used instead. If the roll is shorter than 15m or longer than 250m, nothing is captured and AUTOLAND is unavailable for that flight., 2: Build the approach circuit (loiter-down circle and turn onto final) on the side of the runway line away from the pits, regardless of where the aircraft is when AUTOLAND starts. The pits are the first good GPS fix point after power-up, so power up behind the flight line. With bits 1 and 2 both set: arming is blocked until that point is saved, ground takeoffs in TAKEOFF mode are blocked if the pits side can't be determined, and TAKEOFF mode's loiter, RTL (when no rally point is in use) and Guided fence returns circle abeam the takeoff point, with the circle's near edge AUTOLAND_SIDEGAP+10m from the runway centerline on the side away from the pits (radius WP_LOITER_RAD, or larger if speed, wind and ROLL_LIMIT_DEG prevent flying that tight). See README.md for the full list of behaviours enabled by bits 1+2.
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 4, ModeAutoLand, options, 0),

    // @Param: CLIMB
    // @DisplayName: Minimum altitude above terrain before turning upon entry
    // @Description: Vehicle will climb with limited turn ability (LEVEL_ROLL_LIMIT) until it is at least this altitude above the terrain at the point of entry, before proceeding to loiter-to-alt and landing legs. 0 Disables.
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("CLIMB", 5, ModeAutoLand, terrain_alt_min, 0),

    // @Param: SIDEGAP
    // @DisplayName: Buffer from the runway line to the circles
    // @Description: Buffer between the extended runway centreline and the near edge of the field mode's circles, on the side away from the pits. The AUTOLAND base circle is this far out, so the turn onto final ends there and can't overshoot toward the pits; the final then tracks onto the centreline. With AUTOLAND_OPTIONS bits 1+2 the runway circle (TAKEOFF loiter, RTL without a rally point, fence return) is this plus 10m out (at least 15m). Other loiters (LOITER mode, mission and GUIDED loiters) are not affected. 0 puts the base circle on the centreline and lets the turn overshoot toward the pits; 5 is typical.
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("SIDEGAP", 6, ModeAutoLand, side_gap, 0),

    // @Param: TD_OFS
    // @DisplayName: Touchdown target offset
    // @Description: Aim the landing this far short of the roll start to absorb the float after the flare. Needs AUTOLAND_OPTIONS bit 1. Negative moves the aim point further down the runway. The whole landing circuit moves with it. Each landing reports where it touched down from the roll start ("Autoland: touchdown +4m from roll start") and the value that would have put that landing on the roll start ("Autoland: suggest AUTOLAND_TD_OFS 29 (now 25)"): this value plus the reported distance. Set it from a few landings without elevator or throttle input. The default, 25m, is a typical float; 0 aims at the roll start itself.
    // @Range: -50 100
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("TD_OFS", 7, ModeAutoLand, td_offset, AUTOLAND_TD_OFS_DEFAULT),

    // @Param: DSC_ARC
    // @DisplayName: Descent start on the final turn
    // @Description: With AUTOLAND_OPTIONS bits 1+2, the circuit is flown level and the descent starts this many degrees before the end of the final turn. With AUTOLAND_FNL_HGT set, the circle descent is one even sink rate down to AUTOLAND_FNL_HGT, reached 60 degrees before the end of the turn; the last 60 degrees are flown level, then the final glides from there to the touchdown aim. This is where the aircraft itself starts descending. From downwind the turn onto final is about 180 degrees, so 180 starts the descent as that turn begins: about 1.8 m/s from a 21m circuit to 8m on a 54m circle at 16 m/s. The higher the circuit and the smaller this angle, the faster the sink; on a 20m circuit below about 150 the arc is too short ("Autoland: descent N m/s, raise AUTOLAND_DSC_ARC") and it flies another lap. Established on the circle past the start point, it starts there if at least 60% of the descent arc is left, so a value above the arrival turn (e.g. 240 on a 180 degree arrival) starts where the aircraft joins the circle. Raised to at least 90. With AUTOLAND_FNL_HGT 0: one continuous glide from circuit height over the rest of the turn and the final. 0 = stay level until lined up on final.
    // @Range: 0 270
    // @Increment: 5
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("DSC_ARC", 8, ModeAutoLand, desc_arc, 0),

    // @Param: RO_GAIN
    // @DisplayName: Rollout steering gain
    // @Description: With AUTOLAND_OPTIONS bits 1+2, multiplies the rudder/tailwheel steering from touchdown until stopped, to hold the runway heading on the landing roll. 1 = normal ground steering.
    // @Range: 1 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RO_GAIN", 9, ModeAutoLand, rollout_gain, 1.0f),

    // indexes 10 and 11 were BAROLRN and TDLRN; don't reuse them

    // @Param: FNL_HGT
    // @DisplayName: Height at the start of the final
    // @Description: With AUTOLAND_OPTIONS bits 1+2 and AUTOLAND_DSC_ARC set, the circle descent ends at this height above the touchdown aim, reached 60 degrees before the end of the final turn and held level to the final; the final then glides from it to the aim over AUTOLAND_WP_DIST (8m over 85m is about 5.4 degrees, 1.5 m/s at 16 m/s). A gentle final needs little flare. Low finals fly low over the ground before the aim point: keep it clear. 0 = the final start height follows from the circle geometry (one continuous glide).
    // @Range: 0 50
    // @Increment: 0.5
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("FNL_HGT", 12, ModeAutoLand, final_hgt, 8),

    AP_GROUPEND
};

ModeAutoLand::ModeAutoLand() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeAutoLand::_enter()
{
    //must be flying to enter
    if (!plane.is_flying()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Must already be flying!");
        return false;
    }

    // autoland not available for quadplanes as capture of takeoff direction
    // doesn't make sense
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "autoland not available");
        return false;
    }
#endif

    if (!plane.takeoff_state.initial_direction.initialized) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff initial direction not set");
        return false;
    }

    plane.set_target_altitude_current();
    plane.auto_state.next_wp_crosstrack = true;

    plane.prev_WP_loc = plane.current_loc;

    // In flight stage normal for approach
    plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);

    // landing target: HOME, or the captured takeoff roll start point
    const bool use_roll_start = autoland_option_is_set(AutoLandOption::LAND_AT_ROLL_START);
    if (use_roll_start && !land_point_valid) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Autoland: no takeoff point captured");
        return false;
    }
    Location home = use_roll_start ? land_point : ahrs.get_home();
    if (use_roll_start && td_offset != 0) {
        // aim short of the roll start by AUTOLAND_TD_OFS to allow for the flare float
        home.offset_bearing(plane.takeoff_state.initial_direction.heading, -float(td_offset.get()));
    }
    // measure and report where this landing touches down
    td_lrn.state = use_roll_start ? TdLearnState::WATCH : TdLearnState::IDLE;
    td_lrn.aim_ofs = td_offset;
    td_lrn.pilot_input = false;
    td_lrn.spike_ms = 0;
    td_lrn.still_loc_pending = true;

    // circle descent (AUTOLAND_DSC_ARC / AUTOLAND_FNL_HGT)
    circle_dsc.armed = false;
    circle_dsc.start_deg = 0;
    circle_dsc.start_hgt = 0;
    circle_dsc.started = false;
    circle_dsc.lowest = 0;
    circle_dsc.extra_laps = 0;
    circle_dsc.block_exit = false;
    circle_dsc.final_frac_max = 1.0f;
    circle_dsc.tconst = 0;
    IGNORE_RETURN(AP_Param::get("TECS_HDEM_TCONST", circle_dsc.tconst));

#ifndef HAL_LANDING_DEEPSTALL_ENABLED
    if (plane.landing.get_type() == AP_Landing::TYPE_DEEPSTALL) {
        // Deep stall landings require only a landing location, they do there own loiter to alt and approach
        cmd_land.id = MAV_CMD_NAV_LAND;
        cmd_land.content.location = home;

        // p1 gives the altitude from which to start the deep stall above the location alt
        cmd_land.p1 = final_wp_alt;
        plane.start_command(cmd_land);

        stage = AutoLandStage::LANDING;
        return true;
    }
#endif // HAL_LANDING_DEEPSTALL_ENABLED

    /*
      Glide slope landing is in 3 steps:
        1) a loitering to alt waypoint centered on base leg
        2) exiting and proceeeing to a final approach land start WP, with crosstrack
        3) the landing WP at home, with crosstrack

      the base leg point is 90 degrees off from the landing leg
     */

    /*
      first calculate the starting waypoint we will use when doing the
      NAV_LAND. This is offset by final_wp_dist from home, in a
      direction 180 degrees from takeoff direction
     */
    land_start = home;
    land_start.offset_bearing(plane.takeoff_state.initial_direction.heading, -final_wp_dist);
    float land_target_alt_m;
    approach_hgt = final_wp_alt;
    if (use_roll_start && home.get_alt_m(Location::AltFrame::ABSOLUTE, land_target_alt_m)) {
        // final approach altitude is relative to the landing target, not HOME
        float cur_abs_m;
        if (runway_options_enabled() && plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, cur_abs_m)) {
            // fly the whole circuit at the altitude AUTOLAND was selected at
            // (never below AUTOLAND_WP_ALT) and only descend once turned onto
            // final. Capped at 0.25 x AUTOLAND_WP_DIST (a ~16 degree final after the flare aim point).
            const float max_hgt = MAX(float(final_wp_alt.get()), final_wp_dist * 0.25f);
            approach_hgt = constrain_float(cur_abs_m - land_target_alt_m, float(final_wp_alt.get()), max_hgt);
            gcs().send_text(MAV_SEVERITY_INFO, "Autoland: circuit at %.0fm, descend on final", approach_hgt);
        }
        land_start.set_alt_m(land_target_alt_m + approach_hgt, Location::AltFrame::ABSOLUTE);
    } else {
        land_start.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_HOME);
        land_start.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    /*
      now create the initial target waypoint for the loitering to alt centered on base leg waypoint. We
      choose if we will do a right or left turn onto the landing based
      on where we are when we enter the landing mode
     */
    const float bearing_to_current_deg = degrees(land_start.get_bearing(plane.current_loc));
    float bearing_err_deg = wrap_180(plane.takeoff_state.initial_direction.heading - bearing_to_current_deg);

    if (autoland_option_is_set(AutoLandOption::CIRCUIT_AWAY_FROM_HOME)) {
        // put the circuit on the side of the runway line opposite the pits
        // (HOME, or the first good GPS fix point if HOME is on the runway)
        const int8_t pits = pits_side(land_start, plane.takeoff_state.initial_direction.heading);
        if (pits != 0) {
            // bearing_err > 0 means circuit on the left; pits right (+1) -> circuit left
            bearing_err_deg = (pits > 0) ? 1.0f : -1.0f;
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Autoland: pits side unknown");
        }
    }
    const float bearing_offset_deg = (bearing_err_deg > 0) ? -90 : 90;
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland: circuit on %s side", (bearing_err_deg > 0) ? "left" : "right");

    // Try and minimize loiter radius by using the smaller of the waypoint loiter radius or 1/3 of the final WP distance
    float loiter_radius = MIN(final_wp_dist * 0.333, abs(plane.aparm.loiter_radius));
    if (runway_options_enabled()) {
        // command a base circle the aircraft can really fly: it comes down the
        // circle ~1 m/s faster than cruise, and on the downwind side the wind
        // adds to that. A circle that is too tight saturates the roll limit,
        // the aircraft swings wide and overshoots the turn onto final.
        // capped at 60m to keep the circuit inside a small-field fence
        loiter_radius = MAX(loiter_radius, MIN(flown_circle_radius(loiter_radius, 1.0f), 60.0f));
    }

    // corrected_loiter_radius is the radius the vehicle will actually fly, this gets larger as altitude increases.
    // Strictly this gets the loiter radius at the current altitude, really we want the loiter radius at final_wp_alt.
    const float corrected_loiter_radius = plane.nav_controller->loiter_radius(loiter_radius);

    cmd_loiter.id = MAV_CMD_NAV_LOITER_TO_ALT;
    cmd_loiter.p1 = loiter_radius;
    cmd_loiter.content.location = land_start;
    cmd_loiter.content.location.offset_bearing(plane.takeoff_state.initial_direction.heading + bearing_offset_deg,
                                               corrected_loiter_radius + MAX(side_gap.get(), 0));
    cmd_loiter.content.location.loiter_ccw = bearing_err_deg>0? 1 :0;

    // May need to climb first
    bool climb_first = false;
    if (terrain_alt_min > 0) {
        // Work out the distance needed to climb above terrain
#if AP_TERRAIN_AVAILABLE
        const bool use_terrain = plane.terrain_enabled_in_current_mode();
#else
        const bool use_terrain = false;
#endif
        const float dist_to_climb = terrain_alt_min - plane.relative_ground_altitude(RangeFinderUse::CLIMB, use_terrain);
        if (is_positive(dist_to_climb)) {
            // Copy loiter and update target altitude to current altitude plus climb altitude
            cmd_climb = cmd_loiter;
            float abs_alt;
            if (plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, abs_alt)) {
                cmd_climb.content.location.set_alt_m(abs_alt + dist_to_climb + fast_climb_extra_alt, Location::AltFrame::ABSOLUTE);
                climb_first = true;
            }
        }
    }

#if AP_TERRAIN_AVAILABLE
    // Update loiter location to be relative terrain if enabled
    if (plane.terrain_enabled_in_current_mode()) {
        cmd_loiter.content.location.set_alt_m(final_wp_alt, Location::AltFrame::ABOVE_TERRAIN);
    };
#endif
    // land WP at home
    cmd_land.id = MAV_CMD_NAV_LAND;
    cmd_land.content.location = home;

    // runway options: hold the current altitude on the way to the base
    // circle, and only descend to AUTOLAND_WP_ALT once on the circle
    bool hold_first = false;
    if (!climb_first && runway_options_enabled()) {
        float cur_abs_m, loiter_abs_m;
        if (plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, cur_abs_m) &&
            cmd_loiter.content.location.get_alt_m(Location::AltFrame::ABSOLUTE, loiter_abs_m) &&
            cur_abs_m > loiter_abs_m + 5.0f) {
            cmd_climb = cmd_loiter;
            cmd_climb.content.location.set_alt_m(cur_abs_m, Location::AltFrame::ABSOLUTE);
            hold_first = true;
        }
    }

    // start first leg toward the base leg loiter to alt point
    if (climb_first) {
        stage = AutoLandStage::CLIMB;
        plane.start_command(cmd_climb);

    } else if (hold_first) {
        stage = AutoLandStage::TRANSIT;
        plane.start_command(cmd_climb);
        plane.prev_WP_loc = plane.current_loc;

    } else {
        stage = AutoLandStage::LOITER;
        plane.start_command(cmd_loiter);
    }

    return true;
}

void ModeAutoLand::update()
{
    plane.calc_nav_roll();

    // Apply level roll limit in climb stage
    if (stage == AutoLandStage::CLIMB) {
        plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
        plane.nav_roll_cd = constrain_int16(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    }

    plane.calc_nav_pitch();
    if (plane.landing.is_throttle_suppressed()) {
        // if landing is considered complete throttle is never allowed, regardless of landing type
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    } else {
        plane.calc_throttle();
    }
}

/*
  Between leaving the base circle and the landing code switching to its
  APPROACH stage (lined up on the runway line), stock ArduPilot targets the
  touchdown altitude directly: the drop from AUTOLAND_WP_ALT is less than
  ALT_SLOPE_MIN, so no altitude slope is set up, and TECS dives at
  TECS_SINK_MAX. Hold the approach start altitude until the glide slope
  takes over.
 */
void ModeAutoLand::update_target_altitude()
{
    if (stage == AutoLandStage::LANDING &&
        !plane.landing.is_on_approach() &&
        !plane.landing.is_flaring()) {
        // follow a straight glide path from AUTOLAND_WP_ALT at AUTOLAND_WP_DIST
        // down to the touchdown point, by distance to go. Holds WP_ALT while
        // still further out than WP_DIST, and keeps descending normally if
        // lining up on the runway takes a while (crosswind)
        Location target = cmd_land.content.location;
        const float dist = target.get_distance(plane.current_loc);
        const float frac = constrain_float(dist / MAX(target.get_distance(land_start), 10.0f), 0.0f, circle_dsc.final_frac_max);
        target.offset_up_m(approach_hgt * frac);
        plane.set_target_altitude_location(target);
        return;
    }
    if (stage == AutoLandStage::LOITER && runway_options_enabled() &&
        desc_arc > 0 && plane.reached_loiter_target()) {
        float hgt;
        if (circle_descent_height(hgt)) {
            Location target = cmd_land.content.location;
            target.offset_up_m(hgt);
            plane.set_target_altitude_location(target);
            return;
        }
    }
    Mode::update_target_altitude();
}

/*
  target height above the touchdown aim on the base circle (AUTOLAND_DSC_ARC),
  false while still level at circuit height.

  With AUTOLAND_FNL_HGT: one even descent, starting where the aircraft is
  AUTOLAND_DSC_ARC degrees before the end of the final turn, down to
  AUTOLAND_FNL_HGT reached 60 degrees before the end, then level to the final.
  Outside the landing stage TECS lags its height demand by TECS_HDEM_TCONST, so
  the plan is read that far ahead along the circle: the aircraft then flies the
  plan itself and levels off where planned (without it, it trails the plan by
  sink rate x 3 s, and the final starts that far above its glide path). At the
  start the read-ahead target is already part way down; the lag turns that into
  the same even sink from the start point. The descent starts from the height
  the aircraft is at, and never climbs back: a further lap holds the lowest
  target reached.

  Without AUTOLAND_FNL_HGT: one continuous glide from circuit height over the
  rest of the turn and the final, by path distance still to go.
 */
bool ModeAutoLand::circle_descent_height(float &hgt)
{
    const float rem_deg = final_turn_remaining_deg();
    const float radius = MAX(plane.nav_controller->loiter_radius(cmd_loiter.p1), 10.0f);
    const float final_len = MAX(cmd_land.content.location.get_distance(land_start), 10.0f);

    if (!is_positive(final_hgt.get())) {
        if (rem_deg > desc_arc) {
            return false;
        }
        const float frac = (radius * radians(rem_deg) + final_len) /
                           (radius * radians(float(desc_arc.get())) + final_len);
        hgt = approach_hgt * frac;
        return true;
    }

    const float fnl = MIN(float(final_hgt.get()), approach_hgt);
    const float dsc = MAX(float(desc_arc.get()), AUTOLAND_MIN_DSC_ARC_DEG);
    const float gs = MAX(plane.gps.ground_speed(), 5.0f);
    const float lead_deg = MIN(degrees(gs * MAX(circle_dsc.tconst, 0.0f) / radius), 60.0f);
    const float ahead = rem_deg - lead_deg;

    if (!circle_dsc.started) {
        // only on the circle and turning its way: the degrees still to go mean
        // nothing while joining it (often an S-turn)
        const float yaw_rate = degrees(plane.ahrs.get_gyro().z);
        const bool turning = cmd_loiter.content.location.loiter_ccw ? yaw_rate < -5.0f : yaw_rate > 5.0f;
        const float circle_err = fabsf(plane.current_loc.get_distance(cmd_loiter.content.location) - radius);
        if (!turning || circle_err > MAX(15.0f, 0.25f * radius)) {
            return false;
        }
        // selected above circuit height: the stock loiter spirals down to it on this
        // circle first; the profile only starts from circuit height, so a high entry
        // doesn't spiral straight down to AUTOLAND_FNL_HGT and circle there
        float cur_abs, tgt_abs;
        const bool have_hgt = plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, cur_abs) &&
                              cmd_land.content.location.get_alt_m(Location::AltFrame::ABSOLUTE, tgt_abs);
        if (have_hgt && cur_abs - tgt_abs > approach_hgt + 3.0f) {
            return false;
        }
        if (rem_deg >= dsc - 5.0f) {
            // before the descent start point: passing it starts the descent
            circle_dsc.armed = true;
            circle_dsc.start_deg = dsc;
        } else if (!circle_dsc.armed && rem_deg - AUTOLAND_LEVEL_ARC_DEG >= 0.6f * (dsc - AUTOLAND_LEVEL_ARC_DEG)) {
            // established on the circle past the start point: start here if at least
            // 60% of the descent arc is left (at most 1.7x the planned sink rate),
            // else one more lap
            circle_dsc.armed = true;
            circle_dsc.start_deg = rem_deg;
        }
        if (!circle_dsc.armed || rem_deg > circle_dsc.start_deg) {
            return false;
        }
        circle_dsc.start_hgt = have_hgt ? constrain_float(cur_abs - tgt_abs, fnl, approach_hgt) : approach_hgt;
        circle_dsc.started = true;
        circle_dsc.lowest = circle_dsc.start_hgt;
        const float secs = radius * radians(circle_dsc.start_deg - AUTOLAND_LEVEL_ARC_DEG) / gs;
        const float sink = (circle_dsc.start_hgt - fnl) / MAX(secs, 1.0f);
        if (sink > 2.5f) {
            // the descent arc is too short for the height to lose
            gcs().send_text(MAV_SEVERITY_INFO, "Autoland: descent %.1fm/s, raise AUTOLAND_DSC_ARC", sink);
        }
    }

    float h;
    if (ahead >= circle_dsc.start_deg) {
        h = circle_dsc.start_hgt;
    } else if (ahead >= AUTOLAND_LEVEL_ARC_DEG) {
        h = fnl + (circle_dsc.start_hgt - fnl) * (ahead - AUTOLAND_LEVEL_ARC_DEG) /
                  MAX(circle_dsc.start_deg - AUTOLAND_LEVEL_ARC_DEG, 1.0f);
    } else if (ahead >= 0) {
        h = fnl;
    } else {
        // looking ahead onto the final
        h = MAX(fnl * (1.0f - radius * radians(-ahead) / final_len), 0.0f);
    }
    // circle part only (not below AUTOLAND_FNL_HGT), latched so it never climbs back
    circle_dsc.lowest = MIN(circle_dsc.lowest, MAX(h, fnl));
    // on a further lap, hold the circle height until clear of the exit (the
    // degrees to go read near 0 just past it, which would look onto the final)
    hgt = circle_dsc.block_exit ? circle_dsc.lowest : MIN(circle_dsc.lowest, h);
    return true;
}

void ModeAutoLand::touchdown_reset(void)
{
    tdet.tier = TdTier::NONE;
    tdet.cand = TdTier::NONE;
    tdet.cand_ms = 0;
    tdet.seen_good = false;
    tdet.last_was_good = false;
    tdet.blind_ok = false;
    tdet.last_good_ms = 0;
    tdet.last_good_dist = 0;
}

/*
  touchdown detection on an AUTOLAND final, sensor agnostic: only the
  configured RNGFNDx_MIN and RNGFNDx_GNDCLR, the rangefinder status, the
  landing height and the EKF vertical speed. First tier whose condition holds
  long enough wins:
  1. ground visible: Good readings at or below GNDCLR + max(0.15 m, 10% of
     GNDCLR), for 0.25 s; only GNDCLR + 0.05 m if GNDCLR is less than 0.05 m above
     RNGFNDx_MIN (a sensor that can't see the ground never reads that low)
  2. blind zone (the sensor can't read its ground clearance, or stops giving
     Good readings at the bottom): the last Good reading was at or below
     RNGFNDx_MIN + 0.3 m and at most 0.5 s before the range ended; the status
     is still not Good; and corroborated by the landing height (compensated
     baro plus the held rangefinder correction) under 1.5 m and the sink
     stopped (EKF vertical speed under 0.3 m/s), for 0.3 s. A dropout at
     altitude fails the height check, a dropout higher up the range fails the
     minimum-range check.
  3. no usable rangefinder (none, RNGFND_LANDING off, or nothing usable on
     this final): is_flying() false
  A climb at more than 0.7 m/s (bounce) un-detects it.
 */
void ModeAutoLand::touchdown_update(void)
{
    if (!hal.util->get_soft_armed() || plane.control_mode != &plane.mode_autoland ||
        stage != AutoLandStage::LANDING) {
        touchdown_reset();
        return;
    }
    const uint32_t now = AP_HAL::millis();
    Vector3f vel;
    const float vz = plane.ahrs.get_velocity_NED(vel) ? vel.z : 0.0f;

    if (tdet.tier != TdTier::NONE) {
        if (vz < -0.7f) {
            // bounced: airborne again
            tdet.tier = TdTier::NONE;
            tdet.cand = TdTier::NONE;
        }
        return;
    }

    TdTier cond = TdTier::NONE;
    float dist = 0;
#if AP_RANGEFINDER_ENABLED
    const auto orient = plane.rangefinder_orientation();
    if (plane.rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) && plane.rangefinder.has_orientation(orient)) {
        const bool good = plane.rangefinder.status_orient(orient) == RangeFinder::Status::Good;
        const float rmin = plane.rangefinder.min_distance_orient(orient);
        const float clear = plane.rangefinder.ground_clearance_orient(orient);
        dist = plane.rangefinder.distance_orient(orient);
        if (good) {
            tdet.seen_good = true;
            tdet.last_good_ms = now;
            tdet.last_good_dist = dist;
            tdet.blind_ok = false;
        } else if (tdet.last_was_good) {
            // the range just ended: at the bottom of the sensor's range?
            tdet.blind_ok = now - tdet.last_good_ms <= 500 && tdet.last_good_dist <= rmin + 0.3f;
        }
        tdet.last_was_good = good;

        // GNDCLR within 0.05 m of the minimum range: only readings right at the
        // ground clearance, so readings just above the minimum don't count
        const float band = (clear >= rmin + 0.05f) ? MAX(0.15f, 0.1f * clear) : 0.05f;
        if (good && dist <= clear + band) {
            cond = TdTier::GROUND_VISIBLE;
        } else if (!good && tdet.blind_ok) {
            bool rf_active;
            const float landing_hgt = plane.get_landing_height(rf_active);
            if (landing_hgt < 1.5f && fabsf(vz) < 0.3f) {
                cond = TdTier::BLIND_ZONE;
            }
        }
    }
#endif
    if (cond == TdTier::NONE && !plane.is_flying()) {
        cond = TdTier::NOT_FLYING;
    }

    if (cond != tdet.cand) {
        tdet.cand = cond;
        tdet.cand_ms = now;
        tdet.cand_loc = plane.current_loc;
    }
    const uint32_t hold_ms = (cond == TdTier::GROUND_VISIBLE) ? 250 : (cond == TdTier::BLIND_ZONE) ? 300 : 0;
    if (cond == TdTier::NONE || now - tdet.cand_ms < hold_ms) {
        return;
    }
    tdet.tier = cond;
    tdet.loc = tdet.cand_loc;
#if HAL_LOGGING_ENABLED
    bool rf_active;
    // @LoggerMessage: TDET
    // @Description: AUTOLAND touchdown detected (rollout steering, touchdown report)
    // @Field: TimeUS: Time since system startup
    // @Field: Tier: 1 rangefinder sees the ground, 2 rangefinder blind zone, 3 not flying (no usable rangefinder)
    // @Field: Dly: time from the condition starting to the detection
    // @Field: LH: landing height (compensated baro and held rangefinder correction)
    // @Field: VZ: EKF vertical speed, down positive
    // @Field: D: rangefinder distance now
    // @Field: GD: last Good rangefinder distance
    // @Field: GAge: time since the last Good rangefinder reading
    AP::logger().WriteCritical("TDET", "TimeUS,Tier,Dly,LH,VZ,D,GD,GAge", "s-smnmms", "F-C00000", "QBIfffff",
                       AP_HAL::micros64(), uint8_t(cond), now - tdet.cand_ms,
                       plane.get_landing_height(rf_active), vz, dist, tdet.last_good_dist,
                       tdet.seen_good ? (now - tdet.last_good_ms) * 0.001f : -1.0f);
#endif
}

/*
  information only, at arming: what the rangefinder can and can't do on this
  setup (AUTOLAND_OPTIONS bits 1+2 and RNGFND_LANDING)
 */
void ModeAutoLand::rangefinder_config_check(void)
{
#if AP_RANGEFINDER_ENABLED
    const auto orient = plane.rangefinder_orientation();
    if (!runway_options_enabled() || !plane.rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) ||
        !plane.rangefinder.has_orientation(orient)) {
        return;
    }
    if (plane.rangefinder.ground_clearance_orient(orient) < plane.rangefinder.min_distance_orient(orient)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder can't see ground on wheels: blind-zone touchdown");
    }
    float flare_alt = 0;
    if (AP_Param::get("LAND_FLARE_ALT", flare_alt) &&
        plane.rangefinder.max_distance_orient(orient) < flare_alt + 1.0f) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder range below flare height: flare decided by baro");
    }
#endif
}

/*
  degrees the ground track still has to turn, in the loiter direction,
  before it is lined up with the runway. Just past lined up counts as 0.
 */
float ModeAutoLand::final_turn_remaining_deg() const
{
    const float course = plane.gps.ground_course();
    const float runway = plane.takeoff_state.initial_direction.heading;
    float rem = cmd_loiter.content.location.loiter_ccw ? wrap_360(course - runway)
                                                       : wrap_360(runway - course);
    if (rem > 330.0f) {
        rem = 0;
    }
    return rem;
}

void ModeAutoLand::_exit()
{
    plane.L1_controller.set_period_scale(1.0f);
}

void ModeAutoLand::navigate()
{
    // tighter line tracking on the final, until the flare (see AUTOLAND_FINAL_L1_SCALE)
    plane.L1_controller.set_period_scale((stage == AutoLandStage::LANDING && runway_options_enabled() &&
                                          !plane.landing.is_flaring()) ? AUTOLAND_FINAL_L1_SCALE : 1.0f);

    switch (stage) {
    case AutoLandStage::CLIMB:
        // Update loiter, although roll limit is applied the vehicle will still navigate (slowly)
        plane.update_loiter(cmd_climb.p1);

        ftype dist;
        if (plane.reached_loiter_target() || !cmd_climb.content.location.get_height_above(plane.current_loc, dist) || (dist < fast_climb_extra_alt)) {
            // Reached destination or Climb is done, move onto loiter
            plane.auto_state.next_wp_crosstrack = true;
            stage = AutoLandStage::LOITER;
            plane.start_command(cmd_loiter);
            plane.prev_WP_loc = plane.current_loc;
        }
        break;

    case AutoLandStage::TRANSIT:
        // holding altitude until the base circle is reached
        plane.update_loiter(cmd_climb.p1);
        if (plane.reached_loiter_target()) {
            stage = AutoLandStage::LOITER;
            plane.start_command(cmd_loiter);
            gcs().send_text(MAV_SEVERITY_INFO, "Autoland: on circle, descending");
        }
        break;

    case AutoLandStage::LOITER: {
        // check if we have arrived and completed loiter at base leg waypoint
        const bool ready = plane.verify_loiter_to_alt(cmd_loiter);
        const bool fnl_mode = runway_options_enabled() && desc_arc > 0 && is_positive(final_hgt.get());
        if (circle_dsc.block_exit) {
            // flying another lap: wait until well away from the exit
            const float rem = final_turn_remaining_deg();
            if (rem > 90.0f && rem < 330.0f) {
                circle_dsc.block_exit = false;
            }
            break;
        }
        if (!ready) {
            break;
        }
        if (fnl_mode && circle_dsc.extra_laps < 2) {
            // don't start the final high: that forces a dive to catch the glide path
            const float fnl = MIN(float(final_hgt.get()), approach_hgt);
            float cur_abs, tgt_abs;
            if (plane.current_loc.get_alt_m(Location::AltFrame::ABSOLUTE, cur_abs) &&
                cmd_land.content.location.get_alt_m(Location::AltFrame::ABSOLUTE, tgt_abs) &&
                cur_abs - tgt_abs > fnl + AUTOLAND_EXIT_HIGH_M) {
                circle_dsc.extra_laps++;
                circle_dsc.block_exit = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Autoland: %.0fm high on the final turn, another lap",
                                cur_abs - tgt_abs - fnl);
                break;
            }
        }
        stage = AutoLandStage::LANDING;
        if (runway_options_enabled() && desc_arc > 0) {
            if (fnl_mode) {
                // the final glides from AUTOLAND_FNL_HGT where the aircraft leaves the
                // circle (usually a little before the final approach point) to the aim:
                // land_start's height is put on that line
                const float final_len = MAX(cmd_land.content.location.get_distance(land_start), 10.0f);
                const float exit_dist = MAX(plane.current_loc.get_distance(cmd_land.content.location), final_len);
                circle_dsc.final_frac_max = MIN(exit_dist / final_len, 2.0f);
                approach_hgt = MIN(float(final_hgt.get()), approach_hgt) / circle_dsc.final_frac_max;
            } else {
                // the planned glide is already part way down at the end of
                // the turn: continue the final from that height
                const float radius = plane.nav_controller->loiter_radius(cmd_loiter.p1);
                const float final_len = MAX(cmd_land.content.location.get_distance(land_start), 10.0f);
                approach_hgt *= final_len / (radius * radians(float(desc_arc.get())) + final_len);
            }
            float tgt_abs;
            if (cmd_land.content.location.get_alt_m(Location::AltFrame::ABSOLUTE, tgt_abs)) {
                land_start.set_alt_m(tgt_abs + approach_hgt, Location::AltFrame::ABSOLUTE);
            }
        }
        plane.start_command(cmd_land);
        // Crosstrack from the land start location
        plane.prev_WP_loc = land_start;
        break;
    }

    case AutoLandStage::LANDING:
        plane.set_flight_stage(AP_FixedWing::FlightStage::LAND);
        plane.verify_command(cmd_land);
        // make sure we line up
        plane.auto_state.crosstrack = true;
        break;
    }
}

/*
  Takeoff direction is initialized after arm when sufficient altitude
  and ground speed is obtained, then captured takeoff direction +
  offset used as landing direction in AUTOLAND
*/
void ModeAutoLand::check_takeoff_direction()
{
#if HAL_QUADPLANE_ENABLED
    // we don't allow fixed wing autoland for quadplanes
    if (quadplane.available()) {
        return;
    }
#endif

    update_gps_fix_point();
    touchdown_update();
    td_learn_update();

    // remember if this arming has flown (blocks a second ground TAKEOFF)
    if (!hal.util->get_soft_armed()) {
        flown_this_arming = false;
    } else if (plane.is_flying() && plane.relative_altitude > 5.0f) {
        flown_this_arming = true;
    }

    if (autoland_option_is_set(AutoLandOption::LAND_AT_ROLL_START)) {
        check_roll_start_capture();
        return;
    }

    if (plane.takeoff_state.initial_direction.initialized) {
        return;
    }
    //set autoland direction to GPS course over ground
    if (plane.control_mode->allows_autoland_direction_capture() &&
        plane.is_flying() &&
        hal.util->get_soft_armed() &&
        plane.gps.ground_speed() > GPS_GND_CRS_MIN_SPD) {
        set_autoland_direction(plane.gps.ground_course() + landing_dir_off);
    }
}

/*
  LAND_AT_ROLL_START: while armed and sitting still on the ground, keep
  recording the position. That last stationary point is the takeoff
  roll start. Once the aircraft is 3m above it, the landing target is
  the roll start and the landing direction is the bearing from the
  roll start to the current position.

  Safety checks: the climb-out point must be 15-250m from the roll
  start and the current GPS track must agree with that bearing within
  25 degrees. Otherwise the roll was curved (e.g. taxi straight into a
  takeoff without stopping) and nothing is captured, so AUTOLAND is
  refused for this flight rather than landing somewhere unintended.
 */
void ModeAutoLand::check_roll_start_capture(void)
{
    if (!hal.util->get_soft_armed()) {
        roll_start_valid = false;
        land_point_valid = false;
        roll_capture_rejected = false;
        return;
    }
    if (land_point_valid || roll_capture_rejected) {
        return;
    }
    if (plane.gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    const float ground_speed = plane.gps.ground_speed();
    if (!plane.is_flying() && ground_speed < 1.0f) {
        // stationary on the ground: candidate roll start
        roll_start_loc = plane.current_loc;
        // line-up heading: only sample it with the motor near idle, the motor
        // current pulls the compass several degrees as the throttle comes up
        if (!roll_start_valid || SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < 20.0f) {
            roll_start_yaw_deg = wrap_360(degrees(plane.ahrs.get_yaw_rad()));
        }
        roll_start_valid = true;
        return;
    }
    if (!roll_start_valid) {
        return;
    }

    ftype height_above_start;
    if (!plane.current_loc.get_height_above(roll_start_loc, height_above_start) ||
        height_above_start < 3.0f) {
        return;
    }

    const float dist = roll_start_loc.get_distance(plane.current_loc);
    const float bearing_deg = wrap_360(degrees(roll_start_loc.get_bearing(plane.current_loc)));
    if (dist < 15.0f || dist > 250.0f) {
        roll_capture_rejected = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "Autoland: no target, roll %.0fm", dist);
        return;
    }

    // runway direction: the roll vector, unless the roll veered away from the
    // heading the aircraft was lined up on, in which case trust the line-up
    float runway_deg = bearing_deg;
    const float veer_deg = wrap_180(bearing_deg - roll_start_yaw_deg);
    if (fabsf(veer_deg) > 8.0f) {
        runway_deg = roll_start_yaw_deg;
        gcs().send_text(MAV_SEVERITY_WARNING, "Autoland: roll veered %.0f, using line-up hdg %.0f",
                        veer_deg, runway_deg);
    }

    land_point = roll_start_loc;
    land_point_valid = true;
    set_autoland_direction(runway_deg + landing_dir_off);
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland: target set %.0fm from home",
                    plane.ahrs.get_home().get_distance(land_point));
}

/*
  runway circle for TAKEOFF loiter and RTL (when no rally point is in use).
  Needs AUTOLAND_OPTIONS bits 1 and 2, a captured takeoff point, and HOME
  more than 5m to one side of the runway line. The circle's near edge is
  5m from the runway centerline on the side away from HOME, abeam the
  takeoff point.
 */
bool ModeAutoLand::get_runway_loiter_center(Location &loc) const
{
    if (!autoland_option_is_set(AutoLandOption::LAND_AT_ROLL_START) ||
        !autoland_option_is_set(AutoLandOption::CIRCUIT_AWAY_FROM_HOME) ||
        !land_point_valid ||
        !plane.takeoff_state.initial_direction.initialized) {
        return false;
    }
    const float heading = plane.takeoff_state.initial_direction.heading;
    const int8_t pits = pits_side(land_point, heading);
    if (pits == 0) {
        // can't tell which side the pits are on
        return false;
    }
    const float radius = flown_circle_radius(fabsf(plane.aparm.loiter_radius));

    // circle on the side opposite the pits. Near edge AUTOLAND_SIDEGAP + 10m
    // from the runway centerline: 10m more than the AUTOLAND circle, because
    // the first lap after takeoff overshoots inward while the wind is learned
    const float side_deg = (pits < 0) ? 90.0f : -90.0f;
    loc = land_point;
    loc.offset_bearing(heading + side_deg, radius + MAX(float(side_gap.get()), 5.0f) + 10.0f);
    last_circle = loc;
    last_circle_valid = true;
    return true;
}

/*
  the circle the aircraft will actually fly: on the downwind side it can't
  turn tighter than (cruise + wind)^2 / (g tan(bank)) at 80% of ROLL_LIMIT_DEG.
  The wind estimate is not trusted below 4 m/s (it is near zero just after
  takeoff) and is capped at 10 m/s.
 */
float ModeAutoLand::flown_circle_radius(float commanded_m, float extra_speed) const
{
    float radius = commanded_m;
    const float bank_rad = radians(0.8f * plane.aparm.roll_limit);
    if (bank_rad > radians(10.0f)) {
        const float wind = constrain_float(plane.ahrs.wind_estimate().xy().length(), 4.0f, 10.0f);
        const float speed = plane.aparm.airspeed_cruise + extra_speed + wind;
        radius = MAX(radius, sq(speed) / (GRAVITY_MSS * tanf(bank_rad)));
    }
    return radius;
}

bool ModeAutoLand::refresh_runway_circle(Location &wp) const
{
    if (!last_circle_valid || wp.get_distance(last_circle) > 2.0f) {
        // wp is not (any longer) the runway circle
        return false;
    }
    Location c;
    if (!get_runway_loiter_center(c)) {
        return false;
    }
    if (c.get_distance(wp) < 5.0f) {
        last_circle = wp;   // keep matching the waypoint actually in use
        return false;
    }
    wp.lat = c.lat;
    wp.lng = c.lng;
    last_circle = wp;
    return true;
}

/*
  which side of a line (through line_pt along heading_deg) the pits are on:
  -1 = left, +1 = right, 0 = unknown. The pits reference is the first good
  GPS fix point since power-up (where the aircraft was switched on, in the
  pits). HOME is deliberately not used: it moves when re-arming on the
  runway or when set from a GCS.
 */
int8_t ModeAutoLand::pits_side(const Location &line_pt, float heading_deg, float *offset_ret) const
{
    if (!gps_fix_loc_valid) {
        return 0;
    }
    const float err_deg = wrap_180(heading_deg - degrees(line_pt.get_bearing(gps_fix_loc)));
    const float offset_m = line_pt.get_distance(gps_fix_loc) * sinf(radians(err_deg));
    if (offset_ret != nullptr) {
        *offset_ret = fabsf(offset_m);
    }
    if (fabsf(offset_m) <= 5.0f) {
        return 0;
    }
    // positive err = pits left of the line
    return (err_deg > 0) ? -1 : 1;
}

/*
  record the first good GPS fix point (3D fix, horizontal accuracy under
  5m or 8+ satellites). Kept until reboot.
 */
void ModeAutoLand::update_gps_fix_point(void)
{
    if (gps_fix_loc_valid || plane.gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }
    float hacc;
    const bool good = plane.gps.horizontal_accuracy(hacc) ? (hacc < 5.0f) : (plane.gps.num_sats() >= 8);
    if (!good) {
        return;
    }
    gps_fix_loc = plane.gps.location();
    gps_fix_loc_valid = true;
    if (runway_options_enabled()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autoland: GPS fix point saved");
    }
}

/*
  TAKEOFF pre-check for AUTOLAND_OPTIONS bits 1+2: on the ground, the
  runway line is the aircraft's position and compass heading. Refuse
  the takeoff if the pits side can't be determined.
 */
bool ModeAutoLand::takeoff_pits_check(void)
{
    if (!autoland_option_is_set(AutoLandOption::LAND_AT_ROLL_START) ||
        !autoland_option_is_set(AutoLandOption::CIRCUIT_AWAY_FROM_HOME) ||
        plane.is_flying()) {
        return true;
    }
    if (!plane.ahrs.use_compass()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Takeoff blocked: no compass heading");
        return false;
    }
    const float heading = wrap_360(plane.ahrs.get_yaw_deg());
    float offset_m = 0;
    const int8_t pits = pits_side(plane.current_loc, heading, &offset_m);
    if (pits == 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Takeoff blocked: pits side unknown");
        return false;
    }
    // confirm the guard: where it thinks the pits are, and where it will circle
    gcs().send_text(MAV_SEVERITY_INFO, "Takeoff: pits %.0fm %s, circle on %s", offset_m,
                    pits < 0 ? "left" : "right", pits < 0 ? "right" : "left");
    return true;
}

// Sets autoland direction using ground course + offset parameter
void ModeAutoLand::set_autoland_direction(const float heading)
{
    plane.takeoff_state.initial_direction.heading = wrap_360(heading);
    plane.takeoff_state.initial_direction.initialized = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Autoland direction= %u",int(plane.takeoff_state.initial_direction.heading));
}

/*
  return true when the LOITER_TO_ALT is lined up ready for the landing, used in commands_logic verify loiter to alt
 */
bool ModeAutoLand::landing_lined_up(void)
{
    // use the line between the center of the LOITER_TO_ALT on the base leg and the
    // start of the landing leg (land_start_WP)
    return plane.mode_loiter.isHeadingLinedUp(cmd_loiter.content.location, cmd_land.content.location);
}

// possibly capture heading on arming for autoland mode if option is set and using a compass
void ModeAutoLand::arm_check(void)
{
    rangefinder_config_check();
    if (autoland_option_is_set(ModeAutoLand::AutoLandOption::LAND_AT_ROLL_START)) {
        // direction comes from the takeoff roll instead
        return;
    }
    if (plane.ahrs.use_compass() && autoland_option_is_set(ModeAutoLand::AutoLandOption::AUTOLAND_DIR_ON_ARM)) {
        set_autoland_direction(plane.ahrs.get_yaw_deg());
    }
}

bool ModeAutoLand::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

/*
  Touchdown report: the landing aims AUTOLAND_TD_OFS short of the roll start.
  Where it really touches down, measured along the runway line from the roll
  start, is reported (and logged, TDLN) so the pilot can adjust AUTOLAND_TD_OFS.
  Nothing is changed automatically: a touchdown point moved by a height error,
  wind or a pilot correction is not the float, and learning from it walks the
  offset away.

  Touchdown, after the flare starts: the rangefinder tiers of touchdown_update()
  (ground visible, or blind zone), at the position where their condition
  started; otherwise the point where the sink stops for good (EKF sink rate
  below 0.3 m/s for 1.5 s), taken at the first impact on the accelerometer
  (0.25 g above 1 g) there if there is one. The baro isn't used. A bounce
  (climbing at 0.7 m/s) waits for the next contact. 2 s down without a bounce
  confirms it.
 */
void ModeAutoLand::td_learn_update(void)
{
    if (!hal.util->get_soft_armed()) {
        if (td_lrn.state == TdLearnState::TOUCHED) {
            td_learn_finish(TdLearnResult::REPORTED);
        } else if (td_lrn.state == TdLearnState::WATCH && stage == AutoLandStage::LANDING) {
            td_learn_finish(TdLearnResult::NOT_SEEN);
        }
        td_lrn.state = TdLearnState::IDLE;
        return;
    }
    if (td_lrn.state != TdLearnState::WATCH && td_lrn.state != TdLearnState::TOUCHED) {
        return;
    }
    if (plane.control_mode != &plane.mode_autoland) {
        if (stage == AutoLandStage::LANDING) {
            td_learn_finish(TdLearnResult::MODE_CHANGE);
        } else {
            // left AUTOLAND before the final
            td_lrn.state = TdLearnState::DONE;
        }
        return;
    }
    if (stage != AutoLandStage::LANDING) {
        return;
    }

    // elevator input, or the throttle stick above 60%, from the start of the final
    // to the confirmed touchdown, flagged in the log (TDLN.PIn). The throttle
    // stick only acts in AUTOLAND above half way (THROTTLE_NUDGE), so 0-60% is
    // ignored. Aileron and rudder corrections only change the direction.
    if (abs(plane.channel_pitch->get_control_in()) > 450 ||
        (plane.g.throttle_nudge && plane.channel_throttle->get_control_in() > 60)) {
        td_lrn.pilot_input = true;
    }

    // vertical acceleration above 1 g (body frame, up positive)
    const float accel_up = -AP::ins().get_accel().z - GRAVITY_MSS;
    // vertical speed, down positive (EKF, GPS aided: not affected by the baro error)
    Vector3f vel;
    const float vz = plane.ahrs.get_velocity_NED(vel) ? vel.z : 0.0f;
    const uint32_t now = AP_HAL::millis();

    if (td_lrn.state == TdLearnState::WATCH) {
        if (!plane.landing.is_flaring() && plane.height_above_target() > 2.0f) {
            // not near the ground yet
            td_lrn.sink_ms = now;
            td_lrn.spike_ms = 0;
            td_lrn.still_loc_pending = true;
            return;
        }
        if (tdet.tier == TdTier::GROUND_VISIBLE || tdet.tier == TdTier::BLIND_ZONE) {
            // the rangefinder tiers of touchdown_update(), at the position the
            // condition started
            td_lrn.state = TdLearnState::TOUCHED;
            td_lrn.touch_ms = now;
            td_lrn.touch_loc = tdet.loc;
            td_lrn.accel = accel_up;
            td_lrn.source = uint8_t(tdet.tier);
            return;
        }
        // without a rangefinder: the touchdown is where the sink stops for good.
        // A flare pull-up also shows on the accelerometer, but the aircraft keeps
        // sinking after it; an impact on the wheels ends the sink.
        if (accel_up > 2.5f && td_lrn.spike_ms == 0) {
            td_lrn.spike_ms = now;
            td_lrn.spike_loc = plane.current_loc;
            td_lrn.spike_accel = accel_up;
        }
        if (vz > 0.3f || vz < -0.7f) {
            // still sinking (or climbing)
            td_lrn.sink_ms = now;
            td_lrn.still_loc_pending = true;
            if (td_lrn.spike_ms != 0 && now - td_lrn.spike_ms > 300) {
                // that spike was not the touchdown
                td_lrn.spike_ms = 0;
            }
            return;
        }
        if (td_lrn.still_loc_pending) {
            td_lrn.still_loc_pending = false;
            td_lrn.still_loc = plane.current_loc;
        }
        if (now - td_lrn.sink_ms > 1500) {
            // down for 1.5 s: touched at the first impact, or where the sink stopped
            const bool spike = td_lrn.spike_ms != 0;
            td_lrn.state = TdLearnState::TOUCHED;
            td_lrn.touch_ms = now;
            td_lrn.touch_loc = spike ? td_lrn.spike_loc : td_lrn.still_loc;
            td_lrn.accel = spike ? td_lrn.spike_accel : accel_up;
            td_lrn.source = 0;
        }
        return;
    }

    // TOUCHED: a bounce means it is flying again, wait for the next contact
    if (vz < -0.7f || (td_lrn.source != 0 && tdet.tier == TdTier::NONE)) {
        td_lrn.state = TdLearnState::WATCH;
        td_lrn.sink_ms = now;
        td_lrn.spike_ms = 0;
        td_lrn.still_loc_pending = true;
        return;
    }
    td_lrn.accel = MAX(td_lrn.accel, accel_up);
    if (now - td_lrn.touch_ms > 2000) {
        td_learn_finish(TdLearnResult::REPORTED);
    }
}

void ModeAutoLand::td_learn_finish(TdLearnResult res)
{
    const bool touched = td_lrn.state == TdLearnState::TOUCHED;
    td_lrn.state = TdLearnState::DONE;

    // touchdown along the runway line from the roll start (positive = past it)
    float along = 0;
    float side = 0;
    if (touched) {
        const Vector2f ne = land_point.get_distance_NE(td_lrn.touch_loc);
        const float hdg = radians(plane.takeoff_state.initial_direction.heading);
        along = ne.x * cosf(hdg) + ne.y * sinf(hdg);
        side = -ne.x * sinf(hdg) + ne.y * cosf(hdg);
    }
    // the float: distance from the aim point to the touchdown
    const float floated = along + td_lrn.aim_ofs;

    switch (res) {
    case TdLearnResult::REPORTED:
        // AUTOLAND_TD_OFS is the pilot's to change; each message within 50 characters.
        // The suggestion moves the aim by the error: + past the roll start raises it
        gcs().send_text(MAV_SEVERITY_INFO, "Autoland: touchdown %+.0fm from roll start", along);
        gcs().send_text(MAV_SEVERITY_INFO, "Autoland: suggest AUTOLAND_TD_OFS %d (now %d)",
                        int(constrain_float(roundf(td_lrn.aim_ofs + along), -50.0f, 100.0f)), td_lrn.aim_ofs);
        break;
    case TdLearnResult::NOT_SEEN:
        gcs().send_text(MAV_SEVERITY_INFO, "Autoland: touchdown not seen");
        break;
    case TdLearnResult::MODE_CHANGE:
        break;
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: TDLN
    // @Description: AUTOLAND: where the landing touched down
    // @Field: TimeUS: Time since system startup
    // @Field: Along: touchdown along the runway line from the roll start, positive past it
    // @Field: Side: touchdown distance from the runway line, positive right
    // @Field: Flt: float, from the aim point to the touchdown
    // @Field: Acc: highest vertical acceleration above 1 g in the 2 s after the touchdown
    // @Field: Ofs: AUTOLAND_TD_OFS the landing was aimed with
    // @Field: Src: touchdown seen by 0 the sink stopping (accelerometer), 1 the rangefinder seeing the ground, 2 the rangefinder blind zone
    // @Field: PIn: 1 if the pilot moved the elevator or raised the throttle stick above 60% on the final
    // @Field: Res: 0 reported, 1 mode change before the touchdown, 2 touchdown not seen
    AP::logger().WriteCritical("TDLN", "TimeUS,Along,Side,Flt,Acc,Ofs,Src,PIn,Res", "smmmom---", "F000000--", "QffffhBBB",
                       AP_HAL::micros64(), along, side, floated, td_lrn.accel, td_lrn.aim_ofs,
                       td_lrn.source, uint8_t(td_lrn.pilot_input), uint8_t(res));
#endif
}


#endif // MODE_AUTOLAND_ENABLED

