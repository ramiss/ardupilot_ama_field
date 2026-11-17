#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

#if MODE_AUTOLAND_ENABLED

// This is added to the target altitude to make sure the true target is exceeded.
// This should be larger than the expected steady state error.
constexpr float fast_climb_extra_alt = 10;

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
    // @DisplayName: Gap between runway line and loiter-down circle
    // @Description: Extra sideways distance between the extended runway centerline and the near edge of the AUTOLAND base circle. With AUTOLAND_OPTIONS bits 1+2 the TAKEOFF/RTL runway circle keeps this gap plus 10m. 0 keeps the base circle touching the centerline.
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("SIDEGAP", 6, ModeAutoLand, side_gap, 0),

    // @Param: TD_OFS
    // @DisplayName: Touchdown target offset
    // @Description: With AUTOLAND_OPTIONS bit 1, aim the landing this far before (short of) the takeoff roll start point, to allow for the float in the flare so the wheels touch down near the roll start. Negative moves the aim point further down the runway. The whole landing circuit moves with it.
    // @Range: -50 100
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("TD_OFS", 7, ModeAutoLand, td_offset, 0),

    // @Param: DSC_ARC
    // @DisplayName: Descent start on the final turn
    // @Description: With AUTOLAND_OPTIONS bits 1+2, the circuit is flown level and the descent starts this many degrees before the end of the final turn (90 = the last quarter of the circle). The descent is one continuous glide from circuit height over the rest of the turn and the final. 0 = stay level until lined up on final.
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
        const float frac = constrain_float(dist / MAX(target.get_distance(land_start), 10.0f), 0.0f, 1.0f);
        target.offset_up_m(approach_hgt * frac);
        plane.set_target_altitude_location(target);
        return;
    }
    if (stage == AutoLandStage::LOITER && runway_options_enabled() &&
        desc_arc > 0 && plane.reached_loiter_target()) {
        const float rem_deg = final_turn_remaining_deg();
        if (rem_deg <= desc_arc) {
            // glide down from circuit height over the rest of the turn plus
            // the final, by path distance still to go
            const float radius = plane.nav_controller->loiter_radius(cmd_loiter.p1);
            const float final_len = MAX(cmd_land.content.location.get_distance(land_start), 10.0f);
            const float frac = (radius * radians(rem_deg) + final_len) /
                               (radius * radians(float(desc_arc.get())) + final_len);
            Location target = cmd_land.content.location;
            target.offset_up_m(approach_hgt * frac);
            plane.set_target_altitude_location(target);
            return;
        }
    }
    Mode::update_target_altitude();
}

bool ModeAutoLand::wheels_on_ground()
{
    if (!plane.is_flying()) {
        return true;
    }
    bool rf_ground = false;
#if AP_RANGEFINDER_ENABLED
    const auto orient = plane.rangefinder_orientation();
    const int8_t q = plane.rangefinder.signal_quality_pct_orient(orient);
    rf_ground = plane.rangefinder.status_orient(orient) == RangeFinder::Status::Good &&
                plane.rangefinder.distance_orient(orient) < 0.4f &&
                (q < 0 || q >= 50);
#endif
    const uint32_t now = AP_HAL::millis();
    if (!rf_ground) {
        rf_ground_ms = 0;
        return false;
    }
    if (rf_ground_ms == 0) {
        rf_ground_ms = now;
    }
    return now - rf_ground_ms > 300;
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

void ModeAutoLand::navigate()
{
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

    case AutoLandStage::LOITER:
        // check if we have arrived and completed loiter at base leg waypoint
        if (plane.verify_loiter_to_alt(cmd_loiter)) {
            stage = AutoLandStage::LANDING;
            if (runway_options_enabled() && desc_arc > 0) {
                // the planned glide is already part way down at the end of
                // the turn: continue the final from that height
                const float radius = plane.nav_controller->loiter_radius(cmd_loiter.p1);
                const float final_len = MAX(cmd_land.content.location.get_distance(land_start), 10.0f);
                approach_hgt *= final_len / (radius * radians(float(desc_arc.get())) + final_len);
                float tgt_abs;
                if (cmd_land.content.location.get_alt_m(Location::AltFrame::ABSOLUTE, tgt_abs)) {
                    land_start.set_alt_m(tgt_abs + approach_hgt, Location::AltFrame::ABSOLUTE);
                }
            }
            plane.start_command(cmd_land);
            // Crosstrack from the land start location
            plane.prev_WP_loc = land_start;

        }
        break;

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
int8_t ModeAutoLand::pits_side(const Location &line_pt, float heading_deg) const
{
    if (!gps_fix_loc_valid) {
        return 0;
    }
    const float err_deg = wrap_180(heading_deg - degrees(line_pt.get_bearing(gps_fix_loc)));
    const float offset_m = line_pt.get_distance(gps_fix_loc) * sinf(radians(err_deg));
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
    if (pits_side(plane.current_loc, heading) == 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Takeoff blocked: pits side unknown");
        return false;
    }
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


#endif // MODE_AUTOLAND_ENABLED

