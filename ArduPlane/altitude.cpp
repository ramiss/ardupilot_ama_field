/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Plane.h"

/*
  altitude handling routines. These cope with both barometric control
  and terrain following control
 */

/*
  adjust altitude target depending on mode
 */
void Plane::adjust_altitude_target()
{
    control_mode->update_target_altitude();
}

void Plane::check_home_alt_change(void)
{
    int32_t home_alt_cm = ahrs.get_home().alt;
    if (home_alt_cm != auto_state.last_home_alt_cm && hal.util->get_soft_armed()) {
        // cope with home altitude changing
        const int32_t alt_change_cm = home_alt_cm - auto_state.last_home_alt_cm;
        fix_terrain_WP(next_WP_loc, __LINE__);

        // reset TECS to force the field elevation estimate to reset
        TECS_controller.offset_altitude(alt_change_cm * 0.01f);
    }
    auto_state.last_home_alt_cm = home_alt_cm;
}

/*
  setup for a gradual altitude slope to the next waypoint, if appropriate
 */
void Plane::setup_alt_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    TECS_controller.set_path_proportion(auto_state.wp_proportion);
    update_flight_stage();

    /*
      work out if we will gradually change altitude, or try to get to
      the new altitude as quickly as possible.
     */
    switch (control_mode->mode_number()) {
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        /* glide down slowly if above target altitude, but ascend more
           rapidly if below it. See
           https://github.com/ArduPilot/ardupilot/issues/39
        */
        if (above_location_current(next_WP_loc)) {
            set_offset_altitude_location(prev_WP_loc, next_WP_loc);
        } else {
            reset_offset_altitude();
        }
        break;

    case Mode::Number::AUTO:
        // climb without doing slope if option is enabled
        if (!above_location_current(next_WP_loc) && plane.flight_option_enabled(FlightOptions::IMMEDIATE_CLIMB_IN_AUTO)) {
            reset_offset_altitude();
            break;
        }

        // otherwise we set up an altitude slope for this leg
        set_offset_altitude_location(prev_WP_loc, next_WP_loc);

        break;
    default:
        reset_offset_altitude();
        break;
    }
}

/*
  return RTL altitude as AMSL cm
 */
int32_t Plane::get_RTL_altitude_cm() const
{
    if (g.RTL_altitude < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude*100 + home.alt;
}

/*
  return relative altitude in meters (relative to terrain, if available,
  or home otherwise)
 */
float Plane::relative_ground_altitude(enum RangeFinderUse use_rangefinder, bool use_terrain_if_available)
{
#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
   float height_AGL;
   // use external HAGL if available
   if (get_external_HAGL(height_AGL)) {
       return height_AGL;
   }
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

#if AP_RANGEFINDER_ENABLED
   if (rangefinder_use(use_rangefinder) && rangefinder_state.in_range) {
        return rangefinder_state.height_estimate;
   }
#endif

#if HAL_QUADPLANE_ENABLED && AP_RANGEFINDER_ENABLED
   if (rangefinder_use(use_rangefinder) && quadplane.in_vtol_land_final() &&
       rangefinder.status_orient(rangefinder_orientation()) == RangeFinder::Status::OutOfRangeLow) {
       // a special case for quadplane landing when rangefinder goes
       // below minimum. Consider our height above ground to be zero
       return 0;
   }
#endif

#if AP_TERRAIN_AVAILABLE
    float altitude;
    if (use_terrain_if_available &&
        terrain.status() == AP_Terrain::TerrainStatusOK &&
        terrain.height_above_terrain(altitude, true)) {
        return altitude;
    }
#endif

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() &&
        !quadplane.landing_with_fixed_wing_spiral_approach()) {
        // when doing a VTOL landing we can use the waypoint height as
        // ground height. We can't do this if using the
        // LAND_FW_APPROACH as that uses the wp height as the approach
        // height
        return height_above_target();
    }
#endif

    return relative_altitude;
}

/*
  return true if we should use the rangefinder for a specific use case
 */
bool Plane::rangefinder_use(enum RangeFinderUse use_rangefinder) const
{
    const uint8_t use = uint8_t(g.rangefinder_landing.get());
    if (use == uint8_t(RangeFinderUse::NONE)) {
        return false;
    }
    if (use & uint8_t(RangeFinderUse::ALL)) {
        // if ALL bit is set then ignore other bits
        return true;
    }
    return (use & uint8_t(use_rangefinder)) != 0;
}

// Helper for above method using terrain if the vehicle is currently terrain following
float Plane::relative_ground_altitude(enum RangeFinderUse use_rangefinder)
{
#if AP_TERRAIN_AVAILABLE
    return relative_ground_altitude(use_rangefinder, target_altitude.terrain_following);
#else
    return relative_ground_altitude(use_rangefinder, false);
#endif
}


/*
  set the target altitude to the current altitude. This is used when 
  setting up for altitude hold, such as when releasing elevator in
  CRUISE mode.
 */
void Plane::set_target_altitude_current(void)
{
    // record altitude above sea level at the current time as our
    // target altitude
    target_altitude.amsl_cm = current_loc.alt;

    // reset any altitude slope offset
    reset_offset_altitude();

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (terrain_enabled_in_current_mode() && terrain.height_above_terrain(terrain_altitude, true) && !terrain_disabled()) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = terrain_altitude*100;
    } else {
        // if terrain following is disabled, or we don't know our
        // terrain altitude when we set the altitude then don't
        // terrain follow
        target_altitude.terrain_following = false;        
    }
#endif
}

/*
  set target altitude based on a location structure
 */
void Plane::set_target_altitude_location(const Location &loc)
{
    target_altitude.amsl_cm = loc.alt;
    if (loc.relative_alt) {
        target_altitude.amsl_cm += home.alt;
    }
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following_pending) {
        /* we didn't get terrain data to init when we started on this
           target, retry
        */
        setup_terrain_target_alt(next_WP_loc);
    }
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (loc.terrain_alt && terrain.height_above_terrain(height, true)) {
        target_altitude.terrain_following = true;
        target_altitude.terrain_alt_cm = loc.alt;
    } else {
        target_altitude.terrain_following = false;
    }
#endif
}

/*
  return relative to home target altitude in centimeters. Used for
  altitude control libraries
 */
int32_t Plane::relative_target_altitude_cm(void)
{
#if AP_TERRAIN_AVAILABLE
    float relative_home_height;
    if (target_altitude.terrain_following && 
        terrain.height_relative_home_equivalent(target_altitude.terrain_alt_cm*0.01f,
                                                relative_home_height, true)) {
        // add lookahead adjustment the target altitude
        target_altitude.lookahead = lookahead_adjustment();
        relative_home_height += target_altitude.lookahead;

#if AP_RANGEFINDER_ENABLED
        // correct for rangefinder data
        relative_home_height += rangefinder_correction();
#endif

        // we are following terrain, and have terrain data for the
        // current location. Use it.
        return relative_home_height*100;
    }
#endif
    int32_t relative_alt = target_altitude.amsl_cm - home.alt;
    relative_alt += mission_alt_offset()*100;
#if AP_RANGEFINDER_ENABLED
    relative_alt += rangefinder_correction() * 100;
#endif
    return relative_alt;
}

/*
  change the current target altitude by an amount in centimeters. Used
  to cope with changes due to elevator in CRUISE or FBWB
 */
void Plane::change_target_altitude(int32_t change_cm)
{
    target_altitude.amsl_cm += change_cm;
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following && !terrain_disabled()) {
        target_altitude.terrain_alt_cm += change_cm;
    }
#endif
}

/*
  change target altitude by a proportion of the target altitude offset
  (difference in height to next WP from previous WP). proportion
  should be between 0 and 1. 

  When proportion is zero we have reached the destination. When
  proportion is 1 we are at the starting waypoint.

  Note that target_altitude is setup initially based on the
  destination waypoint
 */
void Plane::set_target_altitude_proportion(const Location &loc, float proportion)
{
    set_target_altitude_location(loc);

    // Only do altitude slope handling when above CLIMB_SLOPE_HGT or when
    // descending. This is meant to prevent situations where the aircraft tries
    // to slowly gain height at low altitudes, potentially hitting obstacles.
    if (target_altitude.offset_cm > 0 &&
        (adjusted_relative_altitude_cm() <
         (g2.waypoint_climb_slope_height_min * 100))) {
        // Early return to ensure a full-rate climb past CLIMB_SLOPE_HGT
        return;
    }

    proportion = constrain_float(proportion, 0.0f, 1.0f);
    change_target_altitude(-target_altitude.offset_cm*proportion);

    // rebuild the altitude slope if we are above it and supposed to be climbing
    if (g.alt_slope_max_height > 0) {
        if (target_altitude.offset_cm > 0 && calc_altitude_error_cm() < -100 * g.alt_slope_max_height) {
            set_target_altitude_location(loc);
            set_offset_altitude_location(current_loc, loc);
            change_target_altitude(-target_altitude.offset_cm*proportion);
            // adjust the new target offset altitude to reflect that we are partially already done
            if (proportion > 0.0f)
                target_altitude.offset_cm = ((float)target_altitude.offset_cm)/proportion;
        }
    }
}

#if AP_TERRAIN_AVAILABLE
/*
  change target altitude along a path between two locations
  (prev_WP_loc and next_WP_loc) where the second location is a terrain
  altitude
 */
bool Plane::set_target_altitude_proportion_terrain(void)
{
    if (!next_WP_loc.terrain_alt ||
        !next_WP_loc.relative_alt) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    /*
      we first need to get the height of the terrain at prev_WP_loc
     */
    float prev_WP_height_terrain;
    if (!plane.prev_WP_loc.get_alt_m(Location::AltFrame::ABOVE_TERRAIN,
                                     prev_WP_height_terrain)) {
        return false;
    }
    // and next_WP_loc alt as terrain
    float next_WP_height_terrain;
    if (!plane.next_WP_loc.get_alt_m(Location::AltFrame::ABOVE_TERRAIN,
                                     next_WP_height_terrain)) {
        return false;
    }
    Location loc = next_WP_loc;
    const auto alt = linear_interpolate(prev_WP_height_terrain, next_WP_height_terrain,
                                        plane.auto_state.wp_proportion, 0, 1);

    loc.set_alt_m(alt, Location::AltFrame::ABOVE_TERRAIN);

    set_target_altitude_location(loc);

    return true;
}
#endif // AP_TERRAIN_AVAILABLE

/*
  constrain target altitude to be between two locations. Used to
  ensure we stay within two waypoints in altitude
 */
void Plane::constrain_target_altitude_location(const Location &loc1, const Location &loc2)
{
    if (loc1.alt > loc2.alt) {
        target_altitude.amsl_cm = constrain_int32(target_altitude.amsl_cm, loc2.alt, loc1.alt);
    } else {
        target_altitude.amsl_cm = constrain_int32(target_altitude.amsl_cm, loc1.alt, loc2.alt);
    }
}

/*
  return error between target altitude and current altitude
 */
int32_t Plane::calc_altitude_error_cm(void)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_height;
    if (target_altitude.terrain_following && 
        terrain.height_above_terrain(terrain_height, true)) {
        return target_altitude.lookahead*100 + target_altitude.terrain_alt_cm - (terrain_height*100);
    }
#endif
    return target_altitude.amsl_cm - adjusted_altitude_cm();
}

/*
  check for cruise_alt_floor and fence min/max altitude
 */
void Plane::check_fbwb_altitude(void)
{
    float max_alt_cm = 0.0;
    float min_alt_cm = 0.0;
    bool should_check_max = false;
    bool should_check_min = false;

#if AP_FENCE_ENABLED
    // taking fence max and min altitude (with margin)
    const uint8_t enabled_fences = plane.fence.get_enabled_fences();
    if ((enabled_fences & AC_FENCE_TYPE_ALT_MIN) != 0) {
        min_alt_cm = plane.fence.get_relative_safe_alt_min_m()*100.0;
        should_check_min = true;
    }
    if ((enabled_fences & AC_FENCE_TYPE_ALT_MAX) != 0) {
        max_alt_cm = plane.fence.get_relative_safe_alt_max_m()*100.0;
        should_check_max = true;
    }
#endif

    if (g.cruise_alt_floor > 0) {
        // FBWB min altitude exists
        min_alt_cm = MAX(min_alt_cm, plane.g.cruise_alt_floor*100.0);
        should_check_min = true;
    }

    if (!should_check_min && !should_check_max) {
        return;
    }

//check if terrain following (min and max)
#if AP_TERRAIN_AVAILABLE
    if (target_altitude.terrain_following) {
        // set our target terrain height to be at least the min set
        if (should_check_max) {
            target_altitude.terrain_alt_cm = MIN(target_altitude.terrain_alt_cm, max_alt_cm);
        }
        if (should_check_min) {
            target_altitude.terrain_alt_cm = MAX(target_altitude.terrain_alt_cm, min_alt_cm);
        }
        return;
    }
#endif

    if (should_check_max) {
        target_altitude.amsl_cm = MIN(target_altitude.amsl_cm, home.alt + max_alt_cm);
    }
    if (should_check_min) {
        target_altitude.amsl_cm = MAX(target_altitude.amsl_cm, home.alt + min_alt_cm);
    }
}

/*
  reset the altitude offset used for altitude slopes
 */
void Plane::reset_offset_altitude(void)
{
    target_altitude.offset_cm = 0;
}


/*
  reset the altitude offset used for slopes, based on difference between
  altitude at a destination and a specified start altitude. If destination is
  above the starting altitude then the result is positive.
 */
void Plane::set_offset_altitude_location(const Location &start_loc, const Location &destination_loc)
{
    ftype alt_difference_m = 0;
    if (destination_loc.get_height_above(start_loc, alt_difference_m)) {
        target_altitude.offset_cm = alt_difference_m * 100;
    } else {
        target_altitude.offset_cm = 0;
    }

#if AP_TERRAIN_AVAILABLE
    /*
      if this location has the terrain_alt flag set and we know the
      terrain altitude of our current location then treat it as a
      terrain altitude
     */
    float height;
    if (destination_loc.terrain_alt && 
        target_altitude.terrain_following &&
        terrain.height_above_terrain(height, true)) {
        target_altitude.offset_cm = target_altitude.terrain_alt_cm - (height * 100);
    }
#endif

    if (flight_stage != AP_FixedWing::FlightStage::LAND) {
        // if we are within ALT_SLOPE_MIN meters of the target altitude then
        // reset the offset to not use an altitude slope. This allows for more
        // accurate flight of missions where the aircraft may lose or gain a bit
        // of altitude near waypoint turn points due to local terrain changes
        if (g.alt_slope_min <= 0 ||
            labs(target_altitude.offset_cm)*0.01f < g.alt_slope_min) {
            target_altitude.offset_cm = 0;
        }
    }
}

/*
  return true if current_loc is above loc. Used for altitude slope
  calculations.

  "above" is simple if we are not terrain following, as it just means
  the pressure altitude of one is above the other.

  When in terrain following mode "above" means the over-the-terrain
  current altitude is above the over-the-terrain alt of loc. It is
  quite possible for current_loc to be "above" loc when it is at a
  lower pressure altitude, if current_loc is in a low part of the
  terrain
 */
bool Plane::above_location_current(const Location &loc)
{
#if AP_TERRAIN_AVAILABLE
    float terrain_alt;
    if (loc.terrain_alt && 
        terrain.height_above_terrain(terrain_alt, true)) {
        float loc_alt = loc.alt*0.01f;
        if (!loc.relative_alt) {
            loc_alt -= home.alt*0.01f;
        }
        return terrain_alt > loc_alt;
    }
#endif

    float loc_alt_cm = loc.alt;
    if (loc.relative_alt) {
        loc_alt_cm += home.alt;
    }
    return current_loc.alt > loc_alt_cm;
}

/*
  modify a destination to be setup for terrain following if
  TERRAIN_FOLLOW is enabled
 */
void Plane::setup_terrain_target_alt(Location &loc)
{
#if AP_TERRAIN_AVAILABLE
    if (terrain_enabled_in_current_mode()) {
        if (!loc.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            target_altitude.terrain_following_pending = true;
            return;
        }
    }
    target_altitude.terrain_following_pending = false;
#endif
}

/*
  return current_loc.alt adjusted for ALT_OFFSET
  This is useful during long flights to account for barometer changes
  from the GCS, or to adjust the flying height of a long mission
 */
int32_t Plane::adjusted_altitude_cm(void)
{
    return current_loc.alt - (mission_alt_offset()*100);
}

/*
  return home-relative altitude adjusted for ALT_OFFSET. This is useful
  during long flights to account for barometer changes from the GCS,
  or to adjust the flying height of a long mission.
 */
int32_t Plane::adjusted_relative_altitude_cm(void)
{
    return (relative_altitude - mission_alt_offset())*100;
}


/*
  return the mission altitude offset. This raises or lowers all
  mission items. It is primarily set using the ALT_OFFSET parameter,
  but can also be adjusted by the rangefinder landing code for a
  NAV_LAND command if we have aborted a steep landing
 */
float Plane::mission_alt_offset(void)
{
    float ret = g.alt_offset;
    if (control_mode == &mode_auto &&
            (flight_stage == AP_FixedWing::FlightStage::LAND || auto_state.wp_is_land_approach)) {
        // when landing after an aborted landing due to too high glide
        // slope we use an offset from the last landing attempt
        ret += landing.alt_offset;
    }
    return ret;
}

/*
  return the height in meters above the next_WP_loc altitude
 */
float Plane::height_above_target(void)
{
    float target_alt = next_WP_loc.alt*0.01;
    if (!next_WP_loc.relative_alt) {
        target_alt -= ahrs.get_home().alt*0.01f;
    }

#if AP_TERRAIN_AVAILABLE
    // also record the terrain altitude if possible
    float terrain_altitude;
    if (next_WP_loc.terrain_alt && 
        terrain.height_above_terrain(terrain_altitude, true)) {
        return terrain_altitude - target_alt;
    }
#endif

    return (adjusted_altitude_cm()*0.01f - ahrs.get_home().alt*0.01f) - target_alt;
}

/*
  work out target altitude adjustment from terrain lookahead
 */
float Plane::lookahead_adjustment(void)
{
#if AP_TERRAIN_AVAILABLE
    int32_t bearing_cd;
    int16_t distance;
    // work out distance and bearing to target
    if (control_mode == &mode_fbwb) {
        // there is no target waypoint in FBWB, so use yaw as an approximation
        bearing_cd = ahrs.yaw_sensor;
        distance = g.terrain_lookahead;
    } else if (!reached_loiter_target()) {
        bearing_cd = nav_controller->target_bearing_cd();
        distance = constrain_float(auto_state.wp_distance, 0, g.terrain_lookahead);
    } else {
        // no lookahead when loitering
        bearing_cd = 0;
        distance = 0;
    }
    if (distance <= 0) {
        // no lookahead
        return 0;
    }

    
    float groundspeed = ahrs.groundspeed();
    if (groundspeed < 1) {
        // we're not moving
        return 0;
    }
    // we need to know the climb ratio. We use 50% of the maximum
    // climb rate so we are not constantly at 100% throttle and to
    // give a bit more margin on terrain
    float climb_ratio = 0.5f * TECS_controller.get_max_climbrate() / groundspeed;

    if (climb_ratio <= 0) {
        // lookahead makes no sense for negative climb rates
        return 0;
    }
    
    // ask the terrain code for the lookahead altitude change
    float lookahead = terrain.lookahead(bearing_cd*0.01f, distance, climb_ratio);
    
    if (target_altitude.offset_cm < 0) {
        // we are heading down to the waypoint, so we don't need to
        // climb as much
        lookahead += target_altitude.offset_cm*0.01f;
    }

    // constrain lookahead to a reasonable limit
    return constrain_float(lookahead, 0, 1000.0f);
#else
    return 0;
#endif
}


#if AP_RANGEFINDER_ENABLED
/*
  correct target altitude using rangefinder data. Returns offset in
  meters to correct target altitude. A positive number means we need
  to ask the speed/height controller to fly higher
 */
float Plane::rangefinder_correction(void)
{
#if MODE_AUTOLAND_ENABLED
    if (rangefinder_autoland_rules() && !rangefinder_state.in_range && rf_land.hold != RFHold::NONE &&
        rangefinder_use(RangeFinderUse::TAKEOFF_LANDING)) {
        return rangefinder_autoland_held_correction();
    }
#endif
    if (millis() - rangefinder_state.last_correction_time_ms > 5000) {
        // we haven't had any rangefinder data for 5s - don't use it
        return 0;
    }

    // for now we only support the rangefinder for landing 
    bool using_rangefinder = (rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) && flight_stage == AP_FixedWing::FlightStage::LAND);
    if (!using_rangefinder) {
        return 0;
    }

    return rangefinder_state.correction;
}

/*
  correct rangefinder data for terrain height difference between
  NAV_LAND point and current location
 */
void Plane::rangefinder_terrain_correction(float &height)
{
#if AP_TERRAIN_AVAILABLE
    if (!rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) ||
        flight_stage != AP_FixedWing::FlightStage::LAND ||
        !terrain_enabled_in_current_mode()) {
        return;
    }
    float terrain_amsl1, terrain_amsl2;
    if (!terrain.height_amsl(current_loc, terrain_amsl1) ||
        !terrain.height_amsl(next_WP_loc, terrain_amsl2)) {
        return;
    }
    float correction = (terrain_amsl1 - terrain_amsl2);
    height += correction;
    auto_state.terrain_correction = correction;
#endif
}

/*
  update the offset between rangefinder height and terrain height
 */
void Plane::rangefinder_height_update(void)
{
    const auto orientation = rangefinder_orientation();
    bool range_ok = rangefinder.status_orient(orientation) == RangeFinder::Status::Good;
    float distance = rangefinder.distance_orient(orientation);
    float corrected_distance = distance;

    /*
      correct distance for attitude
     */
    if (range_ok) {
        // correct the range for attitude
        const auto &dcm = ahrs.get_rotation_body_to_ned();

        Vector3f v{corrected_distance, 0, 0};
        v.rotate(orientation);
        v = dcm * v;

        if (!is_positive(v.z)) {
            // not pointing at the ground
            range_ok = false;
        } else {
            corrected_distance = v.z;
        }
    }

    // custom: reject readings that disagree badly with the baro height.
    // Some ToF sensors (MT-06) report random "good" 2-5m readings when the
    // ground is beyond their real range. Gate = 2.5m + 20% of baro height.
    if (range_ok && ahrs.home_is_set()) {
        const float baro_hgt = (flight_stage == AP_FixedWing::FlightStage::LAND) ?
            height_above_target() :
            (adjusted_altitude_cm() - ahrs.get_home().alt) * 0.01f;
        // a strong return (signal quality >= 50%, MT-06 strength >= 128) is
        // always the real ground: never seen above 15m. Trust it even when
        // the baro is off by more than the gate (baro reads 2-4m low near
        // the ground at landing speed on this airframe)
        const bool strong = rangefinder.signal_quality_pct_orient(orientation) >= 50;
        if (!strong && fabsf(corrected_distance - baro_hgt) > 2.5f + 0.2f * fabsf(baro_hgt)) {
            range_ok = false;
        }
    }

#if MODE_AUTOLAND_ENABLED
    if (range_ok && rangefinder_autoland_rules()) {
        // custom: on an AUTOLAND final the height is the wheels' height, so the
        // flare and touchdown are the same whatever the sensor's mounting height:
        // RNGFNDx_GNDCLR is what the sensor reads standing on its wheels
        corrected_distance -= rangefinder.ground_clearance_orient(orientation);
    }
#endif

    if (range_ok && ahrs.home_is_set()) {
        if (!rangefinder_state.have_initial_reading) {
            rangefinder_state.have_initial_reading = true;
            rangefinder_state.initial_range = distance;
        }
        rangefinder_state.height_estimate = corrected_distance;

        rangefinder_terrain_correction(rangefinder_state.height_estimate);

        // we consider ourselves to be fully in range when we have 10
        // good samples (0.2s) that are different by 5% of the maximum
        // range from the initial range we see. The 5% change is to
        // catch Lidars that are giving a constant range, either due
        // to misconfiguration or a faulty sensor
        if (rangefinder_state.in_range_count < 10) {
#if MODE_AUTOLAND_ENABLED
            if (rangefinder_autoland_rules()) {
                // custom: on an AUTOLAND final, in range once the samples have
                // agreed with the aircraft's own vertical motion for 0.2 s
                if (rangefinder_consistent(orientation, distance, corrected_distance)) {
                    rangefinder_state.in_range_count = 10;
                }
            } else
#endif
            {
                if (!is_equal(distance, rangefinder_state.last_distance) &&
                    fabsf(rangefinder_state.initial_range - distance) > 0.05f * rangefinder.max_distance_orient(rangefinder_orientation())) {
                    rangefinder_state.in_range_count++;
                }
                if (fabsf(rangefinder_state.last_distance - distance) > rangefinder.max_distance_orient(rangefinder_orientation())*0.2) {
                    // changes by more than 20% of full range will reset counter
                    rangefinder_state.in_range_count = 0;
                }
            }
        } else {
            rangefinder_state.in_range = true;
            bool flightstage_good_for_rangefinder_landing = false;
            if (flight_stage == AP_FixedWing::FlightStage::LAND) {
                flightstage_good_for_rangefinder_landing = true;
            }
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland ||
                control_mode == &mode_qrtl ||
                (control_mode == &mode_auto && quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))) {
                flightstage_good_for_rangefinder_landing = true;
            }
#endif
            if (!rangefinder_state.in_use &&
                flightstage_good_for_rangefinder_landing &&
                rangefinder_use(RangeFinderUse::TAKEOFF_LANDING)) {
                rangefinder_state.in_use = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder engaged at %.2fm", (double)rangefinder_state.height_estimate);
            }
        }
        rangefinder_state.last_distance = distance;
    } else {
        rangefinder_state.in_range_count = 0;
        rangefinder_state.in_range = false;
    }

#if MODE_AUTOLAND_ENABLED
    rangefinder_autoland_update(orientation, distance, range_ok);
#endif

    if (rangefinder_state.in_range) {
        // If not using terrain data, we expect zero correction when our height above target is equal to our rangefinder measurement
        float correction = height_above_target() - rangefinder_state.height_estimate;

#if AP_TERRAIN_AVAILABLE
        // if we are terrain following then correction is based on terrain data
        float terrain_altitude;
        if ((target_altitude.terrain_following || terrain_enabled_in_current_mode()) && 
            terrain.height_above_terrain(terrain_altitude, true)) {
            correction = terrain_altitude - rangefinder_state.height_estimate;
        }
#endif    

        // remember the last correction. Use a low pass filter unless
        // the old data is more than 5 seconds old
        uint32_t now = millis();
        if (now - rangefinder_state.last_correction_time_ms > 5000) {
            rangefinder_state.correction = correction;
            rangefinder_state.initial_correction = correction;
            if (rangefinder_use(RangeFinderUse::TAKEOFF_LANDING)) {
                landing.set_initial_slope();
            }
            rangefinder_state.last_correction_time_ms = now;
        } else {
            rangefinder_state.correction = 0.8f*rangefinder_state.correction + 0.2f*correction;
            rangefinder_state.last_correction_time_ms = now;
            if (fabsf(rangefinder_state.correction - rangefinder_state.initial_correction) > 30) {
                // the correction has changed by more than 30m, reset use of Lidar. We may have a bad lidar
                if (rangefinder_state.in_use) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Rangefinder disengaged at %.2fm", (double)rangefinder_state.height_estimate);
                }
                memset(&rangefinder_state, 0, sizeof(rangefinder_state));
            }
        }

    }
}

#if MODE_AUTOLAND_ENABLED
/*
  custom (AMA field mode): rangefinder rules that only apply on an AUTOLAND
  final (flight stage LAND). Everywhere else the stock rules apply.
 */
bool Plane::rangefinder_autoland_rules(void) const
{
    return control_mode == &mode_autoland && flight_stage == AP_FixedWing::FlightStage::LAND;
}

/*
  engagement: a new sample counts if it agrees with the previous one plus the
  aircraft's own vertical motion (EKF vertical speed x time between samples;
  heights tilt-corrected). In range once the agreeing run spans at least
  0.18 s, so the same at 10 Hz and 250 Hz, with at least 3 samples if all have
  a quality of 50% or more, else 6 (2 and 5 agreements in a row).
  - no quality value needed: random "valid" readings beyond a sensor's real
    range don't follow the descent, so they keep restarting the run
  - tolerance: 0.3 m + 0.5 m/s x dt at quality >= 50%, else 0.1 m + 0.3 m/s x dt;
    quality below 20% doesn't count
  - a repeated identical value doesn't count (stock anti-stuck check)
 */
bool Plane::rangefinder_consistent(enum Rotation orientation, float distance, float height)
{
    const uint32_t sample_ms = rangefinder.last_reading_ms(orientation);
    if (sample_ms == rf_land.last_sample_ms) {
        // no new sample since the last call
        return false;
    }
    const float dt = (sample_ms - rf_land.last_sample_ms) * 0.001f;
    rf_land.last_sample_ms = sample_ms;
    if (is_equal(distance, rangefinder_state.last_distance)) {
        return false;
    }
    const int8_t quality = rangefinder.signal_quality_pct_orient(orientation);
    if (quality >= 0 && quality < 20) {
        rf_land.count = 0;
        return false;
    }
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        rf_land.count = 0;
        return false;
    }
    // sensor noise, plus EKF vertical speed error over dt
    const bool strong = quality >= 50;
    const float tolerance = strong ? (0.3f + 0.5f * dt) : (0.1f + 0.3f * dt);
    const float predicted = rf_land.last_height - vel.z * dt;
    if (rf_land.count == 0 || dt > 0.5f || fabsf(height - predicted) > tolerance) {
        // start a new run with this sample
        rf_land.count = 1;
        rf_land.start_ms = sample_ms;
        rf_land.weak_in_run = false;
    } else if (rf_land.count < 255) {
        rf_land.count++;
    }
    rf_land.weak_in_run |= !strong;
    rf_land.last_height = height;
    // a run with any weak or no-quality sample needs 6 samples: random readings
    // agree with the descent by chance now and then, never 5 times in a row
    return rf_land.count >= (rf_land.weak_in_run ? 6 : 3) && sample_ms - rf_land.start_ms >= 180;
}

/*
  leaving range near the ground: once the rangefinder has been in use on the
  final and its range ends (status OutOfRangeLow, NoData or otherwise not
  Good), keep the last correction, so the landing height doesn't step back
  to the baro:
  - last Good reading within RNGFNDx_MIN + 0.3 m and under 1 s before:
    the sensor's minimum range or blind zone: hold for the rest of the landing
  - anywhere else (sensor failed, out of range high): hold 1 s, then fade to
    the baro over 3 s, instead of stock's full correction for 5 s then a step
 */
void Plane::rangefinder_autoland_update(enum Rotation orientation, float distance, bool range_ok)
{
    const uint32_t now_ms = millis();
    if (!rangefinder_autoland_rules()) {
        memset(&rf_land, 0, sizeof(rf_land));
        return;
    }
    if (!range_ok) {
        // an invalid or rejected sample restarts the engagement run
        rf_land.count = 0;
    }
    if (rangefinder.status_orient(orientation) == RangeFinder::Status::Good) {
        rf_land.last_good_ms = now_ms;
        rf_land.last_good_dist = distance;
    }
    if (rangefinder_state.in_range) {
        rf_land.hold = RFHold::NONE;
    } else if (rangefinder_state.in_use && rf_land.hold == RFHold::NONE) {
        const bool near_min = rf_land.last_good_ms != 0 && now_ms - rf_land.last_good_ms < 1000 &&
                              rf_land.last_good_dist <= rangefinder.min_distance_orient(orientation) + 0.3f;
        rf_land.hold = near_min ? RFHold::NEAR_GROUND : RFHold::FADE;
    }

#if HAL_LOGGING_ENABLED
    if (now_ms - rf_land.last_log_ms >= 100) {
        rf_land.last_log_ms = now_ms;
        const float hat = height_above_target();
        const float corr = rangefinder_correction();
        // what stock would apply: the correction for 5 s after the last update
        const float stock_corr = (now_ms - rangefinder_state.last_correction_time_ms > 5000) ? 0 : rangefinder_state.correction;
        // @LoggerMessage: ALRF
        // @Description: AUTOLAND final: rangefinder engagement and landing height
        // @Field: TimeUS: Time since system startup
        // @Field: D: raw rangefinder distance
        // @Field: St: rangefinder status (0 NotConnected, 1 NoData, 2 OutOfRangeLow, 3 OutOfRangeHigh, 4 Good)
        // @Field: Q: signal quality (-1 unknown)
        // @Field: RH: rangefinder height (tilt corrected)
        // @Field: InR: rangefinder in range
        // @Field: Cnt: consistent samples in the current run
        // @Field: Hold: 0 none, 1 holding near the ground, 2 fading to baro
        // @Field: Corr: correction applied
        // @Field: LH: landing height (height above target minus correction)
        // @Field: LHS: landing height with the stock correction rule
        // @Field: VZ: EKF vertical speed, down positive
        Vector3f vel;
        if (!ahrs.get_velocity_NED(vel)) {
            vel.zero();
        }
        AP::logger().WriteStreaming("ALRF", "TimeUS,D,St,Q,RH,InR,Cnt,Hold,Corr,LH,LHS,VZ",
                                    "sm-%m---mmmn", "F0--0---000-", "QfBbfBBBffff",
                                    AP_HAL::micros64(), distance, uint8_t(rangefinder.status_orient(orientation)),
                                    rangefinder.signal_quality_pct_orient(orientation),
                                    rangefinder_state.height_estimate, uint8_t(rangefinder_state.in_range),
                                    rf_land.count, uint8_t(rf_land.hold), corr, hat - corr, hat - stock_corr, vel.z);
    }
#endif
}

float Plane::rangefinder_autoland_held_correction(void) const
{
    if (rf_land.hold == RFHold::NEAR_GROUND) {
        return rangefinder_state.correction;
    }
    // FADE: full for 1 s after the last update, then linearly to 0 over 3 s
    const float t = (millis() - rangefinder_state.last_correction_time_ms) * 0.001f;
    return rangefinder_state.correction * constrain_float(1.0f - (t - 1.0f) / 3.0f, 0.0f, 1.0f);
}
#endif  // MODE_AUTOLAND_ENABLED
#endif  // AP_RANGEFINDER_ENABLED

/*
  determine if Non Auto Terrain Disable is active and allowed in present control mode
 */
bool Plane::terrain_disabled()
{
    return control_mode->allows_terrain_disable() && non_auto_terrain_disable;
}


/*
  Check if terrain following is enabled for the current mode
 */
#if AP_TERRAIN_AVAILABLE
const Plane::TerrainLookupTable Plane::Terrain_lookup[] = {
    {Mode::Number::FLY_BY_WIRE_B, terrain_bitmask::FLY_BY_WIRE_B},
    {Mode::Number::CRUISE, terrain_bitmask::CRUISE},
    {Mode::Number::AUTO, terrain_bitmask::AUTO},
    {Mode::Number::RTL, terrain_bitmask::RTL},
    {Mode::Number::AVOID_ADSB, terrain_bitmask::AVOID_ADSB},
    {Mode::Number::GUIDED, terrain_bitmask::GUIDED},
    {Mode::Number::LOITER, terrain_bitmask::LOITER},
    {Mode::Number::CIRCLE, terrain_bitmask::CIRCLE},
#if HAL_QUADPLANE_ENABLED
    {Mode::Number::QRTL, terrain_bitmask::QRTL},
    {Mode::Number::QLAND, terrain_bitmask::QLAND},
    {Mode::Number::QLOITER, terrain_bitmask::QLOITER},
#endif
#if MODE_AUTOLAND_ENABLED
    {Mode::Number::AUTOLAND, terrain_bitmask::AUTOLAND},
#endif
};

bool Plane::terrain_enabled_in_current_mode() const
{
    return terrain_enabled_in_mode(control_mode->mode_number());
}

bool Plane::terrain_enabled_in_mode(Mode::Number num) const
{
    // Global enable
    if ((g.terrain_follow.get() & int32_t(terrain_bitmask::ALL)) != 0) {
        return true;
    }

    // Specific enable
    for (const struct TerrainLookupTable entry : Terrain_lookup) {
        if (entry.mode_num == num) {
            if ((g.terrain_follow.get() & int32_t(entry.bitmask)) != 0) {
                return true;
            }
            break;
        }
    }

    return false;
}
#endif

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
/*
  handle a MAV_CMD_SET_HAGL request. The accuracy is ignored
 */
void Plane::handle_external_hagl(const mavlink_command_int_t &packet)
{
    auto &hagl = plane.external_hagl;
    hagl.hagl = packet.param1;
    hagl.last_update_ms = AP_HAL::millis();
    hagl.timeout_ms = uint32_t(packet.param3 * 1000);
}

/*
  get HAGL from external source if current
 */
bool Plane::get_external_HAGL(float &height_agl)
{
    auto &hagl = plane.external_hagl;
    if (hagl.last_update_ms != 0) {
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - hagl.last_update_ms <= hagl.timeout_ms) {
            height_agl = hagl.hagl;
            return true;
        }
        hagl.last_update_ms = 0;
    }
    return false;
}
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

/*
  get height for landing. Set using_rangefinder to true if a rangefinder
  or external HAGL source is active
 */
float Plane::get_landing_height(bool &rangefinder_active)
{
    float height;

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    // if external HAGL is active use that
    if (get_external_HAGL(height)) {
        // ensure no terrain correction is applied - that is the job
        // of the external system if it is wanted
        auto_state.terrain_correction = 0;

        // an external HAGL is considered to be a type of rangefinder
        rangefinder_active = true;
        return height;
    }
#endif

    // get basic height above target
    height = height_above_target();
    rangefinder_active = false;

#if AP_RANGEFINDER_ENABLED
    // possibly correct with rangefinder
    height -= rangefinder_correction();
    rangefinder_active = rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) && rangefinder_state.in_range;
#endif

    return height;
}

/*
  if a terrain location doesn't have the relative_alt flag set
  then fix the alt and trigger a flow of control error
 */
void Plane::fix_terrain_WP(Location &loc, uint32_t linenum)
{
    if (loc.terrain_alt && !loc.relative_alt) {
        AP::internalerror().error(AP_InternalError::error_t::flow_of_control, linenum);
        /*
          we definitely have a bug, now we need to guess what was
          really meant. The lack of the relative_alt flag notionally
          means that home.alt has been added to loc.alt, so remove it,
          but only if it doesn't lead to a negative terrain altitude
         */
        if (loc.alt - home.alt > -500) {
            loc.alt -= home.alt;
        }
        loc.relative_alt = true;
    }
}
