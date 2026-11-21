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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_RangeFinder_Backend::AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        state(_state),
		params(_params)
{
    _backend_type = type();
}

MAV_DISTANCE_SENSOR AP_RangeFinder_Backend::get_mav_distance_sensor_type() const {
    if (type() == RangeFinder::Type::NONE) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return _get_mav_distance_sensor_type();
}

RangeFinder::Status AP_RangeFinder_Backend::status() const {
    if (type() == RangeFinder::Type::NONE) {
        // turned off at runtime?
        return RangeFinder::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AP_RangeFinder_Backend::has_data() const {
    return ((state.status != RangeFinder::Status::NotConnected) &&
            (state.status != RangeFinder::Status::NoData));
}

// update status based on distance measurement
void AP_RangeFinder_Backend::update_status(RangeFinder::RangeFinder_State &state_arg) const
{
    // check distance
    if (state_arg.distance_m > max_distance()) {
        set_status(state_arg, RangeFinder::Status::OutOfRangeHigh);
    } else if (state_arg.distance_m < min_distance()) {
        set_status(state_arg, RangeFinder::Status::OutOfRangeLow);
    } else {
        set_status(state_arg, RangeFinder::Status::Good);
    }
}

// set status and update valid count
void AP_RangeFinder_Backend::set_status(RangeFinder::RangeFinder_State &state_arg, RangeFinder::Status _status)
{
    state_arg.status = _status;

    // update valid count
    if (_status == RangeFinder::Status::Good) {
        if (state_arg.range_valid_count < 10) {
            state_arg.range_valid_count++;
        }
    } else {
        state_arg.range_valid_count = 0;
    }
}

#if AP_SCRIPTING_ENABLED
// get a copy of state structure
void AP_RangeFinder_Backend::get_state(RangeFinder::RangeFinder_State &state_arg)
{
    WITH_SEMAPHORE(_sem);
    state_arg = state;
}
#endif

#if HAL_LOGGING_ENABLED
void AP_RangeFinder_Backend::log_raw_sample(float raw_m, uint16_t strength, uint8_t sensor_status, int8_t quality_pct, bool valid)
{
    const uint32_t now_ms = AP_HAL::millis();
    // every valid sample inside the configured range; anything else at most
    // 10 Hz, to keep the rate down on a MAVLink logger
    if (!(valid && raw_m < max_distance()) && now_ms - last_raw_log_ms < 100) {
        return;
    }
    last_raw_log_ms = now_ms;
    // @LoggerMessage: RFSS
    // @Description: Rangefinder raw sample, for drivers that report a signal strength
    // @Field: TimeUS: Time since system startup
    // @Field: Ty: rangefinder type (RNGFNDx_TYPE)
    // @Field: D: raw distance as reported, also for invalid samples
    // @Field: Str: raw signal strength (sensor specific scale)
    // @Field: St: sample status (1 = valid; other values are sensor specific)
    // @Field: Q: signal quality 0-100% as reported to ArduPilot (-1 = unknown)
    AP::logger().WriteStreaming("RFSS", "TimeUS,Ty,D,Str,St,Q", "s-m--%", "F-0---", "QBfHBb",
                                AP_HAL::micros64(), uint8_t(params.type.get()), raw_m, strength,
                                sensor_status, quality_pct);
}
#endif

#endif  // AP_RANGEFINDER_ENABLED
