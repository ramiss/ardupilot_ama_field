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
#include "AP_RangeFinder_SITL.h"

#if AP_RANGEFINDER_SIM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

/*
  constructor - registers instance at top RangeFinder driver
 */
AP_RangeFinder_SITL::AP_RangeFinder_SITL(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, uint8_t instance) :
    AP_RangeFinder_Backend(_state, _params),
    _instance(instance)
{}

/*
  update distance_cm
 */
void AP_RangeFinder_SITL::update(void)
{
    const float dist = AP::sitl()->get_rangefinder(_instance);

    // nan distance means nothing is connected
    if (isnan(dist) || isinf(dist)) {
        state.status = RangeFinder::Status::NoData;
        return;
    }

    state.distance_m = MAX(0, dist);
    state.last_reading_ms = AP_HAL::millis();

    // SIM_SONAR_STRONG: signal quality like a small ToF sensor outdoors,
    // with SIM_SONAR_GLITCH giving weak random readings
    const float strong_m = AP::sitl()->sonar_strong;
    if (is_positive(strong_m)) {
        const float glitch = AP::sitl()->sonar_glitch;
        if (is_positive(glitch) && glitch >= (rand_float() + 1.0f) * 0.5f) {
            state.distance_m = 3.5f + 1.5f * rand_float();
            state.signal_quality_pct = 20;
        } else {
            state.signal_quality_pct = (state.distance_m < strong_m) ? 100 : 20;
        }
    } else {
        state.signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    }

    // update range_valid state based on distance measured
    update_status();
}

#endif  // AP_RANGEFINDER_SIM_ENABLED
