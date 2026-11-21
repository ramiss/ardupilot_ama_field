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
    SITL::SIM *sitl = AP::sitl();
    const float dist = sitl->get_rangefinder(_instance);

    // nan distance means nothing is connected
    if (isnan(dist) || isinf(dist)) {
        state.status = RangeFinder::Status::NoData;
        return;
    }

    // custom test options (SIM_SONAR_*), to simulate different sensors
    const uint32_t now_ms = AP_HAL::millis();
    if (sitl->sonar_fail != 0) {
        // a sensor that died
        set_status(RangeFinder::Status::NoData);
        return;
    }
    if (is_positive(sitl->sonar_hz.get()) && now_ms - last_sample_ms < uint32_t(1000.0f / sitl->sonar_hz)) {
        // between samples of a slower sensor: keep the last one
        return;
    }
    last_sample_ms = now_ms;
    const float drop = sitl->sonar_drop;
    if (is_positive(drop) && drop >= (rand_float() + 1.0f) * 0.5f) {
        set_status(RangeFinder::Status::NoData);
        return;
    }

    float d = MAX(0, dist);
    int8_t quality = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    const float strong_m = sitl->sonar_strong;
    const float glitch = sitl->sonar_glitch;
    if (is_positive(glitch) && glitch >= (rand_float() + 1.0f) * 0.5f) {
        // a spike: random 0.5-5m, weak if the sensor reports a quality
        d = 0.5f + 4.5f * (rand_float() + 1.0f) * 0.5f;
        quality = is_positive(strong_m) ? 20 : RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    } else if (is_positive(strong_m)) {
        // SIM_SONAR_STRONG: signal quality like a small ToF sensor outdoors
        if (d < strong_m) {
            quality = 100;
        } else if (sitl->sonar_garb != 0) {
            // beyond its real range: random "valid" 2-5m readings, like the MT-06
            d = 2.0f + 3.0f * (rand_float() + 1.0f) * 0.5f;
            quality = int8_t(30 + 15 * (rand_float() + 1.0f) * 0.5f);
        } else {
            quality = 20;
        }
    }

    state.distance_m = d;
    state.signal_quality_pct = quality;
    state.last_reading_ms = now_ms;

    // update range_valid state based on distance measured
    update_status();
}

#endif  // AP_RANGEFINDER_SIM_ENABLED
