/*
 * Copyright (C) 2019  Lucas De Marchi <lucas.de.marchi@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

#include <AP_HAL/utility/sparse-endian.h>

#define TFMINIPLUS_ADDR_DEFAULT              0x10        // TFMini default device id

class AP_RangeFinder_Benewake_TFMiniPlus : public AP_RangeFinder_Backend_I2C
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &dev) {
        // this will free the object if configuration fails:
        return configure(NEW_NOTHROW AP_RangeFinder_Benewake_TFMiniPlus(_state, _params, dev));
    }

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    bool init() override;
    void timer();

    // raw sample status, as logged in RFSS.St
    static constexpr uint8_t RAW_WEAK = 0;          // strength < 100: unreliable
    static constexpr uint8_t RAW_VALID = 1;
    static constexpr uint8_t RAW_OVEREXPOSED = 2;   // strength >= 32768 (0xFFFF = saturated)
    static constexpr uint8_t RAW_TOO_FAR = 3;       // beyond the driver's 12 m limit

    // true if the sample is valid; output_distance_cm is only set then
    bool process_raw_measure(le16_t distance_raw, le16_t strength_raw,
                             uint16_t &output_distance_cm, uint8_t &raw_status);
    static int8_t strength_to_quality(uint16_t strength);

    bool check_checksum(uint8_t *arr, int pkt_len);

    struct {
        uint32_t sum;           // valid samples only, cm
        uint32_t count;         // all samples
        uint32_t valid_count;
        uint32_t quality_sum;
        uint16_t last_raw_cm;
        uint16_t last_strength;
        uint8_t last_status;
        int8_t last_quality;
    } accum;
};

#endif
