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
#include "AP_RangeFinder_Benewake_TFMiniPlus.h"

#if AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define DRIVER "TFMiniPlus"
#define BENEWAKE_OUT_OF_RANGE_ADD_CM 100

/*
 * Command format:
 *
 * uint8_t header;
 * uint8_t len;
 * uint8_t id;
 * uint8_t data[];
 * uint8_t checksum;
 */

bool AP_RangeFinder_Benewake_TFMiniPlus::init()
{
    const uint8_t CMD_FW_VERSION[] =         { 0x5A, 0x04, 0x01, 0x5F };
    const uint8_t CMD_SYSTEM_RESET[] =       { 0x5A, 0x04, 0x04, 0x62 };
    const uint8_t CMD_OUTPUT_FORMAT_CM[] =   { 0x5A, 0x05, 0x05, 0x01, 0x65 };
    const uint8_t CMD_ENABLE_DATA_OUTPUT[] = { 0x5A, 0x05, 0x07, 0x01, 0x67 };
    const uint8_t CMD_FRAME_RATE_250HZ[] =   { 0x5A, 0x06, 0x03, 0xFA, 0x00, 0x5D };
    const uint8_t CMD_SAVE_SETTINGS[] =      { 0x5A, 0x04, 0x11, 0x6F };
    const uint8_t *cmds[] = {
        CMD_OUTPUT_FORMAT_CM,
        CMD_FRAME_RATE_250HZ,
        CMD_ENABLE_DATA_OUTPUT,
        CMD_SAVE_SETTINGS,
    };
    uint8_t val[12], i;
    bool ret;

    WITH_SEMAPHORE(dev.get_semaphore());

    dev.set_retries(0);

    /*
     * Check we get a response for firmware version to detect if sensor is there
     */
    ret = dev.transfer(CMD_FW_VERSION, sizeof(CMD_FW_VERSION), nullptr, 0);
    if (!ret) {
        return false;
    }

    hal.scheduler->delay(100);

    ret = dev.transfer(nullptr, 0, val, 7);
    if (!ret || val[0] != 0x5A || val[1] != 0x07 || val[2] != 0x01 ||
        !check_checksum(val, 7)) {
        return false;
    }

    if (val[5] * 10000 + val[4] * 100 + val[3] < 20003) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TFMini: FW ver %u.%u.%u (need>=2.0.3)",
                            (unsigned)val[5],(unsigned)val[4],(unsigned)val[3]);
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TFMiniPlus: found fw version %u.%u.%u",
                        val[5], val[4], val[3]);

    for (i = 0; i < ARRAY_SIZE(cmds); i++) {
        ret = dev.transfer(cmds[i], cmds[i][1], nullptr, 0);
        if (!ret) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TFMiniPlus: Unable to set configuration register %u",
                                cmds[i][2]);
            return false;
        }
        hal.scheduler->delay(100);
    }

    dev.transfer(CMD_SYSTEM_RESET, sizeof(CMD_SYSTEM_RESET), nullptr, 0);

    hal.scheduler->delay(100);

    dev.register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Benewake_TFMiniPlus::timer, void));

    return true;
}

void AP_RangeFinder_Benewake_TFMiniPlus::update()
{
    WITH_SEMAPHORE(_sem);

#if HAL_LOGGING_ENABLED
    if (accum.count > 0) {
        // the last raw sample of this update, valid or not (RFSS)
        log_raw_sample(accum.last_raw_cm * 0.01f, accum.last_strength, accum.last_status,
                       accum.last_quality, accum.last_status == RAW_VALID);
    }
#endif

    if (accum.valid_count > 0) {
        // custom: average only the valid samples (stock averaged the
        // out-of-range placeholder in with them), and report their signal quality
        state.distance_m = (accum.sum * 0.01f) / accum.valid_count;
        state.signal_quality_pct = int8_t(accum.quality_sum / accum.valid_count);
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (accum.count > 0) {
        // only unreliable samples: out of range high, as stock
        state.distance_m = MAX(1200, max_distance()*100 + BENEWAKE_OUT_OF_RANGE_ADD_CM) * 0.01f;
        state.signal_quality_pct = 0;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::Status::NoData);
    }
    accum.sum = 0;
    accum.count = 0;
    accum.valid_count = 0;
    accum.quality_sum = 0;
}

/*
  custom: signal quality from the strength (Amp), provisional until checked
  against flight data. TF-Luna manual (REV 01/04/2024), 3.3 Repeatability:
  standard deviation 3 cm at Amp 100, 2 cm at 400, 1 cm at 1000, 0.5 cm at
  2000 and above; Amp < 100 is unreliable. On that log scale: 0% at Amp 100,
  50% at 316, 100% at 1000 and above, at least 1% for any valid sample.
 */
int8_t AP_RangeFinder_Benewake_TFMiniPlus::strength_to_quality(uint16_t strength)
{
    if (strength < 100 || strength >= 32768) {
        return 0;
    }
    return int8_t(constrain_int16(lroundf(100.0f * log10f(strength * 0.01f)), 1, 100));
}

bool AP_RangeFinder_Benewake_TFMiniPlus::process_raw_measure(le16_t distance_raw, le16_t strength_raw,
                                                             uint16_t &output_distance_cm, uint8_t &raw_status)
{
    uint16_t strength = le16toh(strength_raw);
    const uint16_t MAX_DIST_CM = 1200;
    const uint16_t MIN_DIST_CM = 10;

    output_distance_cm = le16toh(distance_raw);

    /*
     * From manual: "when the signal strength is lower than 100 or equal to
     * 65535, the detection is unreliable, TFmini Plus will set distance
     * value to 0." TF-Luna manual: above 32768 means ambient light
     * overexposure (e.g. facing the sun).
     */
    if (strength < 100) {
        raw_status = RAW_WEAK;
    } else if (strength >= 32768) {
        raw_status = RAW_OVEREXPOSED;
    } else if (output_distance_cm > MAX_DIST_CM) {
        raw_status = RAW_TOO_FAR;
    } else {
        raw_status = RAW_VALID;
        output_distance_cm = constrain_int16(output_distance_cm, MIN_DIST_CM, MAX_DIST_CM);
        return true;
    }
    return false;
}

bool AP_RangeFinder_Benewake_TFMiniPlus::check_checksum(uint8_t *arr, int pkt_len)
{
    uint8_t checksum = 0;
    int i;

    /* sum them all except the last (the checksum) */
    for (i = 0; i < pkt_len - 1; i++) {
        checksum += arr[i];
    }

    return checksum == arr[pkt_len - 1];
}

void AP_RangeFinder_Benewake_TFMiniPlus::timer()
{
    uint8_t CMD_READ_MEASUREMENT[] = { 0x5A, 0x05, 0x00, 0x07, 0x66 };
    union {
        struct PACKED {
            uint8_t header1;
            uint8_t header2;
            le16_t distance;
            le16_t strength;
            le32_t timestamp;
            uint8_t checksum;
        } val;
        uint8_t arr[11];
    } u;
    uint16_t distance;

    if (!dev.transfer(CMD_READ_MEASUREMENT, sizeof(CMD_READ_MEASUREMENT), nullptr, 0)) {
        // Failed write
        return;
    }

    // Short delay helps avoid bad data, despite datasheet saying its not needed
    hal.scheduler->delay_microseconds(10);

    if (!dev.transfer(nullptr, 0, (uint8_t *)&u, sizeof(u))) {
        // Failed read
        return;
    }

    if (u.val.header1 != 0x59 || u.val.header2 != 0x59 || !check_checksum(u.arr, sizeof(u))) {
        return;
    }

    uint8_t raw_status;
    const uint16_t raw_cm = le16toh(u.val.distance);
    const uint16_t strength = le16toh(u.val.strength);
    const bool valid = process_raw_measure(u.val.distance, u.val.strength, distance, raw_status);

    {
        WITH_SEMAPHORE(_sem);
        accum.count++;
        if (valid) {
            const int8_t q = strength_to_quality(strength);
            accum.sum += distance;
            accum.valid_count++;
            accum.quality_sum += q;
            accum.last_quality = q;
        } else {
            accum.last_quality = 0;
        }
        accum.last_raw_cm = raw_cm;
        accum.last_strength = strength;
        accum.last_status = raw_status;
    }
}

#endif  // AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED
