#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>
#include "quadplane.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>
#include "config.h"
#include "pullup.h"
#include "systemid.h"

#ifndef AP_QUICKTUNE_ENABLED
#define AP_QUICKTUNE_ENABLED HAL_QUADPLANE_ENABLED
#endif

#ifndef MODE_AUTOLAND_ENABLED
#define MODE_AUTOLAND_ENABLED 1
#endif

#include <AP_Baro/AP_Baro_config.h>

#include <AP_Quicktune/AP_Quicktune.h>

class AC_PosControl;
class AC_AttitudeControl_Multi;
class AC_Loiter;
class Mode
{
public:

    /* Do not allow copies */
    CLASS_NO_COPY(Mode);

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
#if HAL_QUADPLANE_ENABLED
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
#if QAUTOTUNE_ENABLED
        QAUTOTUNE     = 22,
#endif
        QACRO         = 23,
#endif
        THERMAL       = 24,
#if HAL_QUADPLANE_ENABLED
        LOITER_ALT_QLAND = 25,
#endif
#if MODE_AUTOLAND_ENABLED
        AUTOLAND      = 26,
#endif

    // Mode number 30 reserved for "offboard" for external/lua control.
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // run controllers specific to this mode
    virtual void run();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    bool pre_arm_checks(size_t buflen, char *buffer) const;

    // Reset rate and steering and TECS controllers
    void reset_controllers();

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }

    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }
    
    // true if the mode supports autotuning (via switch for modes other
    // that AUTOTUNE itself
    virtual bool mode_allows_autotuning() const { return false; }

    // method for mode specific target altitude profiles
    virtual void update_target_altitude();

    // handle a guided target request from GCS
    virtual bool handle_guided_request(Location target_loc) { return false; }

    // true if is landing 
    virtual bool is_landing() const { return false; }

    // true if is taking 
    virtual bool is_taking_off() const;

    // true if throttle min/max limits should be applied
    virtual bool use_throttle_limits() const;

    // true if voltage correction should be applied to throttle
    virtual bool use_battery_compensation() const;
 
#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    virtual bool allows_autoland_direction_capture() const { return false; }
#endif

#if AP_QUICKTUNE_ENABLED
    // does this mode support VTOL quicktune?
    virtual bool supports_quicktune() const { return false; }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    virtual bool supports_vtol_systemid() const { return false; }

    // does this mode support plane or quadplane fixed wing systemid?
    virtual bool supports_fw_systemid() const { return false; }

    // Return true if fixed wing system ID should be allowed
    bool allow_fw_systemid() const;
#endif

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // mode specific pre-arm checks
    virtual bool _pre_arm_checks(size_t buflen, char *buffer) const;

    // Helper to output to both k_rudder and k_steering servo functions
    void output_rudder_and_steering(float val);

    // Output pilot throttle, this is used in stabilized modes without auto throttle control
    void output_pilot_throttle();

    // makes the initialiser list in the constructor manageable
    uint8_t unused_integer;

#if HAL_QUADPLANE_ENABLED
    // References for convenience, used by QModes
    AC_PosControl*& pos_control;
    AC_AttitudeControl_Multi*& attitude_control;
    AC_Loiter*& loiter_nav;
    QuadPlane& quadplane;
    QuadPlane::PosControlState &poscontrol;
#endif
    AP_AHRS& ahrs;
};


class ModeAcro : public Mode
{
friend class ModeQAcro;
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "Acro"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    void stabilize();

    void stabilize_quaternion();

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

protected:

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
        Quaternion q;
        bool roll_active_last;
        bool pitch_active_last;
        bool yaw_active_last;
    } acro_state;

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:
    friend class Plane;

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "Auto"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override;

    bool does_auto_throttle() const override;
    
    bool mode_allows_autotuning() const override { return true; }

    bool is_landing() const override;

    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    bool verify_altitude_wait(const AP_Mission::Mission_Command& cmd);

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    bool in_pullup() const { return pullup.in_pullup(); }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
    void _exit() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override;

private:

    // Delay the next navigation command
    struct {
        uint32_t time_max_ms;
        uint32_t time_start_ms;
    } nav_delay;

    // wiggle state and timer for NAV_ALTITUDE_WAIT
    void wiggle_servos();
    struct {
        uint8_t stage;
        uint32_t last_ms;
    } wiggle;

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    GliderPullup pullup;
#endif // AP_PLANE_GLIDER_PULLUP_ENABLED
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "Autotune"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
    
protected:

    bool _enter() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "Guided"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // handle a guided airspeed command, typically from companion computer
    bool handle_change_airspeed(const float airspeed, const float acceleration);
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

    void set_radius_and_direction(const float radius, const bool direction_is_ccw);

    void update_target_altitude() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return true; }
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif

private:
    float active_radius_m;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "Circle"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "Loiter"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    bool allows_terrain_disable() const override { return true; }

    void update_target_altitude() override;
    
    bool mode_allows_autotuning() const override { return true; }

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
};

#if HAL_QUADPLANE_ENABLED
class ModeLoiterAltQLand : public ModeLoiter
{
public:

    Number mode_number() const override { return Number::LOITER_ALT_QLAND; }
    const char *name() const override { return "Loiter to QLand"; }
    const char *name4() const override { return "L2QL"; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:
    bool _enter() override;

    void navigate() override;

private:
    void switch_qland();

};
#endif // HAL_QUADPLANE_ENABLED

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "Manual"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    // true if throttle min/max limits should be applied
    bool use_throttle_limits() const override;

    // true if voltage correction should be applied to throttle
    bool use_battery_compensation() const override { return false; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    // Switch to QRTL if enabled and within radius
    uint32_t circle_refresh_ms;
    bool switch_QRTL();
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "Stabilize"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

private:
    void stabilize_stick_mixing_direct();

};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "Training"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "Initialising"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FBWA"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FBWB"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }
    
    bool mode_allows_autotuning() const override { return true; }

    void update_target_altitude() override {};

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "Cruise"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override {};

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "Avoid ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

#if HAL_QUADPLANE_ENABLED
class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QStabilize"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHover"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLoiter : public Mode
{
friend class QuadPlane;
friend class ModeQLand;
friend class Plane;

public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLoiter"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
    uint32_t last_target_loc_set_ms;

#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLand : public Mode
{
public:
    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLand"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override;

    bool allows_throttle_nudging() const override;

    float get_VTOL_return_radius() const;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    enum class SubMode {
        climb,
        RTL,
    } submode;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QAcro"; }
    const char *name4() const override { return "QACO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

#if QAUTOTUNE_ENABLED
class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAutotune"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    void run() override;

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};
#endif  // QAUTOTUNE_ENABLED

#endif  // HAL_QUADPLANE_ENABLED

class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "Takeoff"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 target_alt;
    AP_Int16 level_alt;
    AP_Float ground_pitch;

protected:
    AP_Int16 target_dist;
    AP_Int8 level_pitch;

    bool takeoff_mode_setup;
    Location start_loc;
    bool runway_circle_set;        // loiter moved to the AUTOLAND runway circle
    uint32_t circle_refresh_ms;

    bool _enter() override;

private:

    // flag that we have already called autoenable fences once in MODE TAKEOFF
    bool have_autoenabled_fences;

};
#if MODE_AUTOLAND_ENABLED
class ModeAutoLand: public Mode
{
public:
    ModeAutoLand();

    Number mode_number() const override { return Number::AUTOLAND; }
    const char *name() const override { return "Autoland"; }
    const char *name4() const override { return "ALND"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }
    
    bool is_landing() const override;
    
    void check_takeoff_direction(void);

    // return true when lined up correctly from the LOITER_TO_ALT
    bool landing_lined_up(void);

    // see if we should capture the direction
    void arm_check(void);

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 final_wp_alt;
    AP_Int16 final_wp_dist;
    AP_Int16 landing_dir_off;
    AP_Int8  options;
    AP_Int16 terrain_alt_min;
    AP_Int16 side_gap;
    AP_Int16 td_offset;
    AP_Int16 desc_arc;
    AP_Float rollout_gain;
    AP_Int8  td_learn;
#if AP_BARO_THST_COMP_ENABLED
    AP_Int8  baro_learn;

    // AUTOLAND_BAROLRN: called every loop
    void baro_learn_update(void);
    // TAKEOFF entered on the ground: measure the baro thrust error on this run-up
    void baro_learn_takeoff_start(void);
#endif

    // Bitfields of AUTOLAND_OPTIONS
    enum class AutoLandOption {
        AUTOLAND_DIR_ON_ARM     = (1U << 0), // set dir for autoland on arm if compass in use.
        LAND_AT_ROLL_START      = (1U << 1), // land at the takeoff roll start point, direction from roll start to liftoff point
        CIRCUIT_AWAY_FROM_HOME  = (1U << 2), // build the approach circuit on the side of the runway away from HOME
    };

    enum class AutoLandStage {
        CLIMB,
        TRANSIT,   // runway options: fly to the base circle holding altitude
        LOITER,
        LANDING
    };

    bool autoland_option_is_set(AutoLandOption option) const {
        return (options & int8_t(option)) != 0;
    }
    // custom runway behaviour (takeoff point landing, pits-side circuit, runway circle, fence turn-back)
    bool runway_options_enabled() const {
        return autoland_option_is_set(AutoLandOption::LAND_AT_ROLL_START) &&
               autoland_option_is_set(AutoLandOption::CIRCUIT_AWAY_FROM_HOME);
    }

    // centre of the "runway circle" used by TAKEOFF loiter and RTL (no rally point):
    // abeam the captured takeoff point, WP_LOITER_RAD + 5m to the side away from HOME.
    // Altitude is left at the takeoff point's ground altitude; callers set it.
    bool get_runway_loiter_center(Location &loc) const;
    // re-centre a loiter waypoint that was set from get_runway_loiter_center()
    // (wind estimate improves after takeoff). Returns true if wp was updated.
    bool refresh_runway_circle(Location &wp) const;
    // radius the aircraft will really fly for a commanded loiter radius (wind, roll limit)
    float flown_circle_radius(float commanded_m, float extra_speed = 0) const;

    // pits side of a line: -1 left, +1 right, 0 unknown (pits = first good GPS fix point since power-up)
    int8_t pits_side(const Location &line_pt, float heading_deg) const;
    bool has_gps_fix_point() const { return gps_fix_loc_valid; }
    bool has_flown_this_arming() const { return flown_this_arming; }
    float get_rollout_gain() const { return rollout_gain; }
    // true once the wheels are on the ground (rangefinder < 0.4m for 0.3s, or not flying)
    bool wheels_on_ground();

    // TAKEOFF pre-check: false (with a CRITICAL message) if the pits side is unknown
    bool takeoff_pits_check(void);

    // hold the approach-start altitude between leaving the base circle and
    // joining the glide slope
    void update_target_altitude() override;
    float final_turn_remaining_deg() const;

protected:
    bool _enter() override;
    AP_Mission::Mission_Command cmd_climb;
    AP_Mission::Mission_Command cmd_loiter;
    AP_Mission::Mission_Command cmd_land;
    Location land_start;
    float approach_hgt;           // height of the circuit / start of final above the touchdown point
    AutoLandStage stage;
    void set_autoland_direction(const float heading);

    // LAND_AT_ROLL_START support
    void check_roll_start_capture(void);
    Location roll_start_loc;      // last stationary position before the takeoff roll
    float roll_start_yaw_deg;     // compass heading while lined up at the roll start
    mutable Location last_circle; // last runway circle centre handed out
    mutable bool last_circle_valid = false;
    bool roll_start_valid = false;
    Location land_point;          // landing target captured at liftoff
    bool land_point_valid = false;
    bool roll_capture_rejected = false; // capture refused for this arming (curved/too long roll)
    void update_gps_fix_point(void);
    Location gps_fix_loc;          // first good GPS fix point since boot (normally the pits)
    bool gps_fix_loc_valid = false;
    bool flown_this_arming = false;
    uint32_t rf_ground_ms;

#if AP_BARO_THST_COMP_ENABLED
    // AUTOLAND_BAROLRN: BARO1_THST_SCALE from the stationary part of the takeoff run-up
    enum class BaroLearnState : uint8_t {
        IDLE,       // armed, parked: keep a baseline of the last second
        WAIT,       // TAKEOFF selected on the ground, waiting for the throttle to rise
        SAMPLING,   // throttle rising, still stationary
        DONE,       // finished for this arming
    };
    enum class BaroLearnResult : uint8_t {
        ACCEPTED = 0,
        TOO_SHORT = 1,
        TOO_NOISY = 2,
        OUT_OF_RANGE = 3,
        NO_BASELINE = 4,
    };
    static constexpr uint8_t BARO_LEARN_BASE_LEN = 20;   // 1 s at 20 Hz
    struct {
        BaroLearnState state;
        float base[BARO_LEARN_BASE_LEN];   // raw BARO1 pressure while parked, Pa
        uint8_t base_n;
        uint8_t base_idx;
        float baseline;
        float base_sd;
        uint32_t last_ms;
        uint32_t start_ms;
        uint32_t end_ms;
        uint16_t n;
        float sum_tp;       // sum of throttle x pressure rise, throttle >= 0.25
        float sum_tt;       // sum of throttle squared, throttle >= 0.25
        uint16_t n_fit;
        float thr_min;
        float thr_max;
        float accel_fwd;    // filtered forward acceleration, m/s/s
        bool save_pending;  // learned value to save at disarm
    } baro_lrn;
    void baro_learn_finish(void);
#endif

    // AUTOLAND_TDLRN: AUTOLAND_TD_OFS from where each landing touches down
    enum class TdLearnState : uint8_t {
        IDLE,       // not landing at the roll start, or learning off
        WATCH,      // AUTOLAND aimed at the roll start: waiting for the touchdown
        TOUCHED,    // touchdown seen: checking it stays down
        DONE,       // finished for this landing
    };
    enum class TdLearnResult : uint8_t {
        ACCEPTED = 0,
        PILOT_INPUT = 1,
        OFF_LINE = 2,
        OUT_OF_RANGE = 3,
        MODE_CHANGE = 4,
        NOT_SEEN = 5,
    };
    struct {
        TdLearnState state;
        bool pilot_input;       // stick input on the final, flare or rollout
        bool save_pending;      // learned value to save at disarm
        uint8_t source;         // touchdown seen by 0 the accelerometer, 1 the rangefinder
        int16_t aim_ofs;        // AUTOLAND_TD_OFS this landing was aimed with
        bool still_loc_pending; // record where the sink next stops
        uint32_t touch_ms;
        uint32_t sink_ms;       // last time sinking faster than 0.3 m/s (or climbing)
        uint32_t spike_ms;      // first impact on the accelerometer since then, 0 = none
        float spike_accel;
        float accel;            // vertical acceleration above 1 g at the touchdown, m/s/s
        Location touch_loc;
        Location spike_loc;
        Location still_loc;     // where the sink stopped
    } td_lrn;
    void td_learn_update(void);
    void td_learn_finish(TdLearnResult res);
};
#endif
#if HAL_SOARING_ENABLED

class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "Thermal"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif
