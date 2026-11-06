#pragma once
/*
  Grbl.h - Single shared header for the label-cutter build of Grbl_ESP32.

  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
  2018 - Bart Dring (ESP32 port)

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <atomic>
#include "Config.h"

const char* const GRBL_VERSION       = "1.3a";
const char* const GRBL_VERSION_BUILD = "LC-20261103";

// ============================================================================
// Basic helpers
// ============================================================================
#define N_AXIS 3
#define MAX_N_AXIS N_AXIS
const int X_AXIS = 0;  // Knife carriage
const int Y_AXIS = 1;  // Sheet transport
const int Z_AXIS = 2;  // Feed roller

const double MM_PER_INCH      = 25.40;
const double INCH_PER_MM      = 0.0393701;
const float  SOME_LARGE_VALUE = 1.0E+38f;
const float  SEC_PER_MIN_SQ   = 3600.0f;
const float  SEC_PER_MIN      = 60.0f;

#undef bit
#define bit(n) (1u << static_cast<unsigned int>(n))
#define bit_true(x, mask) (x) |= (mask)
#define bit_false(x, mask) (x) &= ~(mask)
#define bit_istrue(x, mask) (((x) & (mask)) != 0)
#define bit_isfalse(x, mask) (((x) & (mask)) == 0)
#define clear_vector(a) memset(a, 0, sizeof(a))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define isequal_position_vector(a, b) !(memcmp(a, b, sizeof(float) * N_AXIS))

typedef uint8_t AxisMask;

// ============================================================================
// Error and alarm codes (numbers are part of the serial protocol)
// ============================================================================
enum class Error : uint8_t {
    Ok                          = 0,
    ExpectedCommandLetter       = 1,
    BadNumberFormat             = 2,
    InvalidStatement            = 3,
    NegativeValue               = 4,
    SettingDisabled             = 5,
    SettingStepPulseMin         = 6,
    SettingReadFail             = 7,
    IdleError                   = 8,
    SystemGcLock                = 9,
    SoftLimitError              = 10,
    Overflow                    = 11,
    MaxStepRateExceeded         = 12,
    LineLengthExceeded          = 14,
    TravelExceeded              = 15,
    InvalidJogCommand           = 16,
    HomingNoCycles              = 18,
    GcodeUnsupportedCommand     = 20,
    GcodeModalGroupViolation    = 21,
    GcodeUndefinedFeedRate      = 22,
    GcodeCommandValueNotInteger = 23,
    GcodeAxisCommandConflict    = 24,
    GcodeWordRepeated           = 25,
    GcodeNoAxisWords            = 26,
    GcodeInvalidLineNumber      = 27,
    GcodeValueWordMissing       = 28,
    GcodeUnsupportedCoordSys    = 29,
    GcodeG53InvalidMotionMode   = 30,
    GcodeAxisWordsExist         = 31,
    GcodeNoAxisWordsInPlane     = 32,
    GcodeInvalidTarget          = 33,
    GcodeArcRadiusError         = 34,
    GcodeNoOffsetsInPlane       = 35,
    GcodeUnusedWords            = 36,
    GcodeG43DynamicAxisError    = 37,
    GcodeMaxValueExceeded       = 38,
    PParamMaxExceeded           = 39,
    NumberRange                 = 80,
    InvalidValue                = 81,
    NvsSetFailed                = 100,
    Eol                         = 111,
    JogCancelled                = 130,
    // ---- Label cutter specific ----
    NoPaper               = 150,  // Feed motor motion requested without paper in the tray
    SensorTimeout         = 151,  // A sensor edge was not seen within $303 mm
    PaperOutDuringFeed    = 152,  // Tray ran empty before the sheet reached the rear sensor
    FrontSensorNotCovered = 153,  // Feed step 6 started with the front sensor already clear
    SequenceAborted       = 154,  // Reset / alarm during a feed or eject sequence
};

enum class ExecAlarm : uint8_t {
    None               = 0,
    HardLimit          = 1,
    SoftLimit          = 2,
    AbortCycle         = 3,
    HomingFailReset    = 6,
    HomingFailPulloff  = 8,
    HomingFailApproach = 9,
};

// ============================================================================
// System state
// ============================================================================
enum class State : uint8_t {
    Idle = 0,   // Must be zero
    Alarm,      // Locks out G-code. Settings and $X/$H still allowed.
    CheckMode,  // G-code check mode ($C)
    Homing,
    Cycle,      // Motion is executing (also used by feed / eject sequences)
    Hold,       // Feed hold
    Jog,
};

struct StepControl {
    uint8_t endMotion : 1;
    uint8_t executeHold : 1;
    uint8_t executeSysMotion : 1;
};

struct SuspendBits {
    uint8_t holdComplete : 1;
    uint8_t motionCancel : 1;
    uint8_t jogCancel : 1;
};
union Suspend {
    uint8_t     value;
    SuspendBits bit;
};

typedef struct {
    volatile State state;
    volatile bool  abort;
    Suspend        suspend;
    bool           soft_limit;
    StepControl    step_control;
    volatile AxisMask homing_axis_lock;  // Axes allowed to step during homing
    uint8_t        f_override;
    uint8_t        r_override;
    uint8_t        report_ovr_counter;
    uint8_t        report_wco_counter;
} system_t;
extern system_t sys;

// Realtime executor flags. Set from ISRs / the serial task, consumed by the
// main loop. Atomic read-modify-write so ISRs never lose a flag.
namespace ExecFlag {
    enum : uint32_t {
        StatusReport = bit(0),
        CycleStart   = bit(1),
        FeedHold     = bit(2),
        Reset        = bit(3),
        MotionCancel = bit(4),
    };
}
extern std::atomic<uint32_t> sys_rt_exec_state;
static inline void rt_set(uint32_t f) {
    sys_rt_exec_state.fetch_or(f);
}
static inline void rt_clear(uint32_t f) {
    sys_rt_exec_state.fetch_and(~f);
}
static inline bool rt_is(uint32_t f) {
    return (sys_rt_exec_state.load() & f) != 0;
}

extern volatile ExecAlarm sys_rt_exec_alarm;
extern volatile uint8_t   sys_rt_f_override;
extern volatile uint8_t   sys_rt_r_override;
extern volatile bool      cycle_stop;
extern volatile int32_t   sys_position[N_AXIS];  // Real-time machine position in steps
extern volatile void*     sys_pl_data_inflight;

// ============================================================================
// Settings (Settings.cpp)
// ============================================================================
struct settings_t {
    // Classic Grbl numbers
    uint8_t  pulse_us;          // $0
    uint8_t  idle_lock_ms;      // $1  (255 = keep enabled)
    uint8_t  step_invert;       // $2
    uint8_t  dir_invert;        // $3
    uint8_t  enable_invert;     // $4
    uint8_t  limit_invert;      // $5  0 = X limit active LOW
    uint8_t  status_mask;       // $10
    float    junction_dev;      // $11
    float    arc_tolerance;     // $12
    uint8_t  report_inches;     // $13
    uint8_t  soft_limits;       // $20 (X only)
    uint8_t  hard_limits;       // $21 (X only)
    uint8_t  homing_enable;     // $22 (X only)
    uint8_t  homing_dir_mask;   // $23 bit0 set = X switch at the negative end (X = 0)
    float    homing_feed;       // $24
    float    homing_seek;       // $25
    uint16_t homing_debounce;   // $26
    float    homing_pulloff;    // $27
    float    head_s_max;        // $30 S value that equals 100 % head power
    float    steps_per_mm[N_AXIS];  // $100-$102
    float    max_rate[N_AXIS];      // $110-$112
    float    acceleration[N_AXIS];  // $120-$122 (mm/s^2)
    float    x_max_travel;          // $130 X soft limit: 0 <= X <= $130
    // Label cutter
    float    feed_dr;            // $300 DR: feed+Y travel after the rear sensor triggers
    float    feed_df;            // $301 DF: Y travel after the front sensor clears (signed)
    float    seq_speed;          // $302 Sequence paper speed (mm/min)
    float    seq_max_search;     // $303 Max travel while searching a sensor edge
    float    eject_extra;        // $304 Extra Y travel after both sensors clear
    uint8_t  sensor_invert;      // $305 bit0 paper, bit1 rear, bit2 front
    uint8_t  zero_after_feed;    // $306 Set MPos Y=0 (and Z=0) after a feed
    uint16_t head_pullin_ms;     // $307 Full-power pull-in time for the solenoids
    uint16_t head_switch_ms;     // $308 Release delay when switching heads
    uint8_t  dir_delay_us;       // $309 Delay between DIR change and STEP
};
extern settings_t settings;

enum CoordIndex : uint8_t {
    G54 = 0,
    G55,
    G56,
    G57,
    G58,
    G59,
    NWCSystems,
    G28 = NWCSystems,
    G30,
    CoordEnd,
};
extern float coord_data_store[CoordEnd][N_AXIS];

void  settings_init();
Error settings_store_coord(uint8_t idx, const float* data);
void  settings_read_coord(uint8_t idx, float* data);
Error system_execute_line(char* line, uint8_t client);
void  system_execute_startup(char* line);
bool  read_float(const char* line, uint8_t* char_counter, float* float_ptr);

// ============================================================================
// G-code parser (GCode.cpp)
// ============================================================================
enum class ModalGroup : uint8_t {
    MG0  = 0,   // Non-modal
    MG1  = 1,   // Motion
    MG2  = 2,   // Plane
    MG3  = 3,   // Distance
    MG4  = 4,   // Arc IJK distance
    MG5  = 5,   // Feed rate mode
    MG6  = 6,   // Units
    MG7  = 7,   // Cutter comp
    MG8  = 8,   // Tool length offset
    MG12 = 9,   // Coordinate system
    MG13 = 10,  // Control mode
    MM4  = 11,  // Stopping
    MM7  = 12,  // Heads (M3/M4/M5)
    MM6  = 13,  // Tool change
    MM10 = 14,  // Fan outputs (M67/M68)
};
enum class NonModal : uint8_t {
    NoAction              = 0,
    Dwell                 = 4,
    SetCoordinateData     = 10,
    GoHome0               = 28,
    SetHome0              = 38,
    GoHome1               = 30,
    SetHome1              = 40,
    AbsoluteOverride      = 53,
    SetCoordinateOffset   = 92,
    ResetCoordinateOffset = 102,
};
enum class Motion : uint8_t {
    Seek   = 0,
    Linear = 1,
    CwArc  = 2,
    CcwArc = 3,
    None   = 80,
};
enum class Plane : uint8_t { XY = 0, ZX = 1, YZ = 2 };
enum class Distance : uint8_t { Absolute = 0, Incremental = 1 };
enum class ProgramFlow : uint8_t { Running = 0, Paused = 3, OptionalStop = 1, CompletedM2 = 2, CompletedM30 = 30 };
enum class FeedRate : uint8_t { UnitsPerMin = 0, InverseTime = 1 };
enum class Units : uint8_t { Mm = 0, Inches = 1 };
enum class ToolLengthOffset : uint8_t { Cancel = 0, EnableDynamic = 1 };
// Head selection is carried in the classic Grbl "spindle" modal group:
// M3 = head 1, M4 = head 2, M5 = both heads up.
enum class HeadState : uint8_t { Off = 0, Head1 = 1, Head2 = 2 };
enum class IoControl : uint8_t { None = 0, SetAnalogSync = 5, SetAnalogImmediate = 6 };

enum class GCodeWord : uint8_t { E = 0, F, I, J, K, L, N, P, Q, R, S, T, X, Y, Z };

typedef struct {
    Motion           motion;
    FeedRate         feed_rate;
    Units            units;
    Distance         distance;
    Plane            plane_select;
    ToolLengthOffset tool_length;
    CoordIndex       coord_select;
    ProgramFlow      program_flow;
    HeadState        head;
    IoControl        io_control;
} gc_modal_t;

typedef struct {
    uint8_t e;
    float   f;
    float   ijk[3];
    uint8_t l;
    int32_t n;
    float   p;
    float   q;
    float   r;
    float   s;
    uint8_t t;
    float   xyz[N_AXIS];
} gc_values_t;

typedef struct {
    gc_modal_t modal;
    float      head_power;  // S word (0..$30)
    float      feed_rate;
    uint8_t    tool;
    int32_t    line_number;
    float      position[N_AXIS];
    float      coord_system[N_AXIS];
    float      coord_offset[N_AXIS];
    float      tool_length_offset;
} parser_state_t;
extern parser_state_t gc_state;

typedef struct {
    NonModal    non_modal_command;
    gc_modal_t  modal;
    gc_values_t values;
} parser_block_t;

void  gc_init();
Error gc_execute_line(char* line, uint8_t client);
void  gc_sync_position();

// ============================================================================
// Planner (Planner.cpp)
// ============================================================================
struct PlMotion {
    uint8_t rapidMotion : 1;
    uint8_t systemMotion : 1;
    uint8_t noFeedOverride : 1;
    uint8_t inverseTime : 1;
};

typedef struct {
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    uint8_t  direction_bits;
    PlMotion motion;
    int32_t  line_number;
    float    entry_speed_sqr;
    float    max_entry_speed_sqr;
    float    acceleration;
    float    millimeters;
    float    max_junction_speed_sqr;
    float    rapid_rate;
    float    programmed_rate;
} plan_block_t;

typedef struct {
    float    feed_rate;
    PlMotion motion;
    int32_t  line_number;
    bool     is_jog;
} plan_line_data_t;

void          plan_reset();
void          plan_reset_buffer();
bool          plan_buffer_line(float* target, plan_line_data_t* pl_data);
void          plan_discard_current_block();
plan_block_t* plan_get_system_motion_block();
plan_block_t* plan_get_current_block();
uint8_t       plan_next_block_index(uint8_t block_index);
float         plan_get_exec_block_exit_speed_sqr();
float         plan_compute_profile_nominal_speed(plan_block_t* block);
void          plan_update_velocity_profile_parameters();
void          plan_sync_position();
void          plan_cycle_reinitialize();
uint8_t       plan_get_block_buffer_available();
bool          plan_check_full_buffer();

// ============================================================================
// Stepper (Stepper.cpp)
// ============================================================================
void     stepper_init();
void     st_wake_up();
void     st_go_idle();
void     st_reset();
void     st_prep_buffer();
void     st_update_plan_block_parameters();
float    st_get_realtime_rate();
void     st_disable_motors(bool disable);
void     st_check_idle_disable();
AxisMask st_moving_axes();  // Axes stepping in the block being executed

// ============================================================================
// Motion control, homing, limits, jog (MotionControl.cpp)
// ============================================================================
bool  mc_line(float* target, plan_line_data_t* pl_data);
void  mc_arc(float* target, plan_line_data_t* pl_data, float* position, float* offset, float radius,
             uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, bool is_clockwise_arc);
bool  mc_dwell(int32_t milliseconds);
void  mc_homing_cycle();
void  mc_reset();
Error jog_execute(plan_line_data_t* pl_data, parser_block_t* gc_block, bool* cancelledInflight);
void  limits_init();
bool  limits_x_triggered();
bool  limits_check_travel(const float* target);  // true if outside the X soft limits
void  limits_check_soft(float* target);
bool  delay_msec(int32_t milliseconds);

// ============================================================================
// Protocol (Protocol.cpp)
// ============================================================================
void protocol_main_loop();
void protocol_buffer_synchronize();
void protocol_auto_cycle_start();
void protocol_execute_realtime();
void protocol_exec_rt_system();
void protocol_notify();          // Wake the main loop (task context)
void protocol_notify_from_isr(); // Wake the main loop (ISR context)
void protocol_wait(uint32_t ticks);

// ============================================================================
// Serial (Serial.cpp)
// ============================================================================
const uint8_t CLIENT_CONSOLE = 0;
const uint8_t CLIENT_MASTER  = 1;
const uint8_t CLIENT_COUNT   = 2;
const uint8_t CLIENT_ALL     = 0xFF;

enum class Cmd : uint8_t {
    Reset              = 0x18,  // Ctrl-X
    StatusReport       = '?',
    CycleStart         = '~',
    FeedHold           = '!',
    JogCancel          = 0x85,
    FeedOvrReset       = 0x90,
    FeedOvrCoarsePlus  = 0x91,
    FeedOvrCoarseMinus = 0x92,
    FeedOvrFinePlus    = 0x93,
    FeedOvrFineMinus   = 0x94,
    RapidOvrReset      = 0x95,
    RapidOvrMedium     = 0x96,
    RapidOvrLow        = 0x97,
};

void serial_init();
int  serial_read(uint8_t client);
void serial_reset_read_buffer(uint8_t client);
int  serial_rx_available(uint8_t client);
bool serial_take_overflow(uint8_t client);
void serial_write(uint8_t client, const char* text);

// ============================================================================
// Reports (Report.cpp)
// ============================================================================
enum class Message : uint8_t {
    CriticalEvent = 1,
    AlarmLock,
    AlarmUnlock,
    Enabled,
    Disabled,
    CheckLimits,
    ProgramEnd,
    RestoreDefaults,
};

void grbl_send(uint8_t client, const char* text);
void grbl_sendf(uint8_t client, const char* format, ...) __attribute__((format(printf, 2, 3)));
void grbl_msg_sendf(uint8_t client, const char* format, ...) __attribute__((format(printf, 2, 3)));
void report_status_message(Error status_code, uint8_t client);
void report_alarm_message(ExecAlarm alarm_code);
void report_feedback_message(Message message);
void report_init_message(uint8_t client);
void report_grbl_help(uint8_t client);
void report_grbl_settings(uint8_t client);
void report_ngc_parameters(uint8_t client);
void report_gcode_modes(uint8_t client);
void report_startup_line(uint8_t n, const char* line, uint8_t client);
void report_execute_startup_message(const char* line, Error status_code, uint8_t client);
void report_build_info(uint8_t client);
void report_realtime_status(uint8_t client);
void system_convert_array_steps_to_mpos(float* position, const volatile int32_t* steps);
float* system_get_mpos();
void system_flag_wco_change();

// ============================================================================
// Label cutter machine I/O and sequences (Machine.cpp)
// ============================================================================
enum SensorBit : uint8_t {
    SENSOR_PAPER = bit(0),  // Paper present in the tray
    SENSOR_REAR  = bit(1),  // Rear sensor covered
    SENSOR_FRONT = bit(2),  // Front sensor covered
};

void    machine_init();
void    machine_reset();                      // Called on every soft reset
uint8_t machine_sensors();                    // SENSOR_* bits, 1 = paper detected
bool    machine_paper_present();
void    heads_set(HeadState head, float power);  // Mutually exclusive solenoid control
void    heads_sync(HeadState head, float power); // Waits for queued motion first
void    heads_kill();                         // Immediate, ISR-safe heads up
HeadState heads_get_state();
bool    fans_set(uint8_t index, float percent);
float   fans_get(uint8_t index);
Error   machine_feed_sequence();
Error   machine_eject_sequence();
const char* machine_sequence_name();          // NULL when no sequence is running
uint8_t machine_sequence_step();
void    machine_poll();                       // Deferred messages from ISRs
bool    machine_feed_resume_allowed();
