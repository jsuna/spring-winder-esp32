// ------ Machine Configuration ------
#define MOTOR_STEPS_PER_REV     200    // 1.8 deg steppers
#define MICROSTEPPING             8    // match your TMC2209 MS1/MS2 config
#define SPINDLE_GEAR_RATIO      3.0    // motor revs per spindle-shaft rev
#define LEADSCREW_GEAR_RATIO    0.815  // motor revs per leadscrew-shaft rev — current as-built gearing: 16T<->30T jackshaft, 46T<->20T leadscrew
#define SCREW_LEAD_MM             2.0   // mm of carriage travel per leadscrew-shaft rev

#define JOG_FREQ_HZ            1000.0   // step rate while jogging — tune to taste
#define HOMING_FREQ_HZ         1000.0

#define SPINDLE_CH     0 // timer 1 - channels 0-1
#define LEADSCREW_CH   2 // timer 2 - channels 2-3

#define MAX_SEGMENTS   32

// ------ Direction Logic ------
#define SPINDLE_CW LOW
#define SPINDLE_CCW HIGH
#define LEADSCREW_CW HIGH
#define LEADSCREW_CCW LOW
#define HOME_DIR LEADSCREW_CCW   // toward home switch
#define FAR_DIR  LEADSCREW_CW   // toward far switch
#define BACKOFF_TIMEOUT_MS 5000 // length of time to back off after hitting the home/far switch, before stopping the leadscrew

// -------------------- WIFI / NETWORK --------------------
const char* WIFI_SSID = "Tipsytoes";
const char* WIFI_PASS = "tipsy123!";
const char* MDNS_NAME = "springwinder";   // reachable at springwinder.local