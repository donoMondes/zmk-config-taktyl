#include <zephyr/device.h>

#define IQS5XX_NUM_FINGERS 0x0011
#define IQS5XX_REL_X 0x0012          // 2 bytes.
#define IQS5XX_REL_Y 0x0014          // 2 bytes.
#define IQS5XX_ABS_X 0x0016          // 2 bytes.
#define IQS5XX_ABS_Y 0x0018          // 2 bytes.
#define IQS5XX_TOUCH_STRENGTH 0x001A // 2 bytes.
#define IQS5XX_TOUCH_AREA 0x001C

#define IQS5XX_BOTTOM_BETA 0x0637
#define IQS5XX_STATIONARY_THRESH 0x0672

#define IQS5XX_END_COMM_WINDOW 0xEEEE

#define IQS5XX_SYSTEM_CONTROL_0 0x0431
// System Control 0 bits.
#define IQS5XX_ACK_RESET BIT(7)
#define IQS5XX_AUTO_ATI BIT(5)
#define IQS5XX_ALP_RESEED BIT(4)
#define IQS5XX_RESEED BIT(3)

// System ontrol 1 bits
#define IQS5XX_SYSTEM_CONTROL_1 0x0432
#define IQS5XX_ACK_RESET BIT(7)
#define IQS5XX_AUTO_ATI BIT(5)
#define IQS5XX_ALP_RESEED BIT(4)
#define IQS5XX_RESEED BIT(3)

// Report rate for the active mode
#define IQS5XX_REPORT_RATE_ACTIVE_MODE 0x057A // 2 bytes
#define IQS5XX_REPORT_RATE_IDLE_TOUCH_MODE 0x057C // 2 bytes
#define IQS5XX_REPORT_RATE_IDLE_MODE 0x057E // 2 bytes

#define IQS5XX_SYSTEM_CONFIG_0 0x058E
// System Config 0 bits.
#define IQS5XX_MANUAL_CONTROL BIT(7)
#define IQS5XX_SETUP_COMPLETE BIT(6)
#define IQS5XX_WDT BIT(5)
#define IQS5XX_SW_INPUT_EVENT BIT(4)
#define IQS5XX_ALP_REATI BIT(3)
#define IQS5XX_REATI BIT(2)
#define IQS5XX_SW_INPUT_SELECT BIT(1)
#define IQS5XX_SW_INPUT BIT(0)

#define IQS5XX_SYSTEM_CONFIG_1 0x058F
// System Config 1 bits.
#define IQS5XX_EVENT_MODE BIT(0)
#define IQS5XX_GESTURE_EVENT BIT(1)
#define IQS5XX_TP_EVENT BIT(2)
#define IQS5XX_REATI_EVENT BIT(3)
#define IQS5XX_ALP_PROX_EVENT BIT(4)
#define IQS5XX_SNAP_EVENT BIT(5)
#define IQS5XX_TOUCH_EVENT BIT(6)
#define IQS5XX_PROX_EVENT BIT(7)

// Filter settings register.
#define IQS5XX_FILTER_SETTINGS 0x0632
// Filter settings bits.
#define IQS5XX_IIR_FILTER BIT(0)
#define IQS5XX_MAV_FILTER BIT(1)
#define IQS5XX_IIR_SELECT BIT(2)
#define IQS5XX_ALP_COUNT_FILTER BIT(3)

// Max touch point
#define IQS5XX_MAX_MULTI_TOUCHES 0x066A

// Palm reject treshold area size
#define IQS5XX_PALM_REJECT_THRESHOLD 0x066C

// Palm reject disable timeout
#define IQS5XX_PALM_REJECT_TIMEOUT 0x066D

#define IQS5XX_SYSTEM_INFO_0 0x000F

#define IQS5XX_SYSTEM_INFO_1 0x0010

// These 2 registers have the same bit map.
// The first one configures the gestures,
// the second one reports gesture events at runtime.
#define IQS5XX_SINGLE_FINGER_GESTURES_CONF 0x06B7
#define IQS5XX_GESTURE_EVENTS_0 0x000D
// Single finger gesture identifiers.
#define IQS5XX_SINGLE_TAP BIT(0)
#define IQS5XX_PRESS_AND_HOLD BIT(1)
#define IQS5XX_SWIPE_LEFT BIT(2)
#define IQS5XX_SWIPE_RIGHT BIT(3)
#define IQS5XX_SWIPE_UP BIT(4)
#define IQS5XX_SWIPE_DOWN BIT(5)

// Time in ms, 2 registers wide.
// Hold time + tap time is used as
// a threshold for the press and
// hold gesture.
// TODO: Make hold time configurable with KConfig.
#define IQS5XX_TAP_TIME                 0x06B9
#define IQS5XX_TAP_DISTANCE             0x06BB
#define IQS5XX_HOLD_TIME                0x06BD
#define IQS5XX_SWIPE_INIT_TIME          0x06BF
#define IQS5XX_SWIPE_INIT_DISTANCE      0x06C1
#define IQS5XX_SWIPE_CONSEC_TIME        0x06C3
#define IQS5XX_SWIPE_CONSEC_DISTANCE    0x06C5
#define IQS5XX_SCROLL_INIT_DISTANCE     0x06C8
#define IQS5XX_ZOOM_INIT_DISTANCE       0x06CB
#define IQS5XX_ZOOM_CONSEC_DISTANCE     0x06CD

// Mouse button helpers.
#define LEFT_BUTTON_BIT BIT(0)
#define RIGHT_BUTTON_BIT BIT(1)
#define MIDDLE_BUTTON_BIT BIT(2)
#define LEFT_BUTTON_CODE INPUT_BTN_0
#define RIGHT_BUTTON_CODE INPUT_BTN_0 + 1
#define MIDDLE_BUTTON_CODE INPUT_BTN_0 + 2

// These 2 registers have the same bit map.
// The first one configures the gestures,
// the second one reports gesture events at runtime.
#define IQS5XX_MULTI_FINGER_GESTURES_CONF 0x06B8
#define IQS5XX_GESTURE_EVENTS_1 0x000E
// Multi finger gesture identifiers.
#define IQS5XX_TWO_FINGER_TAP BIT(0)
#define IQS5XX_SCROLL BIT(1)
#define IQS5XX_ZOOM BIT(2)

// Resolution configuration
#define IQS5XX_RESOLUTION_X 0x066E
#define IQS5XX_RESOLUTION_Y 0x0670

// Axes configuration.
#define IQS5XX_XY_CONFIG_0 0x0669
#define IQS5XX_FLIP_X BIT(0)
#define IQS5XX_FLIP_Y BIT(1)
#define IQS5XX_SWITCH_XY_AXIS BIT(2)
#define IQS5XX_ALLOW_PALM_REJECT BIT(3)

#define IQS5XX_NEXT_TOUCH_OFFSET 0x7

#define IQS5XX_INPUT_MAX_TOUCHES 5


// default values
#define DEFAULT_TAP_TIME 0x96
#define DEFAULT_TAP_DISTANCE 0x19 
#define DEFAULT_HOLD_TIME 0x12C 
#define DEFAULT_SWIPE_INITIAL_TIME 0x64 // 0x96 
#define DEFAULT_SWIPE_INITIAL_DISTANCE 0x12C 
#define DEFAULT_SWIPE_CONSECUTIVE_TIME 0x0 
#define DEFAULT_SWIPE_CONSECUTIVE_DISTANCE 0x7D0 
#define DEFAULT_SCROLL_INITIAL_DISTANCE 0x32 
#define DEFAULT_ZOOM_INITIAL_DISTANCE 0x32 
#define DEFAULT_ZOOM_CONSECUTIVE_DISTANCE 0x19

typedef struct iqs5xx_sys_info_1{
    bool tp_movement         :1;
    bool palm_detect         :1;
    bool too_many_fingers    :1;
    bool report_missed       :1;
    bool snap_toggle         :1;
    bool switch_state        :1;
    uint8_t spare            :2;
};

typedef struct iqs5xx_sys_info_0{
    uint8_t current_mode     :3;
    bool ati_error           :1;
    bool reati_occured       :1;
    bool alp_ati_error       :1;
    bool alp_reati_occured   :1;
    bool show_reset          :1;
};

typedef struct iqs5xx_sys_info{
    iqs5xx_sys_info_0 sys_inf0;
    iqs5xx_sys_info_1 sys_inf1;
};

typedef struct iqs5xx_sys_control_0{
    uint8_t mode_select :3;
    bool    reseed      :1;
    bool    alp_reseed  :1;
    bool    auto_ati    :1;
    bool    unused      :1;
    bool    ack_reset   :1;
};

typedef struct iqs5xx_sys_control_1{
    bool    suspend     :1;
    bool    reset       :1;
    bool    unused      :6; 
};

typedef struct iqs5xx_sys_control{
    iqs5xx_sys_control_0 sys_ctrl0;
    iqs5xx_sys_control_1 sys_ctrl1;
};

struct iqs5xx_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec rdy_gpio;
    struct gpio_dt_spec reset_gpio;

    // Gesture configuration.
    bool one_finger_tap;
    bool press_and_hold;
    bool two_finger_tap;
    uint16_t press_and_hold_time;

    // Scrolling configuration.
    bool scroll;
    bool natural_scroll_x;
    bool natural_scroll_y;

    // Axes configuration.
    bool switch_xy;
    bool flip_x;
    bool flip_y;

    // Report rate config
    uint16_t report_rate_active_mode;
    
    // Palm rejection
    bool palm_rejection;
    uint8_t palm_reject_threshold;
    uint8_t palm_reject_timeout;
    
    // Maximum touch points
    uint8_t max_touch_number;

    // Resolution
    uint16_t resolution_x;
    uint16_t resolution_y;
    
    // Sensitivity. configuration.
    uint8_t bottom_beta;
    uint8_t stationary_threshold;
};

struct iqs5xx_data {
    const struct device *dev;
    struct gpio_callback rdy_cb;
    struct k_work work;
    struct k_work_delayable button_release_work;
    // TODO: Pack flags into a bitfield to save space.
    bool initialized;
    // Flag to indicate if the button was pressed in a previous cycle.
    uint8_t buttons_pressed;
    bool active_hold;
    // Scroll accumulators.
    int16_t scroll_x_acc;
    int16_t scroll_y_acc;
};

struct iqs5xx_point_data {
    uint8_t id;     //Touch id
    uint16_t abs_x; //Touch absolute position on the x axis
    uint16_t abs_y; //Touch absolute position on the y axis
    uint8_t size;  //Touch area/size
};
