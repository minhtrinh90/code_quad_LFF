// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
#ifndef __ARDUCOPTER_CONFIG_H__
#define __ARDUCOPTER_CONFIG_H__
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
//
//  DO NOT EDIT this file to adjust your configuration.  Create your own
//  APM_Config.h and use APM_Config.h.example as a reference.
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
///
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// Default and automatic configuration details.
//
// Notes for maintainers:
//
// - Try to keep this file organised in the same order as APM_Config.h.example
//
#include "defines.h"

///
/// DO NOT EDIT THIS INCLUDE - if you want to make a local change, make that
/// change in your local copy of APM_Config.h.
///
#ifdef USE_CMAKE_APM_CONFIG
 #include "APM_Config_cmake.h"  // <== Prefer cmake config if it exists
#else
 #include "APM_Config.h" // <== THIS INCLUDE, DO NOT EDIT IT. EVER.
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_APM_HARDWARE
#error CONFIG_APM_HARDWARE option is deprecated! use CONFIG_HAL_BOARD instead
#endif

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduCopter
#endif

#ifdef __AVR_ATmega1280__
#error ATmega1280 is not supported
#endif
//////////////////////////////////////////////////////////////////////////////
// APM2 HARDWARE DEFAULTS
//

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define CONFIG_IMU_TYPE   CONFIG_IMU_MPU6000
 # define CONFIG_PUSHBUTTON DISABLED
 # define CONFIG_RELAY      DISABLED
 # define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN
 # define MAGNETOMETER ENABLED
 # ifdef APM2_BETA_HARDWARE
  #  define CONFIG_BARO     AP_BARO_BMP085
 # else // APM2 Production Hardware (default)
  #  define CONFIG_BARO          AP_BARO_MS5611
  #  define CONFIG_MS5611_SERIAL AP_BARO_MS5611_SPI
 # endif
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 # define CONFIG_IMU_TYPE   CONFIG_IMU_SITL
 # define CONFIG_PUSHBUTTON DISABLED
 # define CONFIG_RELAY      DISABLED
 # define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN
 # define MAGNETOMETER ENABLED
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
 # define CONFIG_IMU_TYPE   CONFIG_IMU_PX4
 # define CONFIG_BARO       AP_BARO_PX4
 # define CONFIG_PUSHBUTTON DISABLED
 # define CONFIG_RELAY      DISABLED
 # define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN
 # define MAGNETOMETER ENABLED
#elif CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
 # define CONFIG_IMU_TYPE   CONFIG_IMU_MPU6000
 # define CONFIG_BARO       AP_BARO_MS5611
 # define CONFIG_MS5611_SERIAL AP_BARO_MS5611_I2C
 # define CONFIG_ADC        DISABLED
 # define CONFIG_PUSHBUTTON DISABLED
 # define CONFIG_RELAY      DISABLED
 # define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN
 # define MAGNETOMETER ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// FRAME_CONFIG
//
#ifndef FRAME_CONFIG
 # define FRAME_CONFIG   QUAD_FRAME
#endif
#ifndef FRAME_ORIENTATION
 # define FRAME_ORIENTATION      X_FRAME
#endif
#ifndef TOY_EDF
 # define TOY_EDF        DISABLED
#endif
#ifndef TOY_MIXER
 # define TOY_MIXER      TOY_LINEAR_MIXER
#endif

/////////////////////////////////////////////////////////////////////////////////
// Bulk defines for TradHeli
#if FRAME_CONFIG == HELI_FRAME
  # define RC_FAST_SPEED 				125
  # define WP_YAW_BEHAVIOR_DEFAULT      YAW_LOOK_AT_HOME
  # define RATE_INTEGRATOR_LEAK_RATE 	0.02f
  # define RATE_ROLL_D    				0
  # define RATE_PITCH_D       			0
  # define HELI_PITCH_FF				0
  # define HELI_ROLL_FF					0
  # define HELI_YAW_FF					0  
  # define STABILIZE_THROTTLE			THROTTLE_MANUAL
  # define MPU6K_FILTER                 10
#endif


// optical flow doesn't work in SITL yet
#ifdef DESKTOP_BUILD
# define OPTFLOW DISABLED
#endif


//////////////////////////////////////////////////////////////////////////////
// IMU Selection
//
#ifndef CONFIG_IMU_TYPE
 # define CONFIG_IMU_TYPE CONFIG_IMU_OILPAN
#endif
#ifndef MPU6K_FILTER
 # define MPU6K_FILTER MPU6K_DEFAULT_FILTER
#endif

//////////////////////////////////////////////////////////////////////////////
// ADC Enable - used to eliminate for systems which don't have ADC.
//
#ifndef CONFIG_ADC
 # if CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
  #   define CONFIG_ADC ENABLED
 # else
  #   define CONFIG_ADC DISABLED
 # endif
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
// default RC speed in Hz
#ifndef RC_FAST_SPEED
   #   define RC_FAST_SPEED 490
#endif

////////////////////////////////////////////////////////
// LED and IO Pins
//
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
 # define A_LED_PIN        37
 # define B_LED_PIN        36
 # define C_LED_PIN        35
 # define LED_ON           HIGH
 # define LED_OFF          LOW
 # define PUSHBUTTON_PIN   41
 # define USB_MUX_PIN      -1
 # define BATTERY_VOLT_PIN      0      // Battery voltage on A0
 # define BATTERY_CURR_PIN      1      // Battery current on A1
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define A_LED_PIN        27
 # define B_LED_PIN        26
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
 # define PUSHBUTTON_PIN   (-1)
 # define USB_MUX_PIN      23
 # define BATTERY_VOLT_PIN      1      // Battery voltage on A1
 # define BATTERY_CURR_PIN      2      // Battery current on A2
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 # define A_LED_PIN        27
 # define B_LED_PIN        26
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
 # define PUSHBUTTON_PIN   (-1)
 # define USB_MUX_PIN      -1
 # define BATTERY_VOLT_PIN 1      // Battery voltage on A1
 # define BATTERY_CURR_PIN 2      // Battery current on A2
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
 # define A_LED_PIN        27
 # define B_LED_PIN        26
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
 # define PUSHBUTTON_PIN   (-1)
 # define USB_MUX_PIN      -1
 # define BATTERY_VOLT_PIN -1
 # define BATTERY_CURR_PIN -1
#elif CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
// XXX these are just copied, may not make sense
 # define A_LED_PIN        27
 # define B_LED_PIN        26
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
 # define PUSHBUTTON_PIN   (-1)
 # define USB_MUX_PIN      -1
 # define BATTERY_VOLT_PIN -1
 # define BATTERY_CURR_PIN -1
#endif

////////////////////////////////////////////////////////////////////////////////
// CopterLEDs
//

#ifndef COPTER_LEDS
 #define COPTER_LEDS ENABLED
#endif

#define COPTER_LED_ON           HIGH
#define COPTER_LED_OFF          LOW

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
 #define COPTER_LED_1 AN4       // Motor or Aux LED
 #define COPTER_LED_2 AN5       // Motor LED or Beeper
 #define COPTER_LED_3 AN6       // Motor or GPS LED
 #define COPTER_LED_4 AN7       // Motor LED
 #define COPTER_LED_5 AN8       // Motor LED
 #define COPTER_LED_6 AN9       // Motor LED
 #define COPTER_LED_7 AN10      // Motor LED
 #define COPTER_LED_8 AN11      // Motor LED
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL || CONFIG_HAL_BOARD == HAL_BOARD_PX4 || HAL_BOARD_SMACCM
 #define COPTER_LED_1 AN8       // Motor or Aux LED
 #define COPTER_LED_2 AN9       // Motor LED
 #define COPTER_LED_3 AN10      // Motor or GPS LED
 #define COPTER_LED_4 AN11      // Motor LED
 #define COPTER_LED_5 AN12      // Motor LED
 #define COPTER_LED_6 AN13      // Motor LED
 #define COPTER_LED_7 AN14      // Motor LED
 #define COPTER_LED_8 AN15      // Motor LED
#endif


//////////////////////////////////////////////////////////////////////////////
// Pushbutton & Relay
//

#ifndef CONFIG_PUSHBUTTON
 # define CONFIG_PUSHBUTTON ENABLED
#endif

#ifndef CONFIG_RELAY
 # define CONFIG_RELAY ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Barometer
//

#ifndef CONFIG_BARO
 # define CONFIG_BARO AP_BARO_BMP085
#endif

//////////////////////////////////////////////////////////////////////////////
// Sonar
//

#ifndef CONFIG_SONAR_SOURCE
 # define CONFIG_SONAR_SOURCE SONAR_SOURCE_ADC
#endif

#if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC && CONFIG_ADC == DISABLED
 # warning Cannot use ADC for CONFIG_SONAR_SOURCE, becaude CONFIG_ADC is DISABLED
 # warning Defaulting CONFIG_SONAR_SOURCE to ANALOG_PIN
 # undef CONFIG_SONAR_SOURCE
 # define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN
#endif

#if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
 # ifndef CONFIG_SONAR_SOURCE_ADC_CHANNEL
  #  define CONFIG_SONAR_SOURCE_ADC_CHANNEL 7
 # endif
#elif CONFIG_SONAR_SOURCE == SONAR_SOURCE_ANALOG_PIN
 # ifndef CONFIG_SONAR_SOURCE_ANALOG_PIN
  #  define CONFIG_SONAR_SOURCE_ANALOG_PIN 0
 # endif
#else
 # warning Invalid value for CONFIG_SONAR_SOURCE, disabling sonar
 # define CONFIG_SONAR DISABLED
#endif

#ifndef CONFIG_SONAR
 # define CONFIG_SONAR ENABLED
#endif

#ifndef SONAR_ALT_HEALTH_MAX
 # define SONAR_ALT_HEALTH_MAX 3            // number of good reads that indicates a healthy sonar
#endif

#ifndef SONAR_GAIN_DEFAULT
 # define SONAR_GAIN_DEFAULT 0.2            // gain for controlling how quickly sonar range adjusts target altitude (lower means slower reaction)
#endif

#ifndef THR_SURFACE_TRACKING_VELZ_MAX
 # define THR_SURFACE_TRACKING_VELZ_MAX 30  // max vertical speed change while surface tracking with sonar
#endif

//////////////////////////////////////////////////////////////////////////////
// Channel 7 and 8 default options
//

#ifndef CH7_OPTION
 # define CH7_OPTION            AUX_SWITCH_SAVE_WP
#endif

#ifndef CH8_OPTION
 # define CH8_OPTION            AUX_SWITCH_DO_NOTHING
#endif

//////////////////////////////////////////////////////////////////////////////
// HIL_MODE                                 OPTIONAL

#ifndef HIL_MODE
 #define HIL_MODE        HIL_MODE_DISABLED
#endif

#if HIL_MODE != HIL_MODE_DISABLED       // we are in HIL mode

 # undef GPS_PROTOCOL
 # define GPS_PROTOCOL GPS_PROTOCOL_NONE

 #undef CONFIG_SONAR
 #define CONFIG_SONAR DISABLED
#endif


//////////////////////////////////////////////////////////////////////////////
// GPS_PROTOCOL
//
#ifndef GPS_PROTOCOL
 # define GPS_PROTOCOL           GPS_PROTOCOL_AUTO
#endif


#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif


//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
#ifndef SERIAL0_BAUD
 # define SERIAL0_BAUD                   115200
#endif
#ifndef SERIAL3_BAUD
 # define SERIAL3_BAUD                    57600
#endif


//////////////////////////////////////////////////////////////////////////////
// Battery monitoring
//
#ifndef LOW_VOLTAGE
 # define LOW_VOLTAGE                    10.5f
#endif
#ifndef VOLT_DIV_RATIO
 # define VOLT_DIV_RATIO                 3.56f
#endif

#ifndef CURR_AMP_PER_VOLT
 # define CURR_AMP_PER_VOLT              27.32f
#endif
#ifndef CURR_AMPS_OFFSET
 # define CURR_AMPS_OFFSET               0.0f
#endif
#ifndef HIGH_DISCHARGE
 # define HIGH_DISCHARGE                 1760
#endif

#ifndef BOARD_VOLTAGE_MIN
 # define BOARD_VOLTAGE_MIN             4300        // min board voltage in milli volts for pre-arm checks
#endif

#ifndef BOARD_VOLTAGE_MAX
 # define BOARD_VOLTAGE_MAX             5800        // max board voltage in milli volts for pre-arm checks
#endif

// Battery failsafe
#ifndef FS_BATTERY
 # define FS_BATTERY              DISABLED
#endif

// GPS failsafe
#ifndef FS_GPS
 # define FS_GPS                        ENABLED
#endif
#ifndef FAILSAFE_GPS_TIMEOUT_MS
 # define FAILSAFE_GPS_TIMEOUT_MS       5000    // gps failsafe triggers after 5 seconds with no GPS
#endif

// GCS failsafe
#ifndef FS_GCS
 # define FS_GCS                        DISABLED
#endif
#ifndef FS_GCS_TIMEOUT_MS
 # define FS_GCS_TIMEOUT_MS             5000    // gcs failsafe triggers after 5 seconds with no GCS heartbeat
#endif
// possible values for FS_GCS parameter
#define FS_GCS_DISABLED                     0
#define FS_GCS_ENABLED_ALWAYS_RTL           1
#define FS_GCS_ENABLED_CONTINUE_MISSION     2

//////////////////////////////////////////////////////////////////////////////
//  MAGNETOMETER
#ifndef MAGNETOMETER
 # define MAGNETOMETER                   ENABLED
#endif

// expected magnetic field strength.  pre-arm checks will fail if 50% higher or lower than this value
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
 #ifndef COMPASS_MAGFIELD_EXPECTED
  # define COMPASS_MAGFIELD_EXPECTED     330        // pre arm will fail if mag field > 495 or < 165
 #endif
#else // APM1, PX4, SITL
 #ifndef COMPASS_MAGFIELD_EXPECTED
  #define COMPASS_MAGFIELD_EXPECTED      530        // pre arm will fail if mag field > 795 or < 265
 #endif
#endif

//////////////////////////////////////////////////////////////////////////////
//  OPTICAL_FLOW
#if defined( __AVR_ATmega2560__ )       // determines if optical flow code is included
 #define OPTFLOW                        ENABLED
#endif
#ifndef OPTFLOW                         // sets global enabled/disabled flag for optflow (as seen in CLI)
 # define OPTFLOW                       DISABLED
#endif
#ifndef OPTFLOW_ORIENTATION
 # define OPTFLOW_ORIENTATION    AP_OPTICALFLOW_ADNS3080_PINS_FORWARD
#endif
#ifndef OPTFLOW_RESOLUTION
 # define OPTFLOW_RESOLUTION     ADNS3080_RESOLUTION_1600
#endif
#ifndef OPTFLOW_FOV
 # define OPTFLOW_FOV                    AP_OPTICALFLOW_ADNS3080_08_FOV
#endif
// optical flow based loiter PI values
#ifndef OPTFLOW_ROLL_P
 #define OPTFLOW_ROLL_P 2.5f
#endif
#ifndef OPTFLOW_ROLL_I
 #define OPTFLOW_ROLL_I 0.5f
#endif
#ifndef OPTFLOW_ROLL_D
 #define OPTFLOW_ROLL_D 0.12f
#endif
#ifndef OPTFLOW_PITCH_P
 #define OPTFLOW_PITCH_P 2.5f
#endif
#ifndef OPTFLOW_PITCH_I
 #define OPTFLOW_PITCH_I 0.5f
#endif
#ifndef OPTFLOW_PITCH_D
 #define OPTFLOW_PITCH_D 0.12f
#endif
#ifndef OPTFLOW_IMAX
 #define OPTFLOW_IMAX 1
#endif


//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
//

#if !defined(FLIGHT_MODE_1)
 # define FLIGHT_MODE_1                  STABILIZE
#endif
#if !defined(FLIGHT_MODE_2)
 # define FLIGHT_MODE_2                  STABILIZE
#endif
#if !defined(FLIGHT_MODE_3)
 # define FLIGHT_MODE_3                  STABILIZE
#endif
#if !defined(FLIGHT_MODE_4)
 # define FLIGHT_MODE_4                  STABILIZE
#endif
#if !defined(FLIGHT_MODE_5)
 # define FLIGHT_MODE_5                  STABILIZE
#endif
#if !defined(FLIGHT_MODE_6)
 # define FLIGHT_MODE_6                  STABILIZE
#endif


//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//
// possible values for FS_THR parameter
#define FS_THR_DISABLED                    0
#define FS_THR_ENABLED_ALWAYS_RTL          1
#define FS_THR_ENABLED_CONTINUE_MISSION    2

#ifndef FS_THR_VALUE_DEFAULT
 # define FS_THR_VALUE_DEFAULT             975
#endif


#ifndef MINIMUM_THROTTLE
 # define MINIMUM_THROTTLE       130
#endif
#ifndef MAXIMUM_THROTTLE
 # define MAXIMUM_THROTTLE       1000
#endif

#ifndef LAND_SPEED
 # define LAND_SPEED    50          // the descent speed for the final stage of landing in cm/s
#endif
#ifndef LAND_START_ALT
 # define LAND_START_ALT 1000         // altitude in cm where land controller switches to slow rate of descent
#endif
#ifndef LAND_DETECTOR_TRIGGER
 # define LAND_DETECTOR_TRIGGER 50    // number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY
//
#ifndef GROUND_START_DELAY
 # define GROUND_START_DELAY             3
#endif


//////////////////////////////////////////////////////////////////////////////
// CAMERA TRIGGER AND CONTROL
//
#ifndef CAMERA
 # define CAMERA        ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// MOUNT (ANTENNA OR CAMERA)
//
#ifndef MOUNT
 # define MOUNT         ENABLED
#endif

#ifndef MOUNT2
 # define MOUNT2         DISABLED
#endif


//////////////////////////////////////////////////////////////////////////////
// Attitude Control
//

// definitions for earth frame and body frame
// used to specify frame to rate controllers
#define EARTH_FRAME     0
#define BODY_FRAME      1


// Flight mode roll, pitch, yaw, throttle and navigation definitions

// Acro Mode
#ifndef ACRO_YAW
 # define ACRO_YAW           	    YAW_ACRO
#endif

#ifndef ACRO_RP
 # define ACRO_RP            	    ROLL_PITCH_ACRO
#endif

#ifndef ACRO_THR
 # define ACRO_THR           	    THROTTLE_MANUAL
#endif

// Alt Hold Mode
#ifndef ALT_HOLD_YAW
 # define ALT_HOLD_YAW           	YAW_HOLD
#endif

#ifndef ALT_HOLD_RP
 # define ALT_HOLD_RP            	ROLL_PITCH_STABLE
#endif

#ifndef ALT_HOLD_THR
 # define ALT_HOLD_THR           	THROTTLE_HOLD
#endif

// AUTO Mode
// Note: Auto mode yaw behaviour is controlled by WP_YAW_BEHAVIOR parameter
#ifndef WP_YAW_BEHAVIOR_DEFAULT
 # define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL     
#endif

#ifndef AUTO_RP
 # define AUTO_RP                   ROLL_PITCH_AUTO
#endif

#ifndef AUTO_THR
 # define AUTO_THR                  THROTTLE_AUTO
#endif

// CIRCLE Mode
#ifndef CIRCLE_YAW
 # define CIRCLE_YAW             	YAW_CIRCLE
#endif

#ifndef CIRCLE_RP
 # define CIRCLE_RP                 ROLL_PITCH_AUTO
#endif

#ifndef CIRCLE_THR
 # define CIRCLE_THR                THROTTLE_HOLD
#endif

#ifndef CIRCLE_NAV
 # define CIRCLE_NAV           	    NAV_CIRCLE
#endif

#ifndef CIRCLE_RATE
 # define CIRCLE_RATE               5.0f        // degrees per second turn rate
#endif

// Guided Mode
// Note: Guided mode yaw behaviour is controlled by WP_YAW_BEHAVIOR parameter
#ifndef GUIDED_RP
 # define GUIDED_RP                 ROLL_PITCH_AUTO
#endif

#ifndef GUIDED_THR
 # define GUIDED_THR                THROTTLE_AUTO
#endif

#ifndef GUIDED_NAV
 # define GUIDED_NAV           	    NAV_WP
#endif

// LOITER Mode
#ifndef LOITER_YAW
 # define LOITER_YAW             	YAW_HOLD
#endif

#ifndef LOITER_RP
 # define LOITER_RP                 ROLL_PITCH_LOITER
#endif

#ifndef LOITER_THR
 # define LOITER_THR                THROTTLE_HOLD
#endif

#ifndef LOITER_NAV
 # define LOITER_NAV                NAV_LOITER
#endif

// POSITION Mode
#ifndef POSITION_YAW
 # define POSITION_YAW             	YAW_HOLD
#endif

#ifndef POSITION_RP
 # define POSITION_RP               ROLL_PITCH_LOITER
#endif

#ifndef POSITION_THR
 # define POSITION_THR              THROTTLE_MANUAL_TILT_COMPENSATED
#endif

#ifndef POSITION_NAV
 # define POSITION_NAV              NAV_LOITER
#endif


// RTL Mode
// Note: RTL Yaw behaviour is controlled by WP_YAW_BEHAVIOR parameter
#ifndef RTL_RP
 # define RTL_RP                    ROLL_PITCH_AUTO
#endif

#ifndef RTL_THR
 # define RTL_THR                   THROTTLE_AUTO
#endif

#ifndef SUPER_SIMPLE
 # define SUPER_SIMPLE           	DISABLED
#endif

#ifndef SUPER_SIMPLE_RADIUS
 # define SUPER_SIMPLE_RADIUS    	1000
#endif

// RTL Mode
#ifndef RTL_ALT_FINAL
 # define RTL_ALT_FINAL             200     // the altitude the vehicle will move to as the final stage of Returning to Launch.  Set to zero to land.
#endif

#ifndef RTL_ALT
 # define RTL_ALT 				    1500    // default alt to return to home in cm, 0 = Maintain current altitude
#endif

#ifndef RTL_ALT_MAX
 # define RTL_ALT_MAX               8000    // Max height to return to home in cm (i.e 80m)
#endif

#ifndef RTL_LOITER_TIME
 # define RTL_LOITER_TIME           5000    // Time (in milliseconds) to loiter above home before begining final descent
#endif



// Optical Flow LOITER Mode
#ifndef OF_LOITER_YAW
 # define OF_LOITER_YAW          	YAW_HOLD
#endif

#ifndef OF_LOITER_RP
 # define OF_LOITER_RP              ROLL_PITCH_STABLE_OF
#endif

#ifndef OF_LOITER_THR
 # define OF_LOITER_THR             THROTTLE_HOLD
#endif

#ifndef OF_LOITER_NAV
 # define OF_LOITER_NAV             NAV_NONE
#endif

//////////////////////////////////////////////////////////////////////////////
// Attitude Control
//

// Acro mode gains
#ifndef ACRO_P
 # define ACRO_P                 4.5f
#endif

#ifndef AXIS_LOCK_ENABLED
 # define AXIS_LOCK_ENABLED      ENABLED
#endif

// Stabilize (angle controller) gains
#ifndef STABILIZE_ROLL_P
 # define STABILIZE_ROLL_P          4.5f
#endif
#ifndef STABILIZE_ROLL_I
 # define STABILIZE_ROLL_I          0.0f
#endif
#ifndef STABILIZE_ROLL_IMAX
 # define STABILIZE_ROLL_IMAX    	8.0f            // degrees
#endif

#ifndef STABILIZE_PITCH_P
 # define STABILIZE_PITCH_P         4.5f
#endif
#ifndef STABILIZE_PITCH_I
 # define STABILIZE_PITCH_I         0.0f
#endif
#ifndef STABILIZE_PITCH_IMAX
 # define STABILIZE_PITCH_IMAX   	8.0f            // degrees
#endif

#ifndef  STABILIZE_YAW_P
 # define STABILIZE_YAW_P           4.5f            // increase for more aggressive Yaw Hold, decrease if it's bouncy
#endif
#ifndef  STABILIZE_YAW_I
 # define STABILIZE_YAW_I           0.0f
#endif
#ifndef  STABILIZE_YAW_IMAX
 # define STABILIZE_YAW_IMAX        8.0f            // degrees * 100
#endif

#ifndef YAW_LOOK_AHEAD_MIN_SPEED
 # define YAW_LOOK_AHEAD_MIN_SPEED  1000             // minimum ground speed in cm/s required before copter is aimed at ground course
#endif


//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//

#ifndef MAX_INPUT_ROLL_ANGLE
 # define MAX_INPUT_ROLL_ANGLE      4500
#endif
#ifndef MAX_INPUT_PITCH_ANGLE
 # define MAX_INPUT_PITCH_ANGLE     4500
#endif
#ifndef RATE_ROLL_P
 # define RATE_ROLL_P        		0.150f
#endif
#ifndef RATE_ROLL_I
 # define RATE_ROLL_I        		0.100f
#endif
#ifndef RATE_ROLL_D
 # define RATE_ROLL_D        		0.004f
#endif
#ifndef RATE_ROLL_IMAX
 # define RATE_ROLL_IMAX         	5.0f                    // degrees
#endif

#ifndef RATE_PITCH_P
 # define RATE_PITCH_P       		0.150f
#endif
#ifndef RATE_PITCH_I
 # define RATE_PITCH_I       		0.100f
#endif
#ifndef RATE_PITCH_D
 # define RATE_PITCH_D       		0.004f
#endif
#ifndef RATE_PITCH_IMAX
 # define RATE_PITCH_IMAX        	5.0f                    // degrees
#endif

#ifndef RATE_YAW_P
 # define RATE_YAW_P              	0.200f
#endif
#ifndef RATE_YAW_I
 # define RATE_YAW_I              	0.020f
#endif
#ifndef RATE_YAW_D
 # define RATE_YAW_D              	0.000f
#endif
#ifndef RATE_YAW_IMAX
 # define RATE_YAW_IMAX            	8.0f          // degrees
#endif


//////////////////////////////////////////////////////////////////////////////
// Rate controlled stabilized variables
//

#ifndef MAX_ROLL_OVERSHOOT
 #define MAX_ROLL_OVERSHOOT			3000
#endif

#ifndef MAX_PITCH_OVERSHOOT
 #define MAX_PITCH_OVERSHOOT		3000
#endif

#ifndef MAX_YAW_OVERSHOOT
 #define MAX_YAW_OVERSHOOT			1000
#endif

#ifndef ACRO_BALANCE_ROLL
 #define ACRO_BALANCE_ROLL			200
#endif

#ifndef ACRO_BALANCE_PITCH
 #define ACRO_BALANCE_PITCH			200
#endif

#ifndef ACRO_TRAINER_ENABLED
 #define ACRO_TRAINER_ENABLED       ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter position control gains
//
#ifndef LOITER_P
 # define LOITER_P             		1.0f
#endif
#ifndef LOITER_I
 # define LOITER_I             		0.0f
#endif
#ifndef LOITER_IMAX
 # define LOITER_IMAX          		30             // degrees
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter rate control gains
//
#ifndef LOITER_RATE_P
 # define LOITER_RATE_P          	1.0f
#endif
#ifndef LOITER_RATE_I
 # define LOITER_RATE_I          	0.5f
#endif
#ifndef LOITER_RATE_D
 # define LOITER_RATE_D          	0.0f
#endif
#ifndef LOITER_RATE_IMAX
 # define LOITER_RATE_IMAX       	4               // maximum acceleration from I term build-up in m/s/s
#endif

//////////////////////////////////////////////////////////////////////////////
// Autopilot rotate rate limits
//
#ifndef AUTO_YAW_SLEW_RATE
 # define AUTO_YAW_SLEW_RATE        60                     // degrees/sec
#endif


//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//
#ifndef THROTTLE_CRUISE
 # define THROTTLE_CRUISE       450            //
#endif

#ifndef THR_MID
 # define THR_MID        500                            // Throttle output (0 ~ 1000) when throttle stick is in mid position
#endif

#ifndef ALT_HOLD_P
 # define ALT_HOLD_P            1.0f
#endif
#ifndef ALT_HOLD_I
 # define ALT_HOLD_I            0.0f
#endif
#ifndef ALT_HOLD_IMAX
 # define ALT_HOLD_IMAX         300
#endif

// RATE control
#ifndef THROTTLE_P
 # define THROTTLE_P            6.0f
#endif
#ifndef THROTTLE_I
 # define THROTTLE_I            0.0f
#endif
#ifndef THROTTLE_D
 # define THROTTLE_D            0.0f
#endif

#ifndef THROTTLE_IMAX
 # define THROTTLE_IMAX         300
#endif

// default maximum vertical velocity the pilot may request
#ifndef PILOT_VELZ_MAX
 # define PILOT_VELZ_MAX    250     // maximum vertical velocity in cm/s
#endif
#define ACCELERATION_MAX_Z  750     // maximum veritcal acceleration in cm/s/s

// max distance in cm above or below current location that will be used for the alt target when transitioning to alt-hold mode
#ifndef ALT_HOLD_INIT_MAX_OVERSHOOT
 # define ALT_HOLD_INIT_MAX_OVERSHOOT 200
#endif
// the acceleration used to define the distance-velocity curve
#ifndef ALT_HOLD_ACCEL_MAX
 # define ALT_HOLD_ACCEL_MAX 250    // if you change this you must also update the duplicate declaration in AC_WPNav.h
#endif

// Throttle Accel control
#ifndef THROTTLE_ACCEL_P
 # define THROTTLE_ACCEL_P  0.75f
#endif
#ifndef THROTTLE_ACCEL_I
 # define THROTTLE_ACCEL_I  1.50f
#endif
#ifndef THROTTLE_ACCEL_D
 # define THROTTLE_ACCEL_D 0.0f
#endif
#ifndef THROTTLE_ACCEL_IMAX
 # define THROTTLE_ACCEL_IMAX 500
#endif


//////////////////////////////////////////////////////////////////////////////
// Dataflash logging control
//
#ifndef LOGGING_ENABLED
 # define LOGGING_ENABLED                ENABLED
#endif


#ifndef LOG_ATTITUDE_FAST
 # define LOG_ATTITUDE_FAST             DISABLED
#endif
#ifndef LOG_ATTITUDE_MED
 # define LOG_ATTITUDE_MED              ENABLED
#endif
#ifndef LOG_GPS
 # define LOG_GPS                       ENABLED
#endif
#ifndef LOG_PM
 # define LOG_PM                        ENABLED
#endif
#ifndef LOG_CTUN
 # define LOG_CTUN                      ENABLED
#endif
#ifndef LOG_NTUN
 # define LOG_NTUN                      ENABLED
#endif
#ifndef LOG_IMU
 # define LOG_IMU                       DISABLED
#endif
#ifndef LOG_CMD
 # define LOG_CMD                       ENABLED
#endif
// current
#ifndef LOG_CURRENT
 # define LOG_CURRENT                   ENABLED
#endif
// quad motor PWMs
#ifndef LOG_MOTORS
 # define LOG_MOTORS                    DISABLED
#endif
// optical flow
#ifndef LOG_OPTFLOW
 # define LOG_OPTFLOW                   DISABLED
#endif
#ifndef LOG_PID
 # define LOG_PID                       DISABLED
#endif
#ifndef LOG_COMPASS
 # define LOG_COMPASS                   DISABLED
#endif
#ifndef LOG_INAV
 # define LOG_INAV                      DISABLED
#endif
#ifndef LOG_CAMERA
 # define LOG_CAMERA                    DISABLED
#endif

// calculate the default log_bitmask
#define LOGBIT(_s)     (LOG_ ## _s ? MASK_LOG_ ## _s : 0)

#define DEFAULT_LOG_BITMASK \
    LOGBIT(ATTITUDE_FAST)   | \
    LOGBIT(ATTITUDE_MED)    | \
    LOGBIT(GPS)             | \
    LOGBIT(PM)              | \
    LOGBIT(CTUN)            | \
    LOGBIT(NTUN)            | \
    LOGBIT(IMU)             | \
    LOGBIT(CMD)             | \
    LOGBIT(CURRENT)         | \
    LOGBIT(MOTORS)          | \
    LOGBIT(OPTFLOW)         | \
    LOGBIT(PID)             | \
    LOGBIT(COMPASS)         | \
    LOGBIT(INAV)

//////////////////////////////////////////////////////////////////////////////
// Circle navigation defaults
//
#ifndef CIRCLE_RADIUS
 # define CIRCLE_RADIUS 10              // meters for circle mode
#endif

//////////////////////////////////////////////////////////////////////////////
// AP_Limits Defaults
//

// Enable/disable AP_Limits
#ifndef AC_FENCE
 #define AC_FENCE ENABLED
#endif


//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

// use this to completely disable the CLI
#ifndef CLI_ENABLED
  #  define CLI_ENABLED           ENABLED
//  #  define CLI_ENABLED           DISABLED
#endif

// experimental mpu6000 DMP code
#ifndef DMP_ENABLED
 # define DMP_ENABLED DISABLED
#endif

// experimental mpu6000 DMP code
#ifndef SECONDARY_DMP_ENABLED
 # define SECONDARY_DMP_ENABLED DISABLED
#endif

#endif // __ARDUCOPTER_CONFIG_H__
