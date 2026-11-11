// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// ############################### VARIANT SELECTION ###############################
#if !defined(PLATFORMIO)
  #define VARIANT_USART       // Variant for Serial control via USART3 input
#endif
// ########################### END OF VARIANT SELECTION ############################


// ############################### DO-NOT-TOUCH SETTINGS ###############################
#define PWM_FREQ            16000
#define DEAD_TIME              48
#ifndef DELAY_IN_MAIN_LOOP
  #define DELAY_IN_MAIN_LOOP    5
#endif
#define TIMEOUT                20
#define A2BIT_CONV             50

#define ADC_CONV_TIME_1C5       (14)
#define ADC_CONV_TIME_7C5       (20)
#define ADC_CONV_TIME_13C5      (26)
#define ADC_CONV_TIME_28C5      (41)
#define ADC_CONV_TIME_41C5      (54)
#define ADC_CONV_TIME_55C5      (68)
#define ADC_CONV_TIME_71C5      (84)
#define ADC_CONV_TIME_239C5     (252)

#define ADC_CONV_CLOCK_CYCLES   (ADC_CONV_TIME_7C5)
#define ADC_CLOCK_DIV           (4)
#define ADC_TOTAL_CONV_TIME     (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES)
// ########################### END OF  DO-NOT-TOUCH SETTINGS ############################

// ############################### BOARD VARIANT ###############################
#define BOARD_VARIANT           0
// ######################## END OF BOARD VARIANT ###############################

// ############################### BATTERY ###############################
#define BAT_FILT_COEF           655
#define BAT_CALIB_REAL_VOLTAGE  3970
#define BAT_CALIB_ADC           1492
#define BAT_CELLS               10
#define BAT_LVL2_ENABLE         0
#define BAT_LVL1_ENABLE         0
#define BAT_DEAD_ENABLE         0
#define BAT_BLINK_INTERVAL      80
#define BAT_LVL5                (390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL4                (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL3                (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL2                (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL1                (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_DEAD                (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
// ######################## END OF BATTERY ###############################



// ############################### TEMPERATURE ###############################
#define TEMP_FILT_COEF          655
#define TEMP_CAL_LOW_ADC        1655
#define TEMP_CAL_LOW_DEG_C      358
#define TEMP_CAL_HIGH_ADC       1588
#define TEMP_CAL_HIGH_DEG_C     489
#define TEMP_WARNING_ENABLE     0
#define TEMP_WARNING            600
#define TEMP_POWEROFF_ENABLE    0
#define TEMP_POWEROFF           650

#define STARTUP_DELAY_MS        500
// ######################## END OF TEMPERATURE ###############################

// ############################### ODOMETRY ######################################
#define ENABLE_ODOMETRY
// ########################### END OF ODOMETRY ###################################



// ############################### MOTOR CONTROL #########################
#define COM_CTRL        0
#define SIN_CTRL        1
#define FOC_CTRL        2

#define OPEN_MODE       0
#define VLT_MODE        1
#define SPD_MODE        2
#define TRQ_MODE        3

#define MOTOR_LEFT_ENA
#define MOTOR_RIGHT_ENA

#define CTRL_TYP_SEL    FOC_CTRL
#define CTRL_MOD_REQ    SPD_MODE
#define DIAG_ENA        1

#define I_MOT_MAX       15
#define I_DC_MAX        17
#define N_MOT_MAX       1000

#define FIELD_WEAK_ENA  0
#define FIELD_WEAK_MAX  5
#define PHASE_ADV_MAX   25
#define FIELD_WEAK_HI   1000
#define FIELD_WEAK_LO   750
// ########################### END OF MOTOR CONTROL ########################

// Разрешить удержание позиции при нулевой команде
// #define STANDSTILL_HOLD_ENABLE


// ############################## DEFAULT SETTINGS ############################
#define INACTIVITY_TIMEOUT        8
#define BEEPS_BACKWARD            0
#define ADC_MARGIN                100
#define ADC_PROTECT_TIMEOUT       100
#define ADC_PROTECT_THRESH        200
#define AUTO_CALIBRATION_ENA

#define DEFAULT_RATE                480
#define DEFAULT_FILTER              6553
#define DEFAULT_SPEED_COEFFICIENT   16384
#define DEFAULT_STEER_COEFFICIENT   8192
// ######################### END OF DEFAULT SETTINGS ##########################



// ############################## CRUISE CONTROL SETTINGS ############################
// #define CRUISE_CONTROL_SUPPORT
// #define SUPPORT_BUTTONS_LEFT
// #define SUPPORT_BUTTONS_RIGHT
// ######################### END OF CRUISE CONTROL SETTINGS ##########################



// ############################### DEBUG SERIAL ###############################
// #define DEBUG_SERIAL_USART2
// #define DEBUG_SERIAL_USART3
// #define DEBUG_SERIAL_PROTOCOL
// ########################### END OF DEBUG SERIAL ############################



// ############################### DEBUG LCD ###############################
// #define DEBUG_I2C_LCD
// ########################### END OF DEBUG LCD ############################


// ############################### BUZZER ENABLE / DISABLE ###############################
// #define BUZZER_ENABLED
// ########################### END OF BUZZER ENABLE / DISABLE ############################



// ################################# VARIANT_ADC SETTINGS ############################
#ifdef VARIANT_ADC
  #define CONTROL_ADC           0
  #define PRI_INPUT1            3, 0, 0, 4095, 0
  #define PRI_INPUT2            3, 0, 0, 4095, 0
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY     0x1101
    #define CONTROL_SERIAL_USART3 1
    #define FEEDBACK_SERIAL_USART3
    #define AUX_INPUT1          3, -1000, 0, 1000, 0
    #define AUX_INPUT2          3, -1000, 0, 1000, 0
  #else
    #define FLASH_WRITE_KEY     0x1001
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_ADC SETTINGS #########################



// ############################ VARIANT_USART SETTINGS ############################
#ifdef VARIANT_USART
  #define CONTROL_SERIAL_USART3  0
  #define FEEDBACK_SERIAL_USART3

  #define PRI_INPUT1             3, -1000, 0, 1000, 0
  #define PRI_INPUT2             3, -1000, 0, 1000, 0

  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY      0x1102
    #define SIDEBOARD_SERIAL_USART3 1
    #define AUX_INPUT1           3, -1000, 0, 1000, 0
    #define AUX_INPUT2           3, -1000, 0, 1000, 0
  #else
    #define FLASH_WRITE_KEY      0x2001
  #endif

  // === ИСПРАВЛЕНИЕ НАПРАВЛЕНИЯ КОЛЁС ===
  // Раскомментируйте одну из строк, чтобы синхронизировать направление вращения.
  // Если оба колеса крутятся назад при положительной скорости — поменяйте местами.
  #define INVERT_R_DIRECTION
  // #define INVERT_L_DIRECTION

  // #define TANK_STEERING
  // #define SUPPORT_BUTTONS_LEFT
  // #define SUPPORT_BUTTONS_RIGHT
#endif
// ######################## END OF VARIANT_USART SETTINGS #########################



// ################################# VARIANT_NUNCHUK SETTINGS ############################
#ifdef VARIANT_NUNCHUK
  #define CONTROL_NUNCHUK         0
  #define PRI_INPUT1              2, -1024, 0, 1024, 0
  #define PRI_INPUT2              2, -1024, 0, 1024, 0
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1103
    #define CONTROL_SERIAL_USART2 1
    #define FEEDBACK_SERIAL_USART2
    #define AUX_INPUT1            3, -1000, 0, 1000, 0
    #define AUX_INPUT2            3, -1000, 0, 1000, 0
  #else
    #define FLASH_WRITE_KEY       0x1003
    #define DEBUG_SERIAL_USART2
  #endif
  #define FILTER                  3276
  #define SPEED_COEFFICIENT       8192
  #define STEER_COEFFICIENT       62259
#endif
// ############################# END OF VARIANT_NUNCHUK SETTINGS #########################



// ################################# VARIANT_PPM SETTINGS ##############################
#ifdef VARIANT_PPM
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1104
    #define CONTROL_ADC           0
    #define CONTROL_PPM_RIGHT     1
    #define PRI_INPUT1            3,     0, 0, 4095,   0
    #define PRI_INPUT2            3,     0, 0, 4095,   0
    #define AUX_INPUT1            3, -1000, 0, 1000, 100
    #define AUX_INPUT2            3, -1000, 0, 1000, 100
  #else
    #define FLASH_WRITE_KEY       0x1004
    #define CONTROL_PPM_RIGHT     0
    #define PRI_INPUT1            3, -1000, 0, 1000, 100
    #define PRI_INPUT2            3, -1000, 0, 1000, 100
  #endif
  #define PPM_NUM_CHANNELS        6
  #if defined(CONTROL_PPM_RIGHT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2
  #elif defined(CONTROL_PPM_LEFT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_PPM SETTINGS ############################


// ################################# VARIANT_PWM SETTINGS ##############################
#ifdef VARIANT_PWM
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1105
    #define CONTROL_ADC           0
    #define CONTROL_PWM_RIGHT     1
    #define PRI_INPUT1            3,     0, 0, 4095,   0
    #define PRI_INPUT2            3,     0, 0, 4095,   0
    #define AUX_INPUT1            3, -1000, 0, 1000, 100
    #define AUX_INPUT2            3, -1000, 0, 1000, 100
  #else
    #define FLASH_WRITE_KEY       0x1005
    #define CONTROL_PWM_RIGHT     0
    #define PRI_INPUT1            3, -1000, 0, 1000, 100
    #define PRI_INPUT2            3, -1000, 0, 1000, 100
  #endif
  #define FILTER                  6553
  #define SPEED_COEFFICIENT       16384
  #define STEER_COEFFICIENT       16384
  #if defined(CONTROL_PWM_RIGHT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2
  #elif defined(CONTROL_PWM_LEFT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_PWM SETTINGS ############################



// ################################# VARIANT_IBUS SETTINGS ##############################
#ifdef VARIANT_IBUS
  #define CONTROL_IBUS
  #define IBUS_NUM_CHANNELS       14
  #define IBUS_LENGTH             0x20
  #define IBUS_COMMAND            0x40
  #define USART3_BAUD             38400
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1106
    #define CONTROL_ADC           0
    #define CONTROL_SERIAL_USART3 1
    #define FEEDBACK_SERIAL_USART3
    #define PRI_INPUT1            3,     0, 0, 4095, 0
    #define PRI_INPUT2            3,     0, 0, 4095, 0
    #define AUX_INPUT1            3, -1000, 0, 1000, 0
    #define AUX_INPUT2            3, -1000, 0, 1000, 0
  #else
    #define FLASH_WRITE_KEY       0x1006
    #define CONTROL_SERIAL_USART3 0
    #define FEEDBACK_SERIAL_USART3
    #define PRI_INPUT1            3, -1000, 0, 1000, 0
    #define PRI_INPUT2            3, -1000, 0, 1000, 0
  #endif
  #if defined(CONTROL_SERIAL_USART3) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2
  #elif defined(DEBUG_SERIAL_USART2) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_IBUS SETTINGS ############################



// ############################ VARIANT_HOVERCAR SETTINGS ############################
#ifdef VARIANT_HOVERCAR
  #define FLASH_WRITE_KEY         0x1107
  #undef  CTRL_MOD_REQ
  #define CTRL_MOD_REQ            VLT_MODE
  #define CONTROL_ADC             0
  #define SIDEBOARD_SERIAL_USART3 1
  #define FEEDBACK_SERIAL_USART3
  #define DUAL_INPUTS
  #define PRI_INPUT1              1,  1000, 0, 2500, 0
  #define PRI_INPUT2              1,   500, 0, 2200, 0
  #define AUX_INPUT1              2, -1000, 0, 1000, 0
  #define AUX_INPUT2              2, -1000, 0, 1000, 0
  #define SPEED_COEFFICIENT       16384
  #define STEER_COEFFICIENT       8192
  #define MULTI_MODE_DRIVE
  #ifdef MULTI_MODE_DRIVE
      #define MULTI_MODE_DRIVE_M1_MAX   175
      #define MULTI_MODE_DRIVE_M1_RATE  250
      #define MULTI_MODE_M1_I_MOT_MAX   4
      #define MULTI_MODE_M1_N_MOT_MAX   30
      #define MULTI_MODE_DRIVE_M2_MAX   500
      #define MULTI_MODE_DRIVE_M2_RATE  300
      #define MULTI_MODE_M2_I_MOT_MAX   8
      #define MULTI_MODE_M2_N_MOT_MAX   80
      #define MULTI_MODE_DRIVE_M3_MAX   1000
      #define MULTI_MODE_DRIVE_M3_RATE  450
      #define MULTI_MODE_M3_I_MOT_MAX   I_MOT_MAX
      #define MULTI_MODE_M3_N_MOT_MAX   N_MOT_MAX
  #endif
#endif
#define MULTIPLE_TAP_NR           2 * 2
#define MULTIPLE_TAP_HI           600
#define MULTIPLE_TAP_LO           200
#define MULTIPLE_TAP_TIMEOUT      2000
// ######################## END OF VARIANT_HOVERCAR SETTINGS #########################



// ############################ VARIANT_HOVERBOARD SETTINGS ############################
#ifdef VARIANT_HOVERBOARD
  #define FLASH_WRITE_KEY     0x1008
  #define SIDEBOARD_SERIAL_USART2 1
  #define FEEDBACK_SERIAL_USART2
  #define SIDEBOARD_SERIAL_USART3 0
  #define FEEDBACK_SERIAL_USART3
  #define PRI_INPUT1          3, -1000, 0, 1000, 0
  #define PRI_INPUT2          3, -1000, 0, 1000, 0
  #define AUX_INPUT1          3, -1000, 0, 1000, 0
  #define AUX_INPUT2          3, -1000, 0, 1000, 0
#endif
// ######################## END OF VARIANT_HOVERBOARD SETTINGS #########################



// ################################# VARIANT_TRANSPOTTER SETTINGS ############################
#ifdef VARIANT_TRANSPOTTER
  #define FLASH_WRITE_KEY     0x1009
  #define CONTROL_GAMETRAK
  #define SUPPORT_LCD
  #define GAMETRAK_CONNECTION_NORMAL
  #define ROT_P               1.2
  #define SPEED_COEFFICIENT   14746
  #define STEER_COEFFICIENT   8192
  #define INVERT_R_DIRECTION
  #define INVERT_L_DIRECTION
  #define PRI_INPUT1          2, -1000, 0, 1000, 0
  #define PRI_INPUT2          2, -1000, 0, 1000, 0
#endif
// ############################# END OF VARIANT_TRANSPOTTER SETTINGS ########################


// ################################# VARIANT_SKATEBOARD SETTINGS ##############################
#ifdef VARIANT_SKATEBOARD
  #define FLASH_WRITE_KEY     0x1010
  #undef  CTRL_MOD_REQ
  #define CTRL_MOD_REQ        TRQ_MODE
  #define CONTROL_PWM_RIGHT   0
  #define PRI_INPUT1          0, -1000, 0, 1000,   0
  #define PRI_INPUT2          2,  -800, 0,  700, 100
  #define INPUT_BRK           -400
  #define FILTER              6553
  #define SPEED_COEFFICIENT   16384
  #define STEER_COEFFICIENT   0
  #define INVERT_R_DIRECTION
  #define INVERT_L_DIRECTION
  #ifdef CONTROL_PWM_RIGHT
    #define DEBUG_SERIAL_USART2
  #else
    #define DEBUG_SERIAL_USART3
  #endif
#endif
// ############################# END OF VARIANT_SKATEBOARD SETTINGS ############################



// ########################### UART SETIINGS ############################
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #define SERIAL_START_FRAME      0xABCD
  #define SERIAL_BUFFER_SIZE      64
  #define SERIAL_TIMEOUT          160
#endif
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
  #ifndef USART2_BAUD
    #define USART2_BAUD           115200
  #endif
  #define USART2_WORDLENGTH       UART_WORDLENGTH_8B
#endif
#if defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #ifndef USART3_BAUD
    #define USART3_BAUD           115200
  #endif
  #define USART3_WORDLENGTH       UART_WORDLENGTH_8B
#endif
// ########################### UART SETIINGS ############################



// ############################### APPLY DEFAULT SETTINGS ###############################
#ifndef RATE
  #define RATE DEFAULT_RATE
#endif
#ifndef FILTER
  #define FILTER DEFAULT_FILTER
#endif
#ifndef SPEED_COEFFICIENT
  #define SPEED_COEFFICIENT DEFAULT_SPEED_COEFFICIENT
#endif
#ifndef STEER_COEFFICIENT
  #define STEER_COEFFICIENT DEFAULT_STEER_COEFFICIENT
#endif
#if defined(PRI_INPUT1) && defined(PRI_INPUT2) && defined(AUX_INPUT1) && defined(AUX_INPUT2)
  #define INPUTS_NR               2
#else
  #define INPUTS_NR               1
#endif
// ########################### END OF APPLY DEFAULT SETTING ############################



// ############################### VALIDATE SETTINGS ###############################
#if !defined(VARIANT_ADC) && !defined(VARIANT_USART) && !defined(VARIANT_NUNCHUK) && !defined(VARIANT_PPM) && !defined(VARIANT_PWM) && \
    !defined(VARIANT_IBUS) && !defined(VARIANT_HOVERCAR) && !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER) && !defined(VARIANT_SKATEBOARD)
  #error Variant not defined! Please check platformio.ini or Inc/config.h for available variants.
#endif

#if defined(CONTROL_SERIAL_USART2) && defined(SIDEBOARD_SERIAL_USART2)
  #error CONTROL_SERIAL_USART2 and SIDEBOARD_SERIAL_USART2 not allowed, choose one.
#endif
#if defined(CONTROL_SERIAL_USART3) && defined(SIDEBOARD_SERIAL_USART3)
  #error CONTROL_SERIAL_USART3 and SIDEBOARD_SERIAL_USART3 not allowed, choose one.
#endif
#if defined(DEBUG_SERIAL_USART2) && defined(FEEDBACK_SERIAL_USART2)
  #error DEBUG_SERIAL_USART2 and FEEDBACK_SERIAL_USART2 not allowed, choose one.
#endif
#if defined(DEBUG_SERIAL_USART3) && defined(FEEDBACK_SERIAL_USART3)
  #error DEBUG_SERIAL_USART3 and FEEDBACK_SERIAL_USART3 not allowed, choose one.
#endif
#if defined(DEBUG_SERIAL_USART2) && defined(DEBUG_SERIAL_USART3)
  #error DEBUG_SERIAL_USART2 and DEBUG_SERIAL_USART3 not allowed, choose one.
#endif
#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PPM_RIGHT)
  #error CONTROL_PPM_LEFT and CONTROL_PPM_RIGHT not allowed, choose one.
#endif
#if defined(CONTROL_PWM_LEFT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PWM_LEFT and CONTROL_PWM_RIGHT not allowed, choose one.
#endif
#if defined(SUPPORT_BUTTONS_LEFT) && defined(SUPPORT_BUTTONS_RIGHT)
  #error SUPPORT_BUTTONS_LEFT and SUPPORT_BUTTONS_RIGHT not allowed, choose one.
#endif

#if defined(CONTROL_ADC) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_ADC and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PPM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PWM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PWM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error SUPPORT_BUTTONS_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_ADC) || defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error SUPPORT_BUTTONS_LEFT and (CONTROL_ADC or CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_ADC) && (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error CONTROL_ADC and (CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PWM_LEFT)
  #error CONTROL_PPM_LEFT and CONTROL_PWM_LEFT not allowed. It is on the same cable.
#endif

#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_NUNCHUK and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PPM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PWM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PWM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error DEBUG_I2C_LCD and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error SUPPORT_BUTTONS_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif
#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_NUNCHUK) || defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error SUPPORT_BUTTONS_RIGHT and (CONTROL_NUNCHUK or CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error CONTROL_NUNCHUK and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif
#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT))
  #error DEBUG_I2C_LCD and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT) not allowed. It is on the same cable.
#endif
#if defined(CONTROL_PPM_RIGHT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PPM_RIGHT and CONTROL_PWM_RIGHT not allowed. It is on the same cable.
#endif

#if (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)) && !defined(PPM_NUM_CHANNELS)
  #error Total number of PPM channels needs to be set
#endif
// ############################# END OF VALIDATE SETTINGS ############################

#endif