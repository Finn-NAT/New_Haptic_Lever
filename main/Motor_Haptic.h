#ifndef MOTOR_HAPTIC_HPP
#define MOTOR_HAPTIC_HPP

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

#include <stdio.h>
#include <inttypes.h>
#include <cmath>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

/* USER DEFINE LINE ------------------------------------ */
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23

#define PIN_SPI_CS    5  

#define DRIVER_VOLTAGE_POWER_SUPPLY  20

#define FOC_VOLTAGE_LIMIT 20

#define FOC_AZI_PID_P_DEFAULT 415         
#define FOC_AZI_PID_D_DEFAULT 0.01        
#define FOC_AZI_PID_I_DEFAULT 0.7       
#define FOC_AZI_PID_PV_DEFAULT 0.54        
#define FOC_AZI_PID_IV_DEFAULT 0.03
//-------------------------------------
//PID Values for Lever-----------------
#define FOC_PID_P_DEFAULT 950
#define FOC_PID_D_DEFAULT 0.01
#define FOC_PID_I_DEFAULT 0.5
#define FOC_PID_PV_DEFAULT 0.8
#define FOC_PID_IV_DEFAULT 0.1
//-------------------------------------

#define FOC_LOW_PASS_FILTER_VELOCITY 0.09f
#define FOC_PID_VELOCITY_LIMIT 30
#define FOC_VOLTAGE_SENSOR_ALIGN 7
#define FOC_SENSOR_DIRCTION  Direction::CCW

#define FOC_ZERO_ELECTRIC_ANGLE   NOT_SET   //3.61942720  //5.75549555f //NOT_SET  

#define CALIB_PD_P_VALUE 200.0f
#define CALIB_PD_D_VALUE 0.01f
#define CALIB_PD_I_VALUE 0.1f  
#define CALIB_TORQUE_VALUE 5.6f

//Angle Value for Azipod Haptic Functions--------------
#define HAPTIC_AZI_OUT_ANGLE_DEFAULT    (2.5f * PI / 180.0)
//Angle Value for Lever Haptic Functions---------------
#define HAPTIC_OUT_ANGLE_DEFAULT        (1.8f * PI / 180.0)
//-----------------------------------------------------

#define HAPTIC_IN_ANGLE_DEFAULT  (0.25f * PI / 180.0)

/* ----------------------------------------------------- */

//PID Values for Azipod----------------
#define SUB_AZI_FOC_PID_P    200
//-------------------------------------
//PID Values for Lever-----------------
#define SUB_FOC_PID_P        400
//-------------------------------------
#define SUB_ANGLE_STEP       (0.75f * PI / 180.0f)

#define MAIN_FORCE           19.0f

#define OUT_PID_P_VALUE 450.0f
#define OUT_PID_D_VALUE 0.01f
#define OUT_PID_I_VALUE 0.001f
#define OUT_FORCE 7.0f 

/* USER DEFINE LINE END -------------------------------- */

typedef struct motor_info {
    int phA;
    int phB;
    int phC;
    int pole_pairs;
    float zero_electric_angle;
} motor_info_t;

// Loop mode enumeration
enum LoopMode {
    FUNCTION_AZI_MODE_1 = -1,   // Haptic with 4 main detents + 68 sub detents

    FUNCTION_MODE_DEFAULT = 0,  // Basic haptic mode

    FUNCTION_MODE_1 = 1,  // Haptic with 12 detents
    FUNCTION_MODE_2 = 2,  // Boundaries Haptic
    FUNCTION_MODE_3 = 3,  // Haptic with 6 detents + boundaries
    FUNCTION_MODE_4 = 4,  // Haptic with 12 detents + boundaries + vibration
    FUNCTION_MODE_5 = 5   // Function demo mode,
};

enum HapticMotorState {
    HAPTIC_MOTOR_ERROR = 0,
    HAPTIC_MOTOR_CALIB = 1,
    HAPTIC_MOTOR_READY = 2
};

enum LeverType {
    LEFT_LEVER = -1,
    AZIPOD = 0,
    RIGHT_LEVER = 1
};

class MotorHaptic {
public:
    // Constructor & Destructor
    MotorHaptic(motor_info_t motor_info, uint8_t cs_pin);
    ~MotorHaptic();

    // Initialization and main loop
    void init(LeverType lever_type);
    void calibrate(LeverType lever_type);

    void Lever_Calibration_Routine();
    void Azipod_Calibration_Routine();

    void setup();
    void loop();
    float getPosition();

    // Loop mode control
    void setLoopMode(LoopMode mode);
    LoopMode getLoopMode() const { return current_function_mode; }

    void setMotorState(HapticMotorState state) { motor_state = state; }
    HapticMotorState getMotorState() const { return motor_state; }

    // Setup functions for each mode
    void setupF0();
    void setupF1();
    void setupF2();
    void setupF3();
    void setupF4();

    void setupAF1();

    void setupDemo();

    // Different loop implementations
    void loopF0();  // Basic haptic mode
    void loopF1();  // Haptic with 12 detents
    void loopF2();  // Boundaries Haptic
    void loopF3();  // Haptic with 6 detents + boundaries
    void loopF4();  // Haptic with 12 detents + boundaries + vibration

    void loopAF1();

    void loopDemo();

    float TargetPositionDemo = 0.0f;
    bool function_demo_enabled = false;

    // Motor control methods
    float setTorquePercent(float percent);
    float setTorqueValue(float value);

private:
    // Motor components
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MagneticSensorSPI sensor;

    motor_info_t motor_info;
    uint8_t cs_pin;

    float home_angle = 0;
    float max_position = 0;
    float min_position = 0;

    HapticMotorState motor_state = HAPTIC_MOTOR_CALIB;

    LoopMode current_function_mode = FUNCTION_MODE_DEFAULT;
    void (MotorHaptic::*loop_function_ptr)() = &MotorHaptic::loopF0;
    void (MotorHaptic::*setup_function_ptr)() = &MotorHaptic::setupF0;

    float haptic_calib_p = CALIB_PD_P_VALUE;
    float haptic_calib_d = CALIB_PD_D_VALUE;
    float haptic_calib_i = CALIB_PD_I_VALUE;
    float haptic_torque = CALIB_TORQUE_VALUE;

    float haptic_out_pid_p = OUT_PID_P_VALUE;
    float haptic_out_pid_d = OUT_PID_D_VALUE;
    float haptic_out_pid_i = OUT_PID_I_VALUE;
    float haptic_out_force = OUT_FORCE;

    float haptic_default_pid_p = FOC_PID_P_DEFAULT;
    float haptic_default_pid_i = FOC_PID_I_DEFAULT;
    float haptic_default_pid_d = FOC_PID_D_DEFAULT;

    float haptic_velocity_pid_p = FOC_PID_PV_DEFAULT;
    float haptic_velocity_pid_i = FOC_PID_IV_DEFAULT;

    float haptic_sub_pid_p = SUB_FOC_PID_P;

    float main_angle_step = HAPTIC_AZI_OUT_ANGLE_DEFAULT;

    float friction_alpha = 3.0f;
    float friction_force_max = 7.0f;

};

#endif 
