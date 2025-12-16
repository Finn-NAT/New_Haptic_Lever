#ifndef MOTOR_HAPTIC_HPP
#define MOTOR_HAPTIC_HPP

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

#include <stdio.h>
#include <inttypes.h>
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
#define FOC_PID_P_DEFAULT 1270
#define FOC_PID_I_DEFAULT 0.001
#define FOC_PID_D_DEFAULT 0.001
#define FOC_PID_PV_DEFAULT 0.8
#define FOC_PID_IV_DEFAULT 0.1
#define FOC_LOW_PASS_FILTER_VELOCITY 0.05
#define FOC_PID_VELOCITY_LIMIT 30
#define FOC_VOLTAGE_SENSOR_ALIGN 9
#define FOC_SENSOR_DIRCTION  Direction::CCW

#define FOC_ZERO_ELECTRIC_ANGLE  0.1886797f  

#define CALIB_PD_P_VALUE 350.0f
#define CALIB_PD_D_VALUE 0.01f
#define CALIB_PD_I_VALUE 0.1f
#define CALIB_TORQUE_VALUE 6.0f

#define HAPTIC_OUT_ANGLE_DEFAULT (2.0f * PI / 180.0)
#define HAPTIC_IN_ANGLE_DEFAULT  (0.25f * PI / 180.0)

/* ----------------------------------------------------- */

#define SUB_FOC_PID_P        400
#define SUB_ANGLE_STEP       (1.0f * PI / 180.0f)
#define SUB_FORCE            5.0f  

#define MAIN_ANGLE_STEP      HAPTIC_OUT_ANGLE_DEFAULT
#define MAIN_FORCE           19.0f

#define OUT_PID_P_VALUE 400.0f
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
    FUNCTION_MODE_1 = 1,  // Haptic with 12 detents
    FUNCTION_MODE_2 = 2,  // Boundaries Haptic
    FUNCTION_MODE_3 = 3,  // Haptic with 6 detents + boundaries
    FUNCTION_MODE_4 = 4,  // Haptic with 12 detents + boundaries + vibration
    FUNCTION_MODE_DEFAULT = 0  // Basic haptic mode
};

class MotorHaptic {
public:
    // Constructor & Destructor
    MotorHaptic(motor_info_t motor_info, uint8_t cs_pin);
    ~MotorHaptic();

    // Initialization and main loop
    void init();
    void calibrate();
    void setup();
    void loop();
    float getPosition();

    // Loop mode control
    void setLoopMode(LoopMode mode);
    LoopMode getLoopMode() const { return current_function_mode; }

    // Setup functions for each mode
    void setupF0();
    void setupF1();
    void setupF2();
    void setupF3();
    void setupF4();

    // Different loop implementations
    void loopF0();  // Basic haptic mode
    void loopF1();  // Haptic with 12 detents
    void loopF2();  // Boundaries Haptic
    void loopF3();  // Haptic with 6 detents + boundaries
    void loopF4();  // Haptic with 12 detents + boundaries + vibration

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

    LoopMode current_function_mode = FUNCTION_MODE_DEFAULT;
    void (MotorHaptic::*loop_function_ptr)() = &MotorHaptic::loopF0;
    void (MotorHaptic::*setup_function_ptr)() = &MotorHaptic::setupF0;

};

#endif 
