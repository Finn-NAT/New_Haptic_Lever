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

#define FOC_VOLTAGE_LIMIT 19
#define FOC_PID_P_DEFAULT 1140
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
#define CALIB_TORQUE_VALUE 5.5f

#define HAPTIC_OUT_ANGLE_DEFAULT (1.8f * PI / 180.0)
#define HAPTIC_IN_ANGLE_DEFAULT  (0.25f * PI / 180.0)
/* ----------------------------------------------------- */

typedef struct motor_info {
    int phA;
    int phB;
    int phC;
    int pole_pairs;
    float zero_electric_angle;
} motor_info_t;

class MotorHaptic {
public:
    // Constructor & Destructor
    MotorHaptic(motor_info_t motor_info, int cs_pin);
    ~MotorHaptic();

    // Initialization and main loop
    void init();
    void setup();
    void loop();
    void calibrate();

    // Motor control methods
    float setTorquePercent(float percent);
    float setTorqueValue(float value);

private:

    // Motor components
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MagneticSensorSPI sensor;

    motor_info_t motor_info;
    int cs_pin;

    float home_angle = 0;
    float max_position = 0;
    float min_position = 0;
};

#endif 
