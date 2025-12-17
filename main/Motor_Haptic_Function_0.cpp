#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE 0 - Basic haptic mode
   ============================================================================ */


// Static variables for Function0
namespace FunctionF0 {
    bool one_time = false;
    bool one_time_main = false;
    float angle_dead_zone = MAIN_ANGLE_STEP;
}

void MotorHaptic::setupF0() {
    using namespace FunctionF0;

    motor.P_angle.P = FOC_PID_P_DEFAULT;
    motor.P_angle.D = FOC_PID_I_DEFAULT;
    motor.P_angle.I = FOC_PID_D_DEFAULT;
    //motor.controller = MotionControlType::angle;
    
    printf("LoopDefault initialized with basic haptic mode\n");
}

void MotorHaptic::loopF0() {
    using namespace FunctionF0;
    
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    
    if (fabs(error) < angle_dead_zone) {
        if(one_time) {
            angle_dead_zone = fabs(motor.shaft_velocity)/25.0;
            if(angle_dead_zone < MAIN_ANGLE_STEP) {
                angle_dead_zone = MAIN_ANGLE_STEP;
            }
            printf("Deadzone: %.2f deg\n", angle_dead_zone * RAD_TO_DEG);
            if(error > 0) {
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(1);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
            } else {
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(1);
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
            }
            motor.P_angle.P = FOC_PID_P_DEFAULT;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            //motor.controller = MotionControlType::angle;
            one_time = false;
        }
        if(fabs(error) < MAIN_ANGLE_STEP/4 && fabs(motor.shaft_velocity) < 0.1f) {
            angle_dead_zone = MAIN_ANGLE_STEP;
        }
	    float shaft_velocity_sp = motor.P_angle(home_angle - motor.shaft_angle );
        shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
        float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
        current_sp = _constrain(current_sp,-11.4,11.4);
        motor.move(current_sp);
        one_time_main = true;
    }
    else {
        if(one_time_main) {
            // if(error > 0) {
            //     motor.move(MAIN_FORCE);
            //     motor.loopFOC();
            //     delayMicroseconds(5);
            //     motor.move(MAIN_FORCE);
            //     motor.loopFOC();
            // } else {
            //     motor.move(-MAIN_FORCE);
            //     motor.loopFOC();
            //     delayMicroseconds(5);
            //     motor.move(-MAIN_FORCE);
            //     motor.loopFOC();
            // }
            one_time_main = false;
        }
        one_time = true;
        angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
        motor.controller = MotionControlType::torque;
        motor.move(-3*motor.shaft_velocity);
        //motor.move(0);
    }
}
