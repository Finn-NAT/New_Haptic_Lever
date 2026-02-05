#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE AZIPOD 
   ============================================================================ */

// Static variables for Function1
namespace FunctionAF1 {
    bool one_time = false;
    float angle_dead_zone = MAIN_ANGLE_STEP;

    float degree_90 = 90.0f * DEG_TO_RAD;
    float degree_6 = 6.0f * DEG_TO_RAD;
}

void MotorHaptic::setupAF1() {
    using namespace FunctionAF1;

    motor.P_angle.P = haptic_default_pid_p;
    motor.P_angle.D = haptic_default_pid_d;
    motor.P_angle.I = haptic_default_pid_i;
    motor.P_angle.reset();
    motor.PID_velocity.reset(); 
    motor.controller = MotionControlType::angle;
}

void MotorHaptic::loopAF1() {
    using namespace FunctionAF1;
    
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    float main_point_haptic = round(error/degree_90)*degree_90;
    float sub_point_haptic = round(error/degree_6)*degree_6;
    //printf("Error: %f deg, Main point: %f deg\n", error*RAD_TO_DEG, main_point_haptic*RAD_TO_DEG);
    if (fabs(error-main_point_haptic) < angle_dead_zone) {
        angle_dead_zone = MAIN_ANGLE_STEP;
        if(one_time) {
            // angle_dead_zone = fabs(motor.shaft_velocity)/40.0f;
            // if(angle_dead_zone < MAIN_ANGLE_STEP) {
            //     angle_dead_zone = MAIN_ANGLE_STEP;
            // }
            if(error-main_point_haptic > 0) {
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
            motor.P_angle.P = haptic_default_pid_p;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            motor.controller = MotionControlType::angle;
            one_time = false;
        }
        // if(fabs(error) < MAIN_ANGLE_STEP/4 && fabs(motor.shaft_velocity) < 0.1f) {
        //     angle_dead_zone = MAIN_ANGLE_STEP;
        // }
        motor.move(home_angle-main_point_haptic);
    }
    else if(fabs(error-sub_point_haptic) < angle_dead_zone){
        angle_dead_zone = SUB_ANGLE_STEP;
        if(one_time) {
            // angle_dead_zone = fabs(motor.shaft_velocity)/40.0f;
            // if(angle_dead_zone < MAIN_ANGLE_STEP) {
            //     angle_dead_zone = MAIN_ANGLE_STEP;
            // }
            // if(error-main_point_haptic > 0) {
            //     motor.move(MAIN_FORCE);
            //     motor.loopFOC();
            //     delayMicroseconds(1);
            //     motor.move(MAIN_FORCE);
            //     motor.loopFOC();
            // } else {
            //     motor.move(-MAIN_FORCE);
            //     motor.loopFOC();
            //     delayMicroseconds(1);
            //     motor.move(-MAIN_FORCE);
            //     motor.loopFOC();
            // }
            motor.P_angle.P = SUB_FOC_PID_P;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            motor.controller = MotionControlType::angle;
            one_time = false;
        }
        // if(fabs(error) < MAIN_ANGLE_STEP/4 && fabs(motor.shaft_velocity) < 0.1f) {
        //     angle_dead_zone = MAIN_ANGLE_STEP;
        // }
        motor.move(home_angle-sub_point_haptic);
    }
    else {
        one_time = true;
        angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
        motor.controller = MotionControlType::torque;
        motor.move(_constrain(-friction_alpha*motor.shaft_velocity, -friction_force_max, friction_force_max));
    }
}
