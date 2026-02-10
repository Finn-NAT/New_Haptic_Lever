#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE AZIPOD 
   ============================================================================ */

// Static variables for Function1
namespace FunctionAF1 {
    bool one_time = false;
    // bool one_time1 = false;
    float angle_dead_zone = 0.0f;

    float degree_90 = 90.0f * DEG_TO_RAD;
    float degree_5 = 5.0f * DEG_TO_RAD;

    // bool one_time_2 = true;
    // float pv = FOC_AZI_PID_PV_DEFAULT;

}

void MotorHaptic::setupAF1() {
    using namespace FunctionAF1;

    angle_dead_zone = main_angle_step;

    motor.P_angle.P = haptic_default_pid_p;
    motor.P_angle.D = haptic_default_pid_d;
    motor.P_angle.I = haptic_default_pid_i;
    motor.P_angle.reset();
    motor.PID_velocity.reset(); 
    // motor.controller = MotionControlType::angle;
    motor.controller = MotionControlType::torque;

    friction_alpha = 6.0f;
}

void MotorHaptic::loopAF1() {
    using namespace FunctionAF1;
    
    motor.loopFOC();
    // motor.move(5);
    float error = home_angle - motor.shaft_angle;
    float main_point_haptic = round(error/degree_90)*degree_90;
    float sub_point_haptic = round(error/degree_5)*degree_5;
    //printf("Error: %f deg, Main point: %f deg\n", error*RAD_TO_DEG, main_point_haptic*RAD_TO_DEG);
    if (fabs(error-main_point_haptic) < angle_dead_zone) {
        // angle_dead_zone = main_angle_step;
        if(one_time) {
            angle_dead_zone = fabs(motor.shaft_velocity)/25.0f;
            if(angle_dead_zone < main_angle_step) {
                angle_dead_zone = main_angle_step;
            }
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
            // motor.P_angle.D = haptic_default_pid_d;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            // motor.controller = MotionControlType::angle;
            motor.controller = MotionControlType::torque;
            one_time = false;
        }
        if(fabs(error-main_point_haptic) < main_angle_step/4 && fabs(motor.shaft_velocity) < 0.1f) {
            angle_dead_zone = main_angle_step;
        }

        // motor.move(home_angle);

        // if(fabs(error-main_point_haptic) < 1.0f*DEG_TO_RAD ) {
        //     motor.PID_velocity.P = 0.25;
        // }else if(fabs(error-main_point_haptic) < 1.5f*DEG_TO_RAD ) {
        //     motor.PID_velocity.P = 0.3;
        // }else if(fabs(error-main_point_haptic) < 2.0f*DEG_TO_RAD) {
        //     motor.PID_velocity.P = 0.35;
        // } else {
        //     motor.PID_velocity.P = pv;
        // }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////        
        float current_angle = motor.shaftAngle();      // Gọi hàm, không dùng biến member
        float current_velocity = motor.shaftVelocity(); // Gọi hàm, không dùng biến member

        // float current_sp = motor.P_angle(home_angle - main_point_haptic - current_angle);

        float shaft_velocity_sp = motor.P_angle(home_angle - main_point_haptic - current_angle);
        shaft_velocity_sp = _constrain(shaft_velocity_sp, -motor.velocity_limit, motor.velocity_limit);
        float current_sp = motor.PID_velocity(shaft_velocity_sp - current_velocity);
        current_sp = current_sp - 1.2 * current_velocity;
        current_sp = _constrain(current_sp, -11.4f, 11.4f);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////  

        motor.move(current_sp);
        // one_time1 = true;
    }
    else if(fabs(error-sub_point_haptic) < angle_dead_zone){
        angle_dead_zone = SUB_ANGLE_STEP;
        if(one_time) {
            motor.P_angle.P = SUB_FOC_PID_P;
            // motor.PID_velocity.P = 0.25;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            motor.controller = MotionControlType::angle;
            one_time = false;
        }
        motor.move(home_angle-sub_point_haptic);
    }
    else {
        // if(one_time1) {
        //     if(error > 0) {
        //         haptic_default_pid_d += 0.01f;
        //         printf("haptic_default_pid_d: %f\n", haptic_default_pid_d);
        //     } else if(haptic_default_pid_d > 0.01f && error < 0) {
        //             haptic_default_pid_d -= 0.01f;
        //             printf("haptic_default_pid_d: %f\n", haptic_default_pid_d);
        //     }
        //     one_time1 = false;
        // }
        one_time = true;
        angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
        motor.controller = MotionControlType::torque;
        motor.move(_constrain(-friction_alpha*motor.shaft_velocity, -friction_force_max, friction_force_max));
    }
}
