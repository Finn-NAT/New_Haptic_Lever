#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE 0 - Basic haptic mode
   ============================================================================ */


// Static variables for Function0
namespace FunctionF0 {
    bool one_time = false;
    bool one_time_main = false;
    float angle_dead_zone = 0.0f;
}

void MotorHaptic::setupF0() {
    using namespace FunctionF0;

    angle_dead_zone = main_angle_step;

    motor.P_angle.P = haptic_default_pid_p;
    motor.P_angle.D = haptic_default_pid_d;  
    motor.P_angle.I = haptic_default_pid_i;  
    motor.controller = MotionControlType::torque;
    
    printf("LoopDefault initialized with basic haptic mode\n");
}

void MotorHaptic::loopF0() {
    using namespace FunctionF0;
    
    motor.loopFOC();

	    // float shaft_velocity_sp = motor.P_angle(home_angle - motor.shaft_angle );
        // shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
        // float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
        // current_sp = _constrain(current_sp,-11.
        
    float error = home_angle - motor.shaft_angle;
    
    if (fabs(error) < angle_dead_zone) {
        if(one_time) {
            angle_dead_zone = fabs(motor.shaft_velocity)/40.0f;
            //printf("Angle dead zone updated to: %f\n", angle_dead_zone*RAD_TO_DEG);
            if(angle_dead_zone < main_angle_step) {
                angle_dead_zone = main_angle_step;
            }
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
            motor.P_angle.P = haptic_default_pid_p;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            one_time = false;
        }
        if(fabs(error) < main_angle_step/4 && fabs(motor.shaft_velocity) < 0.1f) {
            angle_dead_zone = main_angle_step;
        }

        float current_angle = motor.shaftAngle();      // Gọi hàm, không dùng biến member
        float current_velocity = motor.shaftVelocity(); // Gọi hàm, không dùng biến member

	    float shaft_velocity_sp = motor.P_angle(home_angle - current_angle );
        shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
        float current_sp = motor.PID_velocity(shaft_velocity_sp - current_velocity); 
        current_sp = _constrain(current_sp,-11.4f,11.4f);
        motor.move(current_sp);
    }
    else {
        one_time = true;
        angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
        motor.controller = MotionControlType::torque;
        motor.move(_constrain(-friction_alpha*motor.shaft_velocity, -friction_force_max, friction_force_max));
        //motor.move(0);
    }
}
