#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE 1 - Haptic with 12 detents
   ============================================================================ */

#define F1_MAX_HAPTICS          12

// Static variables for Function1
namespace FunctionF1 {
    float list_haptics[F1_MAX_HAPTICS];
    int current_haptic_index = F1_MAX_HAPTICS+1;
    bool one_time = false;
    float angle_dead_zone = 0.0f;
}

void MotorHaptic::setupF1() {
    using namespace FunctionF1;

    angle_dead_zone = main_angle_step;

    list_haptics[0] = home_angle + PI/18;
    list_haptics[1] = home_angle - PI/18;
    list_haptics[2] = list_haptics[0] + PI/18;
    list_haptics[3] = list_haptics[1] - PI/18;
    list_haptics[4] = list_haptics[2] + PI/18;
    list_haptics[5] = list_haptics[3] - PI/18;
    list_haptics[6] = list_haptics[4] + PI/18;
    list_haptics[7] = list_haptics[5] - PI/18;
    list_haptics[8] = list_haptics[6] + PI/18;
    list_haptics[9] = list_haptics[7] - PI/18;
    list_haptics[10] = list_haptics[8] + PI/18;
    list_haptics[11] = list_haptics[9] - PI/18;

    motor.P_angle.P = haptic_default_pid_p;
    motor.P_angle.D = haptic_default_pid_d;  
    motor.P_angle.I = haptic_default_pid_i; 

    motor.controller = MotionControlType::torque;
}

void MotorHaptic::loopF1() {
    using namespace FunctionF1;
    
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;

    if (fabs(error) < angle_dead_zone) {
        if(one_time) {
            angle_dead_zone = fabs(motor.shaft_velocity)/40.0f;
            if(angle_dead_zone < main_angle_step) {
                angle_dead_zone = main_angle_step;
            }
            // if(error > 0) {
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
        if(current_haptic_index < F1_MAX_HAPTICS && fabs(list_haptics[current_haptic_index] - motor.shaft_angle) < angle_dead_zone){
            angle_dead_zone  = SUB_ANGLE_STEP;
            if(one_time){
                motor.P_angle.P = SUB_FOC_PID_P;
                motor.P_angle.reset();
                motor.PID_velocity.reset();
                motor.controller = MotionControlType::angle;
                one_time = false;
            }
            motor.move(list_haptics[current_haptic_index]);
        }else{
            one_time = true;
            angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
            motor.controller = MotionControlType::torque;
            //motor.move(0);
            motor.move(_constrain(-friction_alpha*motor.shaft_velocity, -friction_force_max, friction_force_max));
            for(int i = 0; i < F1_MAX_HAPTICS; i++){
                float sub_error = list_haptics[i] - motor.shaft_angle;
                if(fabs(sub_error) < angle_dead_zone){
                    current_haptic_index = i;
                    break;
                }
            }
        }
    }
}
