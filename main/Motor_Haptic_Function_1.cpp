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
    bool one_time_main = false;
    float angle_dead_zone = MAIN_ANGLE_STEP;
}

void MotorHaptic::setupF1() {
    using namespace FunctionF1;

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

    motor.P_angle.P = FOC_PID_P_DEFAULT;
    motor.P_angle.D = FOC_PID_I_DEFAULT;
    motor.P_angle.I = FOC_PID_D_DEFAULT;

    //motor.controller = MotionControlType::angle;
}

void MotorHaptic::loopF1() {
    using namespace FunctionF1;
    
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;

    if (fabs(error) < angle_dead_zone) {
        if(one_time) {
            angle_dead_zone = fabs(motor.shaft_velocity)/22.5;
            if(angle_dead_zone < MAIN_ANGLE_STEP) {
                angle_dead_zone = MAIN_ANGLE_STEP;
            }
            //printf("Deadzone: %.2f deg\n", angle_dead_zone * RAD_TO_DEG);
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
        // one_time_main = true;
    }
    else {
        if(current_haptic_index < F1_MAX_HAPTICS && fabs(list_haptics[current_haptic_index] - motor.shaft_angle) < angle_dead_zone){
            angle_dead_zone  = SUB_ANGLE_STEP;
            if(one_time){
                // if(fabs(list_haptics[current_haptic_index] - motor.shaft_angle) > 0){
                //     motor.move(SUB_FORCE);
                //     motor.loopFOC();
                //     delayMicroseconds(5);
                //     motor.move(SUB_FORCE);
                //     motor.loopFOC();
                // }else{
                //     motor.move(-SUB_FORCE);
                //     motor.loopFOC();
                //     delayMicroseconds(5);
                //     motor.move(-SUB_FORCE);
                //     motor.loopFOC();
                // }
                motor.P_angle.P = SUB_FOC_PID_P;
                motor.P_angle.reset();
                motor.PID_velocity.reset();
                motor.controller = MotionControlType::angle;
                one_time = false;
            }
            motor.move(list_haptics[current_haptic_index]);
        }else{
            // if(one_time_main){
            //     if(error > 0){
            //         motor.move(MAIN_FORCE);
            //         motor.loopFOC();
            //         delayMicroseconds(5);
            //         motor.move(MAIN_FORCE);
            //         motor.loopFOC();
            //     }else{
            //         motor.move(-MAIN_FORCE);
            //         motor.loopFOC();
            //         delayMicroseconds(5);
            //         motor.move(-MAIN_FORCE);
            //         motor.loopFOC();
            //     }
            //     one_time_main = false;
            // }
            one_time = true;
            angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
            motor.controller = MotionControlType::torque;
            //motor.move(0);
            motor.move(_constrain(-3*motor.shaft_velocity, -7.0, 7.0));
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
