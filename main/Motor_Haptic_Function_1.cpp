#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE 1 - Haptic with 12 detents
   ============================================================================ */

#define F1_SUB_FOC_PID_P        200
#define F1_SUB_ANGLE_STEP       (1.0f * PI / 180.0f)
#define F1_SUB_FORCE            5.0f  
#define F1_MAIN_ANGLE_STEP      HAPTIC_OUT_ANGLE_DEFAULT
#define F1_MAIN_FORCE           19.0f
#define F1_MAX_HAPTICS          12

// Static variables for loop1
namespace FunctionF1 {
    float list_haptics[F1_MAX_HAPTICS];
    int current_haptic_index = F1_MAX_HAPTICS+1;
    bool one_time = false;
    bool one_time_main = false;
    float angle_dead_zone = F1_MAIN_ANGLE_STEP;
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

    motor.controller = MotionControlType::angle;

}

void MotorHaptic::loopF1() {
    using namespace FunctionF1;
    
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    if (fabs(error) < angle_dead_zone) {
        angle_dead_zone  = F1_MAIN_ANGLE_STEP;
        if(one_time){
            if(error > 0)
            {
                motor.move(F1_MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(5);
                motor.move(F1_MAIN_FORCE);
                motor.loopFOC();
            }
            else
            {
                motor.move(-F1_MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(5);
                motor.move(-F1_MAIN_FORCE);
                motor.loopFOC();
            }
            motor.P_angle.P = FOC_PID_P_DEFAULT;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            motor.controller = MotionControlType::angle;
            one_time = false;
        }
        motor.move(home_angle);
        one_time_main = true;
    }
    else {
        if(current_haptic_index < F1_MAX_HAPTICS && fabs(list_haptics[current_haptic_index] - motor.shaft_angle) < angle_dead_zone){
            angle_dead_zone  = F1_SUB_ANGLE_STEP;
            if(one_time){
                if(fabs(list_haptics[current_haptic_index] - motor.shaft_angle) > 0){
                    motor.move(F1_SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(10);
                    motor.move(F1_SUB_FORCE);
                    motor.loopFOC();
                }else{
                    motor.move(-F1_SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(10);
                    motor.move(-F1_SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(10);
                }
                motor.P_angle.P = F1_SUB_FOC_PID_P;
                motor.P_angle.reset();
                motor.PID_velocity.reset();
                motor.controller = MotionControlType::angle;
                one_time = false;
            }
            motor.move(list_haptics[current_haptic_index]);
        }else{
            if(one_time_main){
                if(error > 0){
                    motor.move(F1_MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(5);
                    motor.move(F1_MAIN_FORCE);
                    motor.loopFOC();
                }else{
                    motor.move(-F1_MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(5);
                    motor.move(-F1_MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(5);
                }
                one_time_main = false;
            }
            one_time = true;
            angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
            motor.controller = MotionControlType::torque;
            //motor.move(0);
            motor.move(-3*motor.shaft_velocity);
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
