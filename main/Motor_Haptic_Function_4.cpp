#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE 4 - Haptic with 12 detents + boundaries + vibration
   ============================================================================ */

#define F4_MAX_HAPTICS          12

#define DELTA_MAX_MIN_ZONE 1.5f * PI / 180.0f
#define MAX_MIN_FORCE 4.5f

// Static variables for Function4
namespace FunctionF4 {
    float list_haptics[F4_MAX_HAPTICS];
    int current_haptic_index = F4_MAX_HAPTICS+1;
    bool one_time = false;
    bool one_time_main = false;
    float angle_dead_zone = MAIN_ANGLE_STEP;

    float max_position_haptic = 0;
    float min_position_haptic = 0;
    bool max_flag = true;
    bool min_flag = true;
    uint16_t max_count = 0;
    uint16_t min_count = 0;
}

void MotorHaptic::setupF4() {
    using namespace FunctionF4;

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

    max_position_haptic = list_haptics[6] + PI/18;
    min_position_haptic = list_haptics[7] - PI/18;

    motor.P_angle.P = FOC_PID_P_DEFAULT;
    motor.P_angle.D = FOC_PID_I_DEFAULT;
    motor.P_angle.I = FOC_PID_D_DEFAULT;
    // motor.P_angle.reset();
    // motor.PID_velocity.reset(); 
    motor.controller = MotionControlType::angle;
    // motor.loopFOC();
    // home_angle = motor.shaft_angle;
}

void MotorHaptic::loopF4() {
    using namespace FunctionF4;
    
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    if (fabs(error) < angle_dead_zone) {
        angle_dead_zone  = MAIN_ANGLE_STEP;
        if(one_time){
            // if(error > 0)
            // {
            //     motor.move(MAIN_FORCE);
            //     motor.loopFOC();
            //     delayMicroseconds(5);
            //     motor.move(MAIN_FORCE);
            //     motor.loopFOC();
            // }
            // else
            // {
            //     motor.move(-MAIN_FORCE);
            //     motor.loopFOC();
            //     delayMicroseconds(5);
            //     motor.move(-MAIN_FORCE);
            //     motor.loopFOC();
            // }
            motor.P_angle.P = FOC_PID_P_DEFAULT;
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            motor.controller = MotionControlType::angle;
            one_time = false;
        }
        motor.move(home_angle);
        one_time_main = true;
    }
    else if(error < (home_angle + SUB_ANGLE_STEP - max_position_haptic) && max_flag){
        if(one_time){
            motor.P_angle.P = OUT_PID_P_VALUE;
            motor.P_angle.reset();
            motor.PID_velocity.reset(); 
            one_time = false;
        }
        float shaft_velocity_sp = motor.P_angle(max_position_haptic - motor.shaft_angle );
        shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
        float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
        current_sp = _constrain(current_sp,-OUT_FORCE,OUT_FORCE);
        motor.move(current_sp); 

        if(fabs(motor.shaft_angle - max_position) < DELTA_MAX_MIN_ZONE){
            max_count++;
            if(max_count > 3000){
                motor.move(MAX_MIN_FORCE-3);
                motor.loopFOC();
                vTaskDelay(5);
                motor.move(-MAX_MIN_FORCE);
                motor.loopFOC();
                vTaskDelay(5);
            }
            if(max_count > 3200){
                max_flag = false;
                // for(int i = 0; i < 60; i++){
                //     motor.move(-MAX_MIN_FORCE);
                //     motor.loopFOC();
                //     vTaskDelay(20);
                //     motor.move(MAX_MIN_FORCE);
                //     motor.loopFOC();
                //     vTaskDelay(20);
                // }
            }
        }else{
            max_count = 0;
            
        }
    }
    else if(error > (home_angle - SUB_ANGLE_STEP - min_position_haptic) && min_flag){
        if(one_time){
            motor.P_angle.P = OUT_PID_P_VALUE;
            motor.P_angle.reset();
            motor.PID_velocity.reset(); 
            one_time = false;
        }
        float shaft_velocity_sp = motor.P_angle(min_position_haptic - motor.shaft_angle );
        shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
        float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
        current_sp = _constrain(current_sp,-OUT_FORCE,OUT_FORCE); 
        motor.move(current_sp); 

        if(fabs(motor.shaft_angle - min_position) < DELTA_MAX_MIN_ZONE){
            min_count++;
            if(min_count > 3000){
                motor.move(-MAX_MIN_FORCE+3);
                motor.loopFOC();
                vTaskDelay(5);
                motor.move(MAX_MIN_FORCE);
                motor.loopFOC();
                vTaskDelay(5);
            }
            if(min_count > 3200){
                min_flag = false;
                // for(int i = 0; i < 60; i++){
                //     motor.move(-MAX_MIN_FORCE);
                //     motor.loopFOC();
                //     vTaskDelay(20);
                //     motor.move(MAX_MIN_FORCE);
                //     motor.loopFOC();
                //     vTaskDelay(20);
                // }
            }
        }else{
            min_count = 0;
        }
    }
    else {
        if(current_haptic_index < F4_MAX_HAPTICS && fabs(list_haptics[current_haptic_index] - motor.shaft_angle) < angle_dead_zone){
            angle_dead_zone  = SUB_ANGLE_STEP;
            if(one_time){
                if(fabs(list_haptics[current_haptic_index] - motor.shaft_angle) > 0){
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(5);
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                }else{
                    motor.move(-SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(5);
                    motor.move(-SUB_FORCE);
                    motor.loopFOC();
                }
                motor.P_angle.P = SUB_FOC_PID_P;
                motor.P_angle.reset();
                motor.PID_velocity.reset();
                motor.controller = MotionControlType::angle;
                one_time = false;
            }
            motor.move(list_haptics[current_haptic_index]);
        }else{
            if(one_time_main){
                if(error > 0){
                    motor.move(MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(5);
                    motor.move(MAIN_FORCE);
                    motor.loopFOC();
                }else{
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(5);
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                }
                one_time_main = false;
            }
            one_time = true;
            angle_dead_zone = HAPTIC_IN_ANGLE_DEFAULT;
            motor.controller = MotionControlType::torque;
            //motor.move(0);
            motor.move(_constrain(-3*motor.shaft_velocity, -7.0, 7.0));
            for(int i = 0; i < F4_MAX_HAPTICS; i++){
                float sub_error = list_haptics[i] - motor.shaft_angle;
                if(fabs(sub_error) < angle_dead_zone){
                    current_haptic_index = i;
                    break;
                }
            }
        }
    }
}
