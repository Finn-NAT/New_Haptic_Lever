#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE DEMO
   ============================================================================ */

// Static variables for Function Demo
namespace FunctionDemo {}

void MotorHaptic::setupDemo() {
    using namespace FunctionDemo;
    motor.P_angle.P = 300.0f;
    motor.P_angle.D = 10.0f;
    motor.P_angle.I = 1.0f;
    motor.controller = MotionControlType::angle;

    TargetPositionDemo = 0.0f;
}

void MotorHaptic::loopDemo() {
    using namespace FunctionDemo;
    
	motor.loopFOC();
    if(TargetPositionDemo + home_angle < min_position) {
        motor.move(min_position);
    }else if(TargetPositionDemo + home_angle > max_position) {
        motor.move(max_position);
    } else {
        //printf("Moving to Target Position Demo: %.4f\n", TargetPositionDemo);
        motor.move(TargetPositionDemo + home_angle);
    }

}
