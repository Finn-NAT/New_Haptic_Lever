#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE 2 - Boundaries Haptic
   ============================================================================ */

// Static variables for Function2
namespace FunctionF2 {}

void MotorHaptic::setupF2() {
    using namespace FunctionF2;

    motor.P_angle.P = OUT_PID_P_VALUE;
    motor.P_angle.D = OUT_PID_D_VALUE;
    motor.P_angle.I = OUT_PID_I_VALUE;
    motor.controller = MotionControlType::torque;
}

void MotorHaptic::loopF2() {
    using namespace FunctionF2;
    
	motor.loopFOC();
	float shaft_velocity_sp = motor.P_angle(home_angle - motor.shaft_angle );
    shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
    float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
    current_sp = _constrain(current_sp,-OUT_FORCE,OUT_FORCE);
    motor.move(current_sp);
}
