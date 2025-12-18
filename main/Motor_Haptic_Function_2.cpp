#include "Motor_Haptic.h"

/* ============================================================================
   LOOP MODE 2 - Boundaries Haptic
   ============================================================================ */

// Static variables for Function2
namespace FunctionF2 {}

void MotorHaptic::setupF2() {
    using namespace FunctionF2;

    motor.P_angle.P = haptic_out_pid_p;
    motor.P_angle.D = haptic_out_pid_d;
    motor.P_angle.I = haptic_out_pid_i;
    motor.controller = MotionControlType::torque;
}

void MotorHaptic::loopF2() {
    using namespace FunctionF2;
    
	motor.loopFOC();
	float shaft_velocity_sp = motor.P_angle(home_angle - motor.shaft_angle );
    shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
    float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
    current_sp = _constrain(current_sp,-haptic_out_force,haptic_out_force);
    motor.move(current_sp);
}
