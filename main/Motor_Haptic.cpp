#include "Motor_Haptic.h"

// Constructor
MotorHaptic::MotorHaptic(motor_info_t motor_info, uint8_t cs_pin) 
    : motor(motor_info.pole_pairs),
      driver(motor_info.phA, motor_info.phB, motor_info.phC),
      sensor(AS5048_SPI, cs_pin)
{
    this->motor_info = motor_info;
    this->cs_pin = cs_pin;
}

// Destructor
MotorHaptic::~MotorHaptic() {
}

// Initialize all components
void MotorHaptic::init() {

    sensor.init();
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = DRIVER_VOLTAGE_POWER_SUPPLY;
    driver.init(); 
    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;
    motor.PID_velocity.P = FOC_PID_PV_DEFAULT;  
    motor.PID_velocity.I = FOC_PID_IV_DEFAULT;
    motor.P_angle.P = FOC_PID_P_DEFAULT;
    motor.P_angle.D = FOC_PID_I_DEFAULT;
    motor.P_angle.I = FOC_PID_D_DEFAULT;
    motor.voltage_limit = FOC_VOLTAGE_LIMIT;
    motor.LPF_velocity.Tf = FOC_LOW_PASS_FILTER_VELOCITY;
    motor.velocity_limit = FOC_PID_VELOCITY_LIMIT;
    motor.init();
    motor.sensor_direction = FOC_SENSOR_DIRCTION;
    motor.voltage_sensor_align = FOC_VOLTAGE_SENSOR_ALIGN;
    if(motor_info.zero_electric_angle != NOT_SET) {
        motor.zero_electric_angle = motor_info.zero_electric_angle;
    }
    motor.initFOC();
    
    printf("MotorHaptic initialized successfully\n");
}

// Calibration routine
void MotorHaptic::calibrate() {
  // motor.loopFOC();
  // home_angle = motor.shaft_angle;
  // max_position = home_angle + (60.0f * DEG_TO_RAD);
  // min_position = home_angle - (60.0f * DEG_TO_RAD);

  motor.controller = MotionControlType::torque;

	motor.loopFOC();
	float old_angle = motor.shaft_angle;
	int count = 0;
	while(1){
	  for(int i = 0 ; i < 40; i++){
		motor.loopFOC();
		motor.move(CALIB_TORQUE_VALUE);
		delay(1);
	  }
	  if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
		count++;
		if(count > 20){
		  max_position = motor.shaft_angle;
		  break;
		}
  
	  }
	  old_angle = motor.shaft_angle; 
	}

	count = 0;
	while(1){
	  for(int i = 0 ; i < 40; i++){
		motor.loopFOC();
		motor.move(-CALIB_TORQUE_VALUE);
		delay(1);
	  }
	  if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
		count++;
		if(count > 20){
		  min_position = motor.shaft_angle;
		  break;
		}
	  }
	  old_angle = motor.shaft_angle; 
	}
	home_angle = (max_position + min_position)/2;

    motor.P_angle.P = CALIB_PD_P_VALUE;
    motor.P_angle.D = CALIB_PD_D_VALUE;
    motor.P_angle.I = CALIB_PD_I_VALUE;

    motor.P_angle.reset();
    motor.PID_velocity.reset();

	count = 0;
	while(1){
	    motor.loopFOC();
	    float error =  motor.shaft_angle - home_angle;
	    float shaft_velocity_sp = motor.P_angle(home_angle - motor.shaft_angle );
        shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
        float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
        current_sp = _constrain(current_sp,-CALIB_TORQUE_VALUE,CALIB_TORQUE_VALUE);
        motor.move(current_sp);
	    if(fabs(error*RAD_TO_DEG) < 0.25){count++;}
	    if(count > 50) {
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            break;
        }
	    delay(1);
	}  


}

void MotorHaptic::setup(){
    (this->*setup_function_ptr)();
}

// Main loop - should be called repeatedly
void MotorHaptic::loop() {
    // Main loop - calls the appropriate loop function based on mode
    (this->*loop_function_ptr)();
}

float MotorHaptic::getPosition() {
    return (motor.shaft_angle - home_angle);
}

// Set loop mode and switch to appropriate loop function
void MotorHaptic::setLoopMode(LoopMode mode) {
    current_function_mode = mode;
    
    switch(mode) {
        case FUNCTION_MODE_1:
            loop_function_ptr = &MotorHaptic::loopF1;
            setup_function_ptr = &MotorHaptic::setupF1;
            break;
        case FUNCTION_MODE_2:
            loop_function_ptr = &MotorHaptic::loopF2;
            setup_function_ptr = &MotorHaptic::setupF2;
            break;
        case FUNCTION_MODE_3:
            loop_function_ptr = &MotorHaptic::loopF3;
            setup_function_ptr = &MotorHaptic::setupF3;
            break;
        case FUNCTION_MODE_4:
            loop_function_ptr = &MotorHaptic::loopF4;
            setup_function_ptr = &MotorHaptic::setupF4;
            break;
        default:
            loop_function_ptr = &MotorHaptic::loopF0;
            setup_function_ptr = &MotorHaptic::setupF0;
            break;
    }
    
    printf("Loop mode changed to: %d\n", mode);
}