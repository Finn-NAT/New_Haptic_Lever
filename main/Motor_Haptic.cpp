#include "Motor_Haptic.h"

#define HAPTIC_FUNCTION      4

#if HAPTIC_FUNCTION == 1
// FUNCTION 1 =======================================================//
#define SUB_FOC_PID_P 200
#define SUB_ANGLE_STEP 1.0f * PI / 180.0f
#define SUB_FORCE 5.0f  
#define MAIN_ANGLE_STEP 2.0f * PI / 180.0f  
#define MAIN_FORCE 19.0f  
#define MAX_HAPTICS 12

float list_haptics[MAX_HAPTICS];
int current_haptic_index = MAX_HAPTICS+1;
bool one_time = false;
bool one_time_main = false;
float angle_dead_zone = MAIN_ANGLE_STEP;
//===================================================================//
#elif HAPTIC_FUNCTION == 2
// FUNCTION 2 =======================================================//
#define MAIN_PID_P_VALUE 600.0f
#define MAIN_PID_D_VALUE 0.001f
#define MAIN_PID_I_VALUE 0.2f
#define MAIN_FORCE 6.0f 
//===================================================================//
#elif HAPTIC_FUNCTION == 3
// FUNCTION 3 =======================================================//
#define SUB_FOC_PID_P 200
#define SUB_ANGLE_STEP 1.0f * PI / 180.0f
#define SUB_FORCE 4.0f  
#define MAIN_ANGLE_STEP 2.0f * PI / 180.0f  
#define MAIN_FORCE 19.0f  
#define MAX_HAPTICS 6

#define OUT_PID_P_VALUE 400.0f
#define OUT_PID_D_VALUE 0.01f
#define OUT_PID_I_VALUE 0.001f
#define OUT_FORCE 7.0f

float list_haptics[MAX_HAPTICS];
int current_haptic_index = MAX_HAPTICS+1;
bool one_time = false;
bool one_time_main = false;
float angle_dead_zone = MAIN_ANGLE_STEP;

float max_position_haptic = 0;
float min_position_haptic = 0;
//===================================================================//
#elif HAPTIC_FUNCTION == 4
// FUNCTION 4 =======================================================//
#define SUB_FOC_PID_P 200
#define SUB_ANGLE_STEP 1.0f * PI / 180.0f
#define SUB_FORCE 4.0f  
#define MAIN_ANGLE_STEP 2.5f * PI / 180.0f  
#define MAIN_FORCE 18.0f  
#define MAX_HAPTICS 12

#define OUT_PID_P_VALUE 400.0f
#define OUT_PID_D_VALUE 0.005f
#define OUT_PID_I_VALUE 0.001f
#define OUT_FORCE 7.0f

#define DELTA_MAX_MIN_ZONE 3.5f * PI / 180.0f
#define MAX_MIN_FORCE 3.5f

float list_haptics[MAX_HAPTICS];
int current_haptic_index = MAX_HAPTICS+1;
bool one_time = false;
bool one_time_main = false;
float angle_dead_zone = MAIN_ANGLE_STEP;

float max_position_haptic = 0;
float min_position_haptic = 0;
bool max_flag = true;
bool min_flag = true;
uint16_t max_count = 0;
uint16_t min_count = 0;
//===================================================================//
#else
#define MAIN_ANGLE_STEP 2 * PI / 180.0  
#define MAIN_FORCE 19.0f  
bool one_time = false;
bool one_time_main = false;
float angle_dead_zone = MAIN_ANGLE_STEP;
#endif

// Constructor
MotorHaptic::MotorHaptic(motor_info_t motor_info, int cs_pin) 
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

// Set torque as percentage
float MotorHaptic::setTorquePercent(float percent) {
    float torque = (percent / 100.0f) * motor.voltage_limit;
    return setTorqueValue(torque);
}

// Set torque as absolute value
float MotorHaptic::setTorqueValue(float value) {
    motor.controller = MotionControlType::torque;
    motor.move(value);
    motor.loopFOC();
    return value;
}

#if HAPTIC_FUNCTION == 1
void MotorHaptic::setup(){
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
    // motor.P_angle.reset();
    // motor.PID_velocity.reset(); 
    motor.controller = MotionControlType::angle;
    // motor.loopFOC();
    // home_angle = motor.shaft_angle;
}

// Main loop - should be called repeatedly
void MotorHaptic::loop() {
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    if (fabs(error) < angle_dead_zone) {
        angle_dead_zone  = MAIN_ANGLE_STEP;
        if(one_time){
            if(error > 0)
            {
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
            }
            else
            {
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
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
        if(current_haptic_index < MAX_HAPTICS && fabs(list_haptics[current_haptic_index] - motor.shaft_angle) < angle_dead_zone){
            angle_dead_zone  = SUB_ANGLE_STEP;
            if(one_time){
                if(fabs(list_haptics[current_haptic_index] - motor.shaft_angle) > 0){
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
                }else{
                    motor.move(-SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
                    motor.move(-SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
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
                    delayMicroseconds(300);
                    motor.move(MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                }else{
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                }
                one_time_main = false;
            }
            one_time = true;
            angle_dead_zone = 0.25 * PI / 180.0;
            motor.controller = MotionControlType::torque;
            //motor.move(0);
            motor.move(-2*motor.shaft_velocity);
            for(int i = 0; i < MAX_HAPTICS; i++){
                float sub_error = list_haptics[i] - motor.shaft_angle;
                if(fabs(sub_error) < angle_dead_zone){
                    current_haptic_index = i;
                    break;
                }
            }
        }
    }
}
#elif HAPTIC_FUNCTION == 2
void MotorHaptic::setup(){
    motor.P_angle.P = MAIN_PID_P_VALUE;
    motor.P_angle.D = MAIN_PID_D_VALUE;
    motor.P_angle.I = MAIN_PID_I_VALUE;
    // motor.P_angle.reset();
    // motor.PID_velocity.reset(); 
    motor.controller = MotionControlType::torque;
    // motor.loopFOC();
    // home_angle = motor.shaft_angle;
}

// Main loop - should be called repeatedly
void MotorHaptic::loop() {
	motor.loopFOC();
	float shaft_velocity_sp = motor.P_angle(home_angle - motor.shaft_angle );
    shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
    float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
    current_sp = _constrain(current_sp,-MAIN_FORCE,MAIN_FORCE);
    motor.move(current_sp);    
}
#elif HAPTIC_FUNCTION == 3
void MotorHaptic::setup(){
    list_haptics[0] = home_angle + PI/18;
    list_haptics[1] = home_angle - PI/18;
    list_haptics[2] = list_haptics[0] + PI/18;
    list_haptics[3] = list_haptics[1] - PI/18;
    list_haptics[4] = list_haptics[2] + PI/18;
    list_haptics[5] = list_haptics[3] - PI/18;

    max_position_haptic = list_haptics[4] + PI/18;
    min_position_haptic = list_haptics[5] - PI/18;

    motor.P_angle.P = FOC_PID_P_DEFAULT;
    motor.P_angle.D = FOC_PID_I_DEFAULT;
    motor.P_angle.I = FOC_PID_D_DEFAULT;
    // motor.P_angle.reset();
    // motor.PID_velocity.reset(); 
    motor.controller = MotionControlType::angle;
    // motor.loopFOC();
    // home_angle = motor.shaft_angle;
}

// Main loop - should be called repeatedly
void MotorHaptic::loop() {
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    if (fabs(error) < angle_dead_zone) {
        angle_dead_zone  = MAIN_ANGLE_STEP;
        if(one_time){
            if(error > 0)
            {
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
            }
            else
            {
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
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
    else if(error < (home_angle + SUB_ANGLE_STEP - max_position_haptic)){
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
    }
    else if(error > (home_angle - SUB_ANGLE_STEP - min_position_haptic)){
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
    }
    else {
        if(current_haptic_index < MAX_HAPTICS && fabs(list_haptics[current_haptic_index] - motor.shaft_angle) < angle_dead_zone){
            angle_dead_zone  = SUB_ANGLE_STEP;
            if(one_time){
                if(fabs(list_haptics[current_haptic_index] - motor.shaft_angle) > 0){
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
                }else{
                    motor.move(-SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
                    motor.move(-SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(50);
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
                    delayMicroseconds(300);
                    motor.move(MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                }else{
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                }
                one_time_main = false;
            }
            one_time = true;
            angle_dead_zone = 0.25 * PI / 180.0;
            motor.controller = MotionControlType::torque;
            //motor.move(0);
            motor.move(_constrain(-2*motor.shaft_velocity, -5, 5));
            for(int i = 0; i < MAX_HAPTICS; i++){
                float sub_error = list_haptics[i] - motor.shaft_angle;
                if(fabs(sub_error) < angle_dead_zone){
                    current_haptic_index = i;
                    break;
                }
            }
        }
    }
}
#elif HAPTIC_FUNCTION == 4
void MotorHaptic::setup(){
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

// Main loop - should be called repeatedly
void MotorHaptic::loop() {
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    if (fabs(error) < angle_dead_zone) {
        angle_dead_zone  = MAIN_ANGLE_STEP;
        if(one_time){
            if(error > 0)
            {
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(5);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
            }
            else
            {
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(5);
                motor.move(-MAIN_FORCE);
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
                motor.move(MAX_MIN_FORCE-1);
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
                motor.move(-MAX_MIN_FORCE+1);
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
        if(current_haptic_index < MAX_HAPTICS && fabs(list_haptics[current_haptic_index] - motor.shaft_angle) < angle_dead_zone){
            angle_dead_zone  = SUB_ANGLE_STEP;
            if(one_time){
                if(fabs(list_haptics[current_haptic_index] - motor.shaft_angle) > 0){
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(10);
                    motor.move(SUB_FORCE);
                    motor.loopFOC();
                }else{
                    motor.move(-SUB_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(10);
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
                    delayMicroseconds(300);
                    motor.move(MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                }else{
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                    motor.move(-MAIN_FORCE);
                    motor.loopFOC();
                    delayMicroseconds(300);
                }
                one_time_main = false;
            }
            one_time = true;
            angle_dead_zone = 0.25 * PI / 180.0;
            motor.controller = MotionControlType::torque;
            //motor.move(0);
            motor.move(-3*motor.shaft_velocity);
            for(int i = 0; i < MAX_HAPTICS; i++){
                float sub_error = list_haptics[i] - motor.shaft_angle;
                if(fabs(sub_error) < angle_dead_zone){
                    current_haptic_index = i;
                    break;
                }
            }
        }
    }
}

#else
void MotorHaptic::setup(){
    motor.P_angle.P = FOC_PID_P_DEFAULT;
    motor.P_angle.D = FOC_PID_I_DEFAULT;
    motor.P_angle.I = FOC_PID_D_DEFAULT;
    // motor.P_angle.reset();
    // motor.PID_velocity.reset(); 
    motor.controller = MotionControlType::angle;
    // motor.loopFOC();
    // home_angle = motor.shaft_angle;
}

// Main loop - should be called repeatedly
void MotorHaptic::loop() {
    motor.loopFOC();
    float error = home_angle - motor.shaft_angle;
    if (fabs(error) < angle_dead_zone) {
        angle_dead_zone  = MAIN_ANGLE_STEP;
        if(one_time){
            if(error > 0)
            {
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
            }
            else
            {
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(100);
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
        if(one_time_main){
            if(error > 0)
            {
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(300);
                motor.move(MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(300);
            }
            else
            {
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(300);
                motor.move(-MAIN_FORCE);
                motor.loopFOC();
                delayMicroseconds(300);
            }
            one_time_main = false;
        }
        one_time = true;
        angle_dead_zone = 0.25 * PI / 180.0;
        motor.controller = MotionControlType::torque;
        motor.move(0);
        //motor.move(-0.5*motor.shaft_velocity);
    }
}
#endif