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
void MotorHaptic::init(LeverType lever_type) {
    if(lever_type == AZIPOD) {
        haptic_default_pid_p = FOC_AZI_PID_P_DEFAULT;
        haptic_default_pid_d = FOC_AZI_PID_D_DEFAULT;
        haptic_default_pid_i = FOC_AZI_PID_I_DEFAULT;

        haptic_velocity_pid_i = FOC_AZI_PID_IV_DEFAULT;
        haptic_velocity_pid_p = FOC_AZI_PID_PV_DEFAULT;

        haptic_sub_pid_p = SUB_AZI_FOC_PID_P;

        main_angle_step = HAPTIC_AZI_OUT_ANGLE_DEFAULT;
    }else {
        haptic_default_pid_p = FOC_PID_P_DEFAULT;
        haptic_default_pid_d = FOC_PID_D_DEFAULT;
        haptic_default_pid_i = FOC_PID_I_DEFAULT;

        haptic_velocity_pid_i = FOC_PID_IV_DEFAULT;
        haptic_velocity_pid_p = FOC_PID_PV_DEFAULT;

        haptic_sub_pid_p = SUB_FOC_PID_P;

        main_angle_step = HAPTIC_OUT_ANGLE_DEFAULT;
    }

    sensor.init();
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = DRIVER_VOLTAGE_POWER_SUPPLY;
    driver.pwm_frequency = 50000;
    driver.init(); 
    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;
    // Velocity PID (vòng trong)
    motor.PID_velocity.P = haptic_velocity_pid_p;
    motor.PID_velocity.I = haptic_velocity_pid_i;
    // motor.PID_velocity.output_ramp = 1000;  // Ramp output để mượt hơn
    
    // Angle PID (vòng ngoài) 
    motor.P_angle.P = haptic_default_pid_p;
    motor.P_angle.D = haptic_default_pid_d;     
    motor.P_angle.I = haptic_default_pid_i;     
    // motor.P_angle.output_ramp = 10000;       
    
    motor.voltage_limit = FOC_VOLTAGE_LIMIT;
    motor.LPF_velocity.Tf = FOC_LOW_PASS_FILTER_VELOCITY;
    motor.LPF_angle.Tf = 0.009f;
    motor.velocity_limit = FOC_PID_VELOCITY_LIMIT;
    motor.init();
    motor.sensor_direction = FOC_SENSOR_DIRCTION;
    motor.voltage_sensor_align = FOC_VOLTAGE_SENSOR_ALIGN;
    if(motor_info.zero_electric_angle != NOT_SET) {
        motor.zero_electric_angle = motor_info.zero_electric_angle;
    }
    motor.initFOC();
    
    printf("MotorHaptic initialized successfully: (zero_electric_angle = %.8f)\n", motor.zero_electric_angle);
}

void MotorHaptic::Lever_Calibration_Routine(){
  setMotorState(HAPTIC_MOTOR_CALIB);

  float vec_tb = 0;
  uint32_t count_vec = 0;
  int count = 0;

  motor.controller = MotionControlType::torque;

  bool run_right = false;
  bool run_left = false;
  
  while(1){
    float old_angle = motor.shaft_angle;
    uint64_t timeout_counter = esp_timer_get_time();

	  motor.loopFOC();
	  while(timeout_counter + 8000000 > esp_timer_get_time()){
	    for(int i = 0 ; i < 40; i++){
		  motor.loopFOC();
		  motor.move(haptic_torque);
		  delay(1);
	    }
	    if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
		    count++;
		    if(count > 20){
		      max_position = motor.shaft_angle;
          run_right = true;
		      break;
		    }else{
          vec_tb += fabs(motor.shaft_velocity);
          count_vec++;
        }
	    }
	    old_angle = motor.shaft_angle; 
	  }
    count = 0;
    motor.move(0);

	  while(timeout_counter + 16000000 > esp_timer_get_time()){
	    for(int i = 0 ; i < 40; i++){
		  motor.loopFOC();
		  motor.move(-haptic_torque);
		  delay(1);
	    }
	    if(fabs(motor.shaft_angle - old_angle) < 0.5*DEG_TO_RAD){
		    count++;
        //printf("Count for min: %d\n", count);
		    if(count > 20){
		      min_position = motor.shaft_angle;
          run_left = true;
		      break;
		    }
	    }else{
        vec_tb += fabs(motor.shaft_velocity);
        count_vec++;
      }

	    old_angle = motor.shaft_angle; 
	  }
    count = 0;
    motor.move(0);

    home_angle = (max_position + min_position)/2;

    if(!run_right || !run_left || (max_position - min_position) == 0){
      motor.loopFOC();
      setMotorState(HAPTIC_MOTOR_ERROR); 
      return; 
    }

    vec_tb /= count_vec;
    printf("Calibration done. vec_tb: %.2f - %ld\n", vec_tb, count_vec);

    bool up_flag = false;
    bool down_flag = false;
    float delta_vec = 1.0f;
    if(vec_tb > 1.5 && vec_tb < 1.7){
      printf("Calibration successful!\n");
      break;
    }else if(vec_tb >= 1.7){
      if(up_flag && down_flag){
        delta_vec *= 2.0f;
        up_flag = false;
        down_flag = false;
      }

      haptic_torque -= (0.5f / delta_vec);
      haptic_calib_p -= (5.0f / delta_vec);

      haptic_out_force -= (0.75f / delta_vec);
      haptic_out_pid_p -= (7.5f / delta_vec);

      haptic_default_pid_p -= (50.0f / delta_vec);

      if(friction_alpha < 3.0f){
        friction_alpha += 0.5f / delta_vec;
      }

      if(friction_force_max < 7.0f){
        friction_force_max += 1.0f / delta_vec;
      }

      vec_tb = 0;
      count_vec = 0;
      down_flag = true;
      printf("Decreasing torque to %.2f for next calibration attempt\n", haptic_torque);
    }else{
      if(up_flag && down_flag){
        delta_vec *= 2.0f;
        up_flag = false;
        down_flag = false;
      }

      haptic_torque += (0.5f / delta_vec);
      haptic_calib_p += (5.0f / delta_vec);

      haptic_out_force += (0.75f / delta_vec);
      haptic_out_pid_p += (7.5f / delta_vec);

      haptic_default_pid_p += (50.0f / delta_vec);

      if(friction_alpha > 0.5f){
        friction_alpha -= 0.5f / delta_vec;
      }

      if(friction_force_max > 2.0f){
        friction_force_max -= 1.0f / delta_vec;
      }

      vec_tb = 0;
      count_vec = 0;
      up_flag = true;
      printf("Increasing torque to %.2f for next calibration attempt\n", haptic_torque);
    }

    run_right = false;
    run_left = false;

    vec_tb = 0;
    count_vec = 0;
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }

  motor.P_angle.P = haptic_calib_p;
  motor.P_angle.D = haptic_calib_d;
  motor.P_angle.I = haptic_calib_i;

  motor.P_angle.reset();
  motor.PID_velocity.reset();

	while(1){
	    motor.loopFOC();
	    float error =  motor.shaft_angle - home_angle;
	    float shaft_velocity_sp = motor.P_angle(home_angle - motor.shaft_angle );
        shaft_velocity_sp = _constrain(shaft_velocity_sp,-motor.velocity_limit, motor.velocity_limit);
        float current_sp = motor.PID_velocity(shaft_velocity_sp - motor.shaft_velocity); 
        current_sp = _constrain(current_sp,-haptic_torque,haptic_torque);
        motor.move(current_sp);
	    if(fabs(error*RAD_TO_DEG) < 0.25){count++;}
	    if(count > 50) {
            motor.P_angle.reset();
            motor.PID_velocity.reset();
            break;
        }
	    delay(1);
	}
  setMotorState(HAPTIC_MOTOR_READY);
  return;    
}

void MotorHaptic::Azipod_Calibration_Routine() {
    motor.loopFOC();
    home_angle = motor.shaft_angle;
    setMotorState(HAPTIC_MOTOR_READY);
    return;  
}

// Calibration routine
void MotorHaptic::calibrate(LeverType lever_type) {
    if(lever_type == AZIPOD) {
        Azipod_Calibration_Routine();
    } else {
        Lever_Calibration_Routine();
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
        case FUNCTION_AZI_MODE_1:
            loop_function_ptr = &MotorHaptic::loopAF1;
            setup_function_ptr = &MotorHaptic::setupAF1;
            break;
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
        case FUNCTION_MODE_5:
            loop_function_ptr = &MotorHaptic::loopDemo;
            setup_function_ptr = &MotorHaptic::setupDemo;
            break;
        default:
            loop_function_ptr = &MotorHaptic::loopF0;
            setup_function_ptr = &MotorHaptic::setupF0;
            break;
    }
    
    printf("Loop mode changed to: %d\n", mode);
}