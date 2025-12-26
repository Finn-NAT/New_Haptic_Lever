#include <SimpleFOC.h>
#include <SPI.h>

#define ANGLE_TORQUE_SWITCH   0

#define VOLTAGE_POWER_SUPPLY 20
#define VOLTAGE_LIMIT 18
#define VELOCITY_LIMIT 20
#define VOLTAGE_SENSOR_ALIGN 8

#define KP_MOTOR_TEST   5
#define KI_MOTOR_TEST   1
#define KD_MOTOR_TEST   0.1

#define KPV_MOTOR_TEST  0.5
#define KIV_MOTOR_TEST  0.1

#define FORCE_MOTOR_TEST 7


BLDCMotor motor = BLDCMotor(11);         
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25);

static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

// Cảm biến AS5048A qua SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);


void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;     
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
#if ANGLE_TORQUE_SWITCH == 1
  motor.controller = MotionControlType::angle;                         
#else
  motor.controller = MotionControlType::torque;  
#endif                       
  motor.PID_velocity.P = KPV_MOTOR_TEST;   
  motor.PID_velocity.I = KIV_MOTOR_TEST;
  motor.P_angle.P = KP_MOTOR_TEST;
  motor.P_angle.D = KD_MOTOR_TEST;
  motor.P_angle.I = KI_MOTOR_TEST;
  motor.voltage_limit = VOLTAGE_LIMIT;        
  motor.LPF_velocity.Tf = 0.05;
  motor.velocity_limit = VELOCITY_LIMIT;
  motor.init();
  motor.sensor_direction = Direction::CCW;
  motor.voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
  motor.initFOC();

  Serial.println("If motor is not rotating, please check the wiring!");
}

#if ANGLE_TORQUE_SWITCH == 1

float angle_target = 90.0f * DEG_TO_RAD;

void loop() {
  static unsigned long last_switch_time = 0;
  unsigned long now = millis();

  motor.loopFOC();
  motor.move(angle_target);

  if(now - last_switch_time > 3000){
    angle_target = -angle_target;
    last_switch_time = now;
  }

}

#else

void loop() {
  static unsigned long last_switch_time = 0;
  unsigned long now = millis();
  motor.loopFOC();
  motor.move(6);
  if(now - last_switch_time > 100){
    Serial.print("shaft velocity: ");
    Serial.println(motor.shaft_velocity);
    last_switch_time = now;
  }
}

#endif