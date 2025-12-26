#include <SimpleFOC.h>
#include <SPI.h>

#define VOLTAGE_POWER_SUPPLY 20
#define VOLTAGE_LIMIT 5
#define VELOCITY_LIMIT 15
#define VOLTAGE_SENSOR_ALIGN 6


BLDCMotor motor = BLDCMotor(11);         
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25);

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;     
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;                         
  motor.voltage_limit = VOLTAGE_LIMIT;        
  motor.LPF_velocity.Tf = 0.05;
  motor.velocity_limit = VELOCITY_LIMIT;
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  Serial.println("If motor is not rotating, please check the wiring!");
}


void loop() {
  motor.move(15);
}
