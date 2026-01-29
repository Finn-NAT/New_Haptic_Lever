#include <SimpleFOC.h>
#include <SPI.h>
#include <Preferences.h>  

#define VOLTAGE_POWER_SUPPLY 20
#define VOLTAGE_LIMIT 18
#define VELOCITY_LIMIT 20
#define VOLTAGE_SENSOR_ALIGN 9

#define KP_MOTOR_TEST   5
#define KI_MOTOR_TEST   1
#define KD_MOTOR_TEST   0.1

#define KPV_MOTOR_TEST  0.5
#define KIV_MOTOR_TEST  0.1

#define FORCE_MOTOR_TEST 7

Preferences preferences;  

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25);

static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

// Cảm biến AS5048A qua SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);

int print_every = 100;
int print_counter = 0;

int roll_counter = 0;
uint64_t loop_counter = 0;

float force = FORCE_MOTOR_TEST;

void setup() {
  Serial.begin(115200);

  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;     
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;                        
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
  // motor.zero_electric_angle = 5.5633659;
  motor.initFOC();
  Serial.println(motor.zero_electric_angle, 7);
  

  // Khởi tạo NVS và đọc loop_counter từ flash
  preferences.begin("motor_data", false); // "motor_data" = namespace, false = read/write mode
  loop_counter = preferences.getULong64("loop_count", 0); // Đọc giá trị, mặc định = 0 nếu chưa có
  Serial.print("Loop counter restored from flash: ");
  Serial.println((unsigned long)loop_counter);
}

void loop() {

  motor.loopFOC();

  if (++roll_counter >= 4000) {
    force = -force;
    roll_counter = 0;
    if(force > 0){
        for(int i = 0; i < 500; i++) {
          motor.loopFOC();
          motor.move(0);
          delay(1); // 1ms x 3000 = 3s
        }
        loop_counter++;
        Serial.print("Loop counter: ");
        Serial.println((unsigned long)loop_counter);
        
        // Lưu vào flash mỗi 1000 cycles
        if(loop_counter % 500 == 0) {
          preferences.putULong64("loop_count", loop_counter);
          Serial.println(">>> Saved to flash!");
          for(int i = 0; i < 150000; i++) {
            motor.loopFOC();
            motor.move(0);
            delay(1); 
          }
        }

    }else{
        for(int i = 0; i < 200; i++) {
          motor.loopFOC();
          motor.move(0);
          delay(1); 
        }
    }
  }

  motor.move(force);


  // motor.move(6);

  // // Print less frequently to further smooth the visual plot
  // if (++print_counter >= print_every) {
  //   Serial.print("motor velocity: ");
  //   Serial.println(motor.shaft_velocity);
  //   print_counter = 0;
  // }
  // _delay(1);


}
