#include <SimpleFOC.h>
#include <SPI.h>

static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;
static const int PIN_SPI_CS   = 5;

// Cảm biến AS5048A qua SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PIN_SPI_CS);


void setup() {
  Serial.begin(115200);
  sensor.init();
}


void loop() {
  sensor.update();
  Serial.println(sensor.getAngle(),5);
  delay(50);
}
