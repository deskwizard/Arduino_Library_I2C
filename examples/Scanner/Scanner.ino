#include <I2C.h>

void setup() {
  Serial.begin(19200);
  I2c.begin();
  I2c.setSpeed(400); // Valid speeds: 50, 100, 200, 250, 400, 500, 800
  I2c.scan(); 
}

void loop() {


}
