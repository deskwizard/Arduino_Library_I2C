/* Demonstration sketch for I2C library

   Basic sketch demonstrating the scan() and setSpeed() functions or the I2C library Rev. 6.0

   Copyright (c) 2015-2016 Charles Dorval (a.k.a Deskwizard).  All right reserved.
   
   Download at: https://github.com/deskwizard/Arduino_Library_I2C
*/

#include <I2C.h>        // Include i2c library

void setup() {
  Serial.begin(19200);   // Start serial output @ 19200bps
  I2c.begin();           // Start i2c bus
  I2c.setSpeed(400);     // Set bus speed to 400 khz (Valid speeds: 50/100/200/250/400/500/800 kHz)
  I2c.scan();            // Do i2c bus scan
}

void loop() {
  // Nothing to do here
}
