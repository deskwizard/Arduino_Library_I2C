/* Demonstration sketch for I2C library

   Basic usage of PCF8574 port expander

   A0, A1 and A3 pins grounded (i2c Address will be 0x20)
   SDA pin to Arduino A4 (Uno and compatibles)
   SCL pin to Arduino A5 (Uno and compatibles)
   LED connected to P0 (Ground side to PCF8574)
   Button connected to P4 (One side to PCF8574 with pulldown resistor (4.7k, between PCF pin and ground), one side to VCC)
   (Optional) Put a second LED connected to P0, it will flash alternatively with the debug LED

   Copyright (c) 2015-2016 Charles Dorval (a.k.a Deskwizard).  All right reserved.
   
   Download at: https://github.com/deskwizard/Arduino_Library_I2C
*/

#include <I2C.h>          // Include i2c library

#define PCF8574 0x20      // I2C Address of the port expander

#define DEBUG_LED 13      // Debug LED on pin 13 (Toggles every FLASH_DELAY milliseconds)
#define FLASH_DELAY 250   // Set LED toggle time to 250 milliseconds

#define BUTTON_DEBOUNCE_DELAY 50 // Debounce delay for buttons

#define PCF_LED_PIN 3
#define PCF_BUTTON_PIN 4

void setup() {

  pinMode(DEBUG_LED, OUTPUT);     // Set debug LED pin as output
  digitalWrite(DEBUG_LED, LOW);   // Make sure we start with the debug LED off (if the LED 5V pin is on the arduino, else change to HIGH)

  Serial.begin(19200);            // Start serial output @ 19200bps

  Serial.println(F("i2c Library PCF8574 demo sketch starting.... "));
  Serial.println();
  Serial.println(F("Button presses and releases will be printed here and"));
  Serial.println(F("the LED on P3 will lit up when the button is pressed"));
  Serial.println();

  I2c.begin();                    // Start i2c bus
  //I2c.pullup(DISABLED);         // Disable internal pull-up resistors (Uncomment if using external pullup resistors)
  I2c.setSpeed(800);              // Set bus speed to 400 khz (Valid speeds: 50/100/200/250/400/500/800 kHz)

  if (I2c.detected(0x20)) { // If chip is detected at PCF8574's address.... 
    Serial.println(F("PCF8574 Port expanded detected"));
  }
  else {
    Serial.println(F("PCF8574 Port expanded - NOT - detected!"));
  }
  Serial.println();

  // Set LED on pin 3 to button pin state
  expanderWrite(PCF_LED_PIN, expanderRead(PCF_BUTTON_PIN));
}


void loop() {
  uint32_t currentMillis = millis();                                // Variable holding the current time

  // Button debouncing / handling
  static uint32_t previousReadMillis;                               // Variable holding the last time the LED was toggled
  static bool buttonState;                                          // State of the button (1 = pressed, 0 = released)
  static uint8_t press_counter;                                     // Stores the number of times the button was pressed/released

  if ((uint32_t)(currentMillis - previousReadMillis) >= BUTTON_DEBOUNCE_DELAY) {
    bool buttonRead;
    static bool lastButtonRead;

    buttonRead = expanderRead(PCF_BUTTON_PIN);

    if (buttonRead == lastButtonRead && buttonRead != buttonState) { // Button has changed...
      expanderWrite(PCF_LED_PIN, buttonRead);

      buttonState = buttonRead;

      if (buttonState == HIGH) {
        press_counter++;
        Serial.print(F("Button was pressed "));
        Serial.print(press_counter);
        Serial.println(F(" time."));
      }
      else {
        Serial.print(F("Button was released "));
        Serial.print(press_counter);
        Serial.println(F(" time."));
      }
      Serial.println();
    }

    lastButtonRead = buttonRead;
    previousReadMillis = currentMillis;
  }


  // Debug LEDs flashing; Toggles the debug LEDs each (FLASH_DELAY) milliseconds
  static uint32_t previousFlashMillis;                              // Variable holding the last time the LED was toggled

  // If the difference between the last time we toggled the led and now is bigger or equal to FLASH_DELAY ...
  if ((uint32_t)(currentMillis - previousFlashMillis) >= FLASH_DELAY) {
    static bool debugLedState = 0;                                  // Variable holding the state of the LED
    debugLedState = !debugLedState;                                 // Toggle variable to opposite state
    digitalWrite(DEBUG_LED, debugLedState);                         // Write the variable to the debug LED pin
    expanderWrite(0, !debugLedState);                               // Write the opposite of the variable to the optional 2nd debug LED on expander
    previousFlashMillis = currentMillis;                            // Save current time as the last time we toggle the LED
  }

} // End of loop()


//***** PCF8574 Functions *****

//** Reads a PCF8574 port pin (Just like digitalRead) **
bool expanderRead(uint8_t _pin) {
  bool _pinstate;
  uint8_t _reading;

  I2c.read(PCF8574, 1);                 // Request one byte from the PCF8574
  _reading = I2c.receive();             // Receive the requested byte
  _pinstate = bitRead(_reading, _pin);  // Get the state of the corresponding bit

  return _pinstate;                     // return the value
}

//** Writes to a PCF8574 port pin (Just like digitalWrite) **
void expanderWrite(uint8_t _pin, bool _state) {
  uint16_t _reading;

  I2c.read(PCF8574, 1);           // Request one byte from the PCF8574
  _reading = I2c.receive();       // Receive the requested byte

  if (!_state) {                  // Remove the ! to change signal polarity)
    bitSet(_reading, _pin);       // Set (Write 1) the port pin bit in variable _read
  }
  else { // LOW is requested ...
    bitClear(_reading, _pin);     // Clear (Write 0) the port pin bit in variable _read
  }

  I2c.write(PCF8574, _reading);   // Update PCF8574 with new values
}

