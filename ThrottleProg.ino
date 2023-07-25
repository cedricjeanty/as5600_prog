#include "AS5600.h"
#include "Wire.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AceButton.h>
using namespace ace_button;

#define BUTTON1_PIN 4
#define BUTTON2_PIN 5
#define BUTTON3_PIN 6

AceButton button1(BUTTON1_PIN);
AceButton button2(BUTTON2_PIN);
AceButton button3(BUTTON3_PIN);

// Forward reference to prevent Arduino compiler becoming confused.
void handleEvent(AceButton*, uint8_t, uint8_t);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define LINE_HEIGHT 8

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AS5600 as5600;
//const uint8_t AS5600_DIR = 4;
//const uint8_t AS5600_PWR = 5;
const uint16_t MIN_ANGLE = 18*2^12/360;
const uint32_t I2C_CLK = 400000;
const uint8_t PWM_PIN = 7; //PWM input pin
const uint8_t PWM_MODE = AS5600_PWM_115;
const uint32_t PWM_PERIOD_US = 8696; //(1/115)*1e6;
const uint16_t PWM_DEADBAND_US = 256;//PWM_PERIOD_US*128/(128+4096+128) //see AS5600 datasheet
const float PWM_DEADBAND_PERCENT = 256 / 8696;
volatile uint32_t rising_ts = 0;
volatile uint32_t falling_ts = 0;
volatile uint32_t high_time_us = 0;
volatile uint32_t low_time_us = 0;
volatile float throttle = 0/1; //start with nan throttle from 0-1

void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ;  // wait for serial port to connect.
  // }

  delay(100);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.display();

  //pinMode(AS5600_PWR, OUTPUT);
  //digitalWrite(AS5600_PWR, HIGH); //use this pin to power the AS5600 for now
  Wire.setClock(I2C_CLK);
  as5600.begin();
  //as5600.begin(AS5600_DIR);  //  set direction pin.
  //as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  pinMode(PWM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), ISR_pwm_pin, CHANGE);

  // Serial.println("Setup complete. Commands:");
  // Serial.println("R to re-program");
  // Serial.println("S to set Start Angle");
  // Serial.println("E to set End Angle");
  // Serial.println("BURN SETTINGS to permanently store settings. This can only be done once!");
  // Serial.println("BURN ANGLES to permanently store angles. This can only be done <3 times!");
  // Serial.println("!!!BURN SETTINGS must be done before BURN ANGLES!!!");
  
  delay(500);
}

void loop() {
  static uint32_t last_disp_ts = 0;
  static uint32_t next_check_ms = 250;

  button1.check();
  button2.check();
  button3.check();

  if( millis() > last_disp_ts + next_check_ms){
    last_disp_ts = millis();
    next_check_ms = 250;

    if (!as5600.isConnected()){
      Serial.println("AS6500 not connected");
      display.clearDisplay();
      display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.print("NO CONNECT");
      display.display();
      next_check_ms = 1000;
      return;
    } else {
      print_config();
    }

    display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
    if (as5600.detectMagnet() == 0){
      Serial.println("Magnet not detected");
      next_check_ms = 1000;
      return;
    }
    uint16_t pot_val = as5600.rawAngle();
    // Serial.print(pot_val);
    // Serial.print(",");
    // Serial.println(calc_throttle());

    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print(pot_val);
    display.print(", ");
    display.print(calc_throttle());
    display.display();
  }
}

// The event handler for both buttons.
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {

  if (!as5600.isConnected()){
    return;
  }

  switch (eventType) {
    case AceButton::kEventClicked:
      if (button->getPin() == BUTTON1_PIN) {
        display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
        display.setCursor(0, 0);
        display.setTextSize(2);
        display.print("ZERO: ");
        uint16_t zpos = as5600.rawAngle();
        bool ret = as5600.setZPosition(zpos);
        delay(5);
        uint16_t zpos2 = as5600.getZPosition();
        if (ret == false || zpos != zpos2){
          Serial.print("Error setting zero position: ");
          display.print("ERR");
          display.display();
          Serial.print(ret);
          Serial.print(", zpos: ");
          Serial.print(zpos);
          Serial.print(", zpos2: ");
          Serial.println(zpos2);
          delay(1000);
        } else {
          Serial.print("Set zero position: ");
          display.print(zpos);
          display.display();
          Serial.println(zpos);
          delay(1000);
        }
      } else if (button->getPin() == BUTTON2_PIN) {
        display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
        display.setCursor(0, 0);
        display.setTextSize(2);
        display.print("MAX: ");
        uint16_t mpos = as5600.rawAngle();
        bool ret = as5600.setMPosition(mpos);
        delay(5);
        uint16_t mpos2 = as5600.getMPosition();
        if (ret == false || mpos != mpos2){
          Serial.print("Error setting max position: ");
          display.print("ERR");
          display.display();
          Serial.print(ret);
          Serial.print(", mpos: ");
          Serial.print(mpos);
          Serial.print(", mpos2: ");
          Serial.println(mpos2);
          delay(1000);
        } else {
          Serial.print("Set max position: ");
          display.print(mpos);
          display.display();
          Serial.println(mpos);
          delay(1000);
        }
      } else if (button->getPin() == BUTTON3_PIN) {
        Serial.print("Programming config...");
        display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
        display.setCursor(0, 0);
        display.setTextSize(2);
        display.print("PROG CONF");
        display.display();
        delay(1000);
        bool ret = reprogram();
        if (ret == false){
          Serial.println("FAIL");
          display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
          display.setCursor(0, 0);
          display.setTextSize(2);
          display.print("ERROR");
        } else {
          Serial.println("SUCCESS");
          display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
          display.setCursor(0, 0);
          display.setTextSize(2);
          display.print("SUCCESS");
        }
        display.display();
        print_config();
        delay(1000);
      }
      break;
    case AceButton::kEventLongPressed:
      uint8_t zmco = as5600.getZMCO();
      display.fillRect(0, 0, display.width(), 2*LINE_HEIGHT, SSD1306_BLACK);
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.display();

      if (button->getPin() == BUTTON3_PIN) {
        if (zmco == 0){
          Serial.println("Burning Settings");
          display.print("BURN CONF");
          display.display();
          uint16_t zpos = as5600.getZPosition();
          uint16_t mpos = as5600.getMPosition();
          uint16_t mang = ((mpos + 4096) - zpos) % 4096;
          bool ret = as5600.setMaxAngle(mang);
          delay(10);
          as5600.burnSetting();
          delay(1000);
        } else {
          Serial.println("Cannot Burn Settings");
          display.print("BURN ERROR");
          display.display();
        }
      }else if (button->getPin() == BUTTON2_PIN){
        Serial.println("Burning angles");
        display.print("BURN ANG");
        display.display();
        as5600.burnAngle();
        delay(1000);
      }
      break;
  }
}

void ISR_pwm_pin(void){
  if (digitalRead(PWM_PIN) == HIGH){
    rising_ts = micros();
    low_time_us = rising_ts - falling_ts;
  } else {
    falling_ts = micros();
    high_time_us = falling_ts - rising_ts;
  }
}

float calc_throttle(void){

  if (micros() > falling_ts + PWM_PERIOD_US*5){
    //last update was a long time ago
    return 0/1;
  }

  float duty_cycle = (float)high_time_us / (high_time_us + low_time_us); 

  if ( duty_cycle > 1.0 - PWM_DEADBAND_PERCENT){
    //PWM value is too high
    return 0/1;
  }

  if ( duty_cycle < PWM_DEADBAND_PERCENT){
    //PWM value is too low
    return 0/1;
  }
  
  return duty_cycle;
}

bool reprogram(void){
  delay(5);
  as5600.setPowerMode(AS5600_POWERMODE_NOMINAL);
  delay(5);
  if (as5600.getPowerMode() != AS5600_POWERMODE_NOMINAL){
    return false;
  }

  as5600.setOutputMode(AS5600_OUTMODE_PWM);
  delay(5);
  if (as5600.getOutputMode() != AS5600_OUTMODE_PWM){
    return false;
  }

  as5600.setPWMFrequency(PWM_MODE);
  delay(5);
  if(as5600.getPWMFrequency() != PWM_MODE){
    return false;
  }

  as5600.setHysteresis(AS5600_HYST_LSB2);
  delay(5);
  if (as5600.getHysteresis() != AS5600_HYST_LSB2){
    return false;
  }

  as5600.setSlowFilter(AS5600_SLOW_FILT_4X);
  delay(5);
  if (as5600.getSlowFilter() != AS5600_SLOW_FILT_4X){
    return false;
  }

  as5600.setFastFilter(AS5600_FAST_FILT_LSB6);
  delay(5);
  if (as5600.getFastFilter() != AS5600_FAST_FILT_LSB6){
    return false;
  }

  as5600.setWatchDog(AS5600_WATCHDOG_OFF);
  delay(5);
  if (as5600.getWatchDog() != AS5600_WATCHDOG_OFF){
    return false;
  }

  return true;
}

void print_config(void){
  // Serial.print("STAT: ");
  // Serial.println(as5600.readStatus(), HEX);
  // Serial.print(" CFG: ");
  // Serial.println(as5600.getConfigure(), HEX);
  // Serial.print("GAIN: ");
  // Serial.println(as5600.readAGC(), HEX);
  // Serial.print(" MAG: ");
  // Serial.println(as5600.readMagnitude(), HEX);
  // Serial.print(" DET: ");
  // Serial.print(as5600.detectMagnet(), HEX);
  // Serial.print(" H:");
  // Serial.print(as5600.magnetTooStrong(), HEX);
  // Serial.print(" L:");
  // Serial.println(as5600.magnetTooWeak(), HEX);
  // Serial.print("ZMCO: ");
  // Serial.print(as5600.getZMCO());
  // Serial.println();

  display.fillRect(0, 2*LINE_HEIGHT+1, display.width(), display.height(), SSD1306_BLACK);
  display.setCursor(0, 2*LINE_HEIGHT+1);
  display.setTextSize(1);

  display.print("Z: ");
  display.print(as5600.getZPosition());
  display.print(" M: ");
  display.print(as5600.getMPosition());
  display.print(" A: ");
  display.println(as5600.getMaxAngle());
  display.print("CFG: ");
  display.println(as5600.getConfigure(), HEX);
  display.print("GAIN: ");
  display.print(as5600.readAGC(), HEX);
  display.print(" MAG: ");
  display.println(as5600.readMagnitude(), HEX);
  display.print("DET: ");
  display.print(as5600.detectMagnet(), HEX);
  display.print(" H:");
  display.print(as5600.magnetTooStrong(), HEX);
  display.print(" L:");
  display.println(as5600.magnetTooWeak(), HEX);
  display.print("ZMCO: ");
  display.print(as5600.getZMCO());
  display.println();
  display.display();
}



