#include <Arduino.h>
#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>

#include <HT1621.h>
#include <Wire.h>

HT1621 rightLcd;
HT1621 leftLcd;

//constants
const unsigned int WHEEL_SENSOR = 2;
const unsigned int TRIP_UP = 4;
const unsigned int TRIP_DOWN = 5;
const unsigned int TRIP_RESET = 6;
const unsigned int LED = 13;
const unsigned int RESET_TIMEOUT = 4000;
const unsigned int SECOND = 1000;
const unsigned int TWO_SECONDS = 2000;
const unsigned int THREE_SECONDS = 3000;
const unsigned int RIGHT_LCD_REFRESH_PERIOD_MS = 250;
const unsigned int BUTTON_DEAD_TIME_PERIOD_MS = 30;
const unsigned int WHEEL_PULSE_DEAD_TIME_PERIOD_MS = 20;
const float DECLINATION_ANGLE_DEG = 6.0 + 8.0/60.0; // for Łódź 6'8" DEG
const double WHEEL_TRIP = 1.67572552/1000.0; // wheel circuit in km
const float TRIP_STEP = 0.01;

double trip = 0.0;
unsigned long lastWheelPulseInMillis = 0;
unsigned long lastResetBtnPressedInMillis = 0;
unsigned long lastTripBtnPressedInMillis = 0;
unsigned int batteryLevel = 0;
unsigned long lastSpeedRefreshInMillis = 0;
unsigned int speed = 0;
unsigned long now;

//methods
void handleButtons();
void wheelPulse();
void setupButtons();
void setupLCDs();
void printMessage(char*, char*);
void handleResetButton();
void handleTripButton(unsigned int, float);
void updateLeftLcd(float);
void updateRightLcd(int);
void updateSpeed(unsigned long);
void clearLcds();

//app
void setup() {
  digitalWrite(LED, LOW);
  setupLCDs();
  setupButtons();

  printMessage((char*) "HELL0", (char*) "M0T0");
  delay(2000);
}

void loop() {
  now = millis();
  handleButtons();
  updateLeftLcd(trip);
  if (now - lastSpeedRefreshInMillis > 3000) {
    speed = 0.0;
  }
  updateRightLcd(speed);
}

void setupButtons() {
  pinMode(LED, OUTPUT);
  pinMode(TRIP_UP, INPUT_PULLUP);
  pinMode(TRIP_DOWN, INPUT_PULLUP);
  pinMode(TRIP_RESET, INPUT_PULLUP);
  pinMode(WHEEL_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_SENSOR), wheelPulse, FALLING);
  printMessage((char*) "B", (char*) "0");
}

void setupLCDs() {
  rightLcd.begin(12, 11, 10); // (cs, wr, Data, backlight)
  leftLcd.begin(9, 8, 7); // (cs, wr, Data, backlight)
  clearLcds();

  printMessage((char*) "LCD", (char*) "0");
}

void clearLcds() {
  leftLcd.clear();
  leftLcd.setBatteryLevel(0);
  rightLcd.clear();
  rightLcd.setBatteryLevel(0);
}

void handleButtons() {
  unsigned long tripBtnLastPressedPeriod = now - lastTripBtnPressedInMillis;
  if (tripBtnLastPressedPeriod < BUTTON_DEAD_TIME_PERIOD_MS) {
    return;
  }
  handleTripButton(TRIP_DOWN, -1 * TRIP_STEP);
  handleTripButton(TRIP_UP, TRIP_STEP);
  handleResetButton();
}

void updateSpeed(unsigned long now) {
  speed = round((WHEEL_TRIP * 3600 * 1000)/(now - lastWheelPulseInMillis));
}

void handleTripButton(unsigned int BUTTON, float tripStep) {
  unsigned int buttonState = digitalRead(BUTTON);
  if (buttonState == LOW && trip + tripStep > 0.0) {
    trip += tripStep; 
    lastTripBtnPressedInMillis = now;
  }
}

void handleResetButton() {
  unsigned int tripResetBtnState = digitalRead(TRIP_RESET);
  if (tripResetBtnState == HIGH) { //reset btn not pressed
    lastResetBtnPressedInMillis = now; //reset clock when not pressed
    batteryLevel = 0;
  }
  unsigned long resetBtnPressingTime = now - lastResetBtnPressedInMillis;

  if (resetBtnPressingTime > SECOND)  {
    batteryLevel = 1;
  }
  if (resetBtnPressingTime > TWO_SECONDS)  {
    batteryLevel = 2;
  }
  if (resetBtnPressingTime > THREE_SECONDS)  {
    batteryLevel = 3;
  }
  if (resetBtnPressingTime >= RESET_TIMEOUT) {
    trip = 0.0;
    batteryLevel = 0;
  }
  leftLcd.setBatteryLevel(batteryLevel);
}

void wheelPulse() {
  unsigned long now = millis();
  if (now - lastWheelPulseInMillis > WHEEL_PULSE_DEAD_TIME_PERIOD_MS) {
    trip += WHEEL_TRIP;
    updateSpeed(now);
  }
  lastWheelPulseInMillis = now;
}

void printMessage(char* leftMsg, char* rightMsg) {
  leftLcd.print(leftMsg);
  rightLcd.print(rightMsg);
}

void updateLeftLcd(float trip) {
  leftLcd.print(trip, 2);
}

void updateRightLcd(int speed) {
  if (now - lastSpeedRefreshInMillis > RIGHT_LCD_REFRESH_PERIOD_MS) {
    lastSpeedRefreshInMillis = now;
    rightLcd.print(speed, (char*) "%6u", 0);
  }
}