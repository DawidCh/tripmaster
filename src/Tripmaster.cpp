#include <Arduino.h>
#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>

#include <HT1621.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>

HMC5883L compass;
MPU6050 mpu;
HT1621 compassLCD;
HT1621 tripLCD;

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
const unsigned int COMPASS_LCD_REFRESH_PERIOD_MS = 250;
const unsigned int BUTTON_DEAD_TIME_PERIOD_MS = 30;
const unsigned int WHEEL_PULSE_DEAD_TIME_PERIOD_MS = 20;
const float DECLINATION_ANGLE_DEG = 6.0 + 8.0/60.0; // for Łódź 6'8" DEG
const double WHEEL_TRIP = 1.67572552/1000.0; // wheel circuit in km
const float TRIP_STEP = 0.01;

double trip = 0.0;
unsigned long revs = 0;
unsigned long lastWheelPulseInMillis = 0;
unsigned long lastResetBtnPressedInMillis = 0;
unsigned long lastTripBtnPressedInMillis = 0;
unsigned int batteryLevel = 0;
unsigned long lastCompassRefreshInMillis = 0;
float speed = 0.0;

//methods
void updateCompass();
void handleButtons();
void wheelPulse();
float radToDeg(float);
float applyDeclinationAngle(float);
int modulo(int, int);
void setupButtons();
void setupCompass();
void setupSerial();
void setupLCDs();
void printMessage(char*, char*);
void setupMPU();
float tiltCompensate(Vector, Vector);
void handleResetButton();
void handleTripButton(unsigned int, float);
void updateTrip();
void updateSpeed();
void refreshCompassLcd();

//app
void setup() {
  digitalWrite(LED, LOW);
  setupLCDs();
  setupButtons();

  setupSerial();
  // setupMPU();
  setupCompass();

  printMessage((char*) "HELL0", (char*) "M0T0");
  delay(2000);
}

void loop() {  
  // updateCompass();
  handleButtons();
  updateTrip();
  tripLCD.print(trip, 2);
  refreshCompassLcd();
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

void setupCompass() {
  printMessage((char*) "COMPASS", (char*) "-0001-");
  while (!compass.begin())
  {
    delay(500);
  }
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setDataRate(HMC5883L_DATARATE_75HZ);
  compass.setOffset(-32, 59);
  printMessage((char*) "COMPASS", (char*) "0");
}

void setupMPU() {
  printMessage((char*) "MPU", (char*) "-0002-");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);
  mpu.setThreshold(0);
  printMessage((char*) "MPU", (char*) "0");
}

void setupSerial() {
  Serial.begin(9600);

  printMessage((char*) "SERIAL", (char*) "0");
}

void setupLCDs() {
  compassLCD.begin(12, 11, 10); // (cs, wr, Data, backlight)
  tripLCD.begin(9, 8, 7); // (cs, wr, Data, backlight)
  tripLCD.clear();
  compassLCD.clear();

  printMessage((char*) "LCD", (char*) "0");
}

void updateCompass() {
  Vector mag = compass.readNormalize();
  // Vector acc = mpu.readScaledAccel();

  // float headingRad = tiltCompensate(mag, acc);
  // if (headingRad == -1000) {
  float headingRad = atan2(mag.YAxis, mag.XAxis);
  // }
  
  float heading = radToDeg(headingRad);
  heading = applyDeclinationAngle(heading);
  
  unsigned long now = millis();
  if (now - lastCompassRefreshInMillis > COMPASS_LCD_REFRESH_PERIOD_MS) {
    lastCompassRefreshInMillis = now;
    compassLCD.print(heading, (char*) "%5u*", 0);
  }
}

void handleButtons() {
  unsigned long now = millis();
  unsigned long tripBtnLastPressedPeriod = now - lastTripBtnPressedInMillis;
  if (tripBtnLastPressedPeriod < BUTTON_DEAD_TIME_PERIOD_MS) {
    return;
  }
  
  handleTripButton(TRIP_DOWN, -1 * TRIP_STEP);
  handleTripButton(TRIP_UP, TRIP_STEP);
  handleResetButton();
}

void updateTrip() {
  trip = revs * WHEEL_TRIP;
}

void updateSpeed(unsigned long now) {
  speed = round((WHEEL_TRIP/(now - lastWheelPulseInMillis)/1000.0)*3600.0);
}

void handleTripButton(unsigned int BUTTON, float tripStep) {
  unsigned long now = millis();
  unsigned int buttonState = digitalRead(BUTTON);
  if (buttonState == LOW) {
    trip += tripStep; 
    lastTripBtnPressedInMillis = now;
  }
}

void handleResetButton() {
  unsigned long now = millis();
  unsigned int tripResetBtnState = digitalRead(TRIP_RESET);
  if (tripResetBtnState == HIGH) { //reset btn not pressed
    lastResetBtnPressedInMillis = now; //reset clock when not pressed
    batteryLevel = 0;
  }
  unsigned long resetBtnPressingTime = now - lastResetBtnPressedInMillis;
  if (trip > 0.0 && resetBtnPressingTime >= RESET_TIMEOUT) {
    trip = 0.0;
    batteryLevel = 0;
  }

  if (resetBtnPressingTime > SECOND)  {
    batteryLevel = 1;
  }
  if (resetBtnPressingTime > TWO_SECONDS)  {
    batteryLevel = 2;
  }
  if (resetBtnPressingTime > THREE_SECONDS)  {
    batteryLevel = 3;
  }
  tripLCD.setBatteryLevel(batteryLevel);
}

void wheelPulse() {
  unsigned long now = millis();
  if (now - lastWheelPulseInMillis > WHEEL_PULSE_DEAD_TIME_PERIOD_MS) {
    revs++;
    updateSpeed(now);
  }
  lastWheelPulseInMillis = now;
}

float radToDeg(float rad) {
  return rad * 180/M_PI;
}

float applyDeclinationAngle(float degHeading) {
  return modulo(round(degHeading + DECLINATION_ANGLE_DEG), 360);
}

int modulo(int x, int N){
    return (x % N + N) %N;
}

void printMessage(char* tripMsg, char* compassMsg) {
  tripLCD.print(tripMsg);
  compassLCD.print(compassMsg);
}

float tiltCompensate(Vector mag, Vector normAccel)
{
  // Pitch & Roll 
  
  float roll;
  float pitch;
  
  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }
  
    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  
  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
 
  return atan2(Yh, Xh);
}

void refreshCompassLcd() {
  unsigned long now = millis();
  if (now - lastCompassRefreshInMillis > COMPASS_LCD_REFRESH_PERIOD_MS) {
    lastCompassRefreshInMillis = now;
    compassLCD.print(speed, (char*) "%u", 0);
  }
}