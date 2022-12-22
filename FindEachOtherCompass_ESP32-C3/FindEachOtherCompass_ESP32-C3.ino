// This is a special compass that doesn't just show North, but also points into the direction of someone else
// so that you can easiy find each other. This is very useful for outdoor team sports like Airsoft.

// TODO:
// - Find the MAC addresses of your own devices and add it to OtherDeviceMACAddress[] in TwoWayComms.h
// - Find the magnetic declination angle of your own location and replace it in ApplyDeclinationAngle()

#define DEVICE 0      // Define this when compiling for device 0
//#define DEVICE 1    // Define this when compiling for device 1

// No need to try send something to Serial if not needed, so comment out when not connected to a PC
//#define DEBUG 1                       // Without this define, nothing will get printed to Serial
//#define DEBUG_LOCATION 1              // Prints debug info about GPS and target locations
//#define DEBUG_ORIENTATION 1           // Prints debug info about magnetometer, tilt correction, etc.
//#define DEBUG_TWO_WAY_COMMS 1         // Prints debug info about connection between two devices

#define APPLY_PITCH_ROLL_CALIBRATION 1

//#define CALIBRATE_HARD_IRON 1
//#define RESET_HARD_IRON_CALIBRATION 1
#define APPLY_HARD_IRON_CALIBRATION 1

//#define CALIBRATE_SOFT_IRON 1
#define APPLY_SOFT_IRON_CALIBRATION 1

#define APPLY_TILT_CORRECTION 1
#define ACCELEROMETER_DRIVES_DISPLAY 1
#define SAVE_POWER 1

// Read GPS data
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Read magnetometer and accelerometer
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>

// Draw to the screen
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define I2C_ADDRESS_SSD1306_OLED 0x3C

// Read/Write persistant data using EEPROM
#include <Preferences.h>
Preferences persistedData;

#include "Utils.h"
#include "Buzzer.h"
#include "RepeatTimer.h"
#include "StateToggleTimer.h"
#include "SimpleMath.h"
#include "TwoWayComms.h"

// Software serial connection to the GPS device
SoftwareSerial softwareSerial(RX, TX);

// OLED display
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPS
TinyGPSPlus GPS;
static const uint32_t GPSBaud = 9600; // Baud rate for BN-880

// Magnetometer and accelerometer
Adafruit_HMC5883_Unified magnetometer(123);
Adafruit_MPU6050 mpu;
Adafruit_Sensor* pAccelerometer;

// Buzzer
#define BUZZER_PIN D8
Buzzer buzzer;

// Keep track of valid GPS data
bool bIsGPSLocationValid = false;
bool bIsTargetLocationRecent = false;
uint saveValidGPSLocationCounter = 0;
uint saveValidTargetLocationCounter = 0;

bool bIsDisplayOn = true;
bool bIsConnected = false;

// Manage indicators on display
StateToggleTimer flashConnected(250);       // Flash when not connected to other decide
StateToggleTimer flashMyLocation(250);      // Flash when GPS doesn't know my own location
StateToggleTimer flashTargetLocation(250);  // Flash when we don't have a recent location of the other device

// Do expensive calculations less frequently when we want to save power running on a battery
#ifdef SAVE_POWER
RepeatTimer timerCalcBearing(150);
RepeatTimer timerCalcPithRoll(500);
RepeatTimer timerSendMyLocation(5000);
#else
RepeatTimer timerCalcBearing(50);
RepeatTimer timerCalcPithRoll(50);
RepeatTimer timerSendMyLocation(2000);
#endif

// All timers use millis(), no need to call it multiple times, just call it once per loop()
unsigned long now = 0;

// Buffer for text messages
char info[128];

double myLongitude = 0.0;
double myLatitude = 0.0;

double targetLongitude = 0;
double targetLatitude = 0;

float roll = 0.0f;
float pitch = 0.0f;
float bearingAngle = 0.0f;

#ifdef CALIBRATE_SOFT_IRON
static const float lowPassFilterLerp = 0.25f;   // Stronger filter for more stable readings during calibration, but is slower to update
#else
static const float lowPassFilterLerp = 0.40f;   // Weaker filter for more responsive updates
#endif

// Monochrome bitmap icon indicating if devices are connected
// This app works well: https://lcd-image-converter.riuson.com/en/about/
const unsigned char PROGMEM connectedIcon[] =
{
    0x00, 0x70, 0x01, 0x80, 0x06, 0x00, 0x08, 0x70, 0x11, 0x80, 0x22, 0x00,
    0x24, 0x30, 0x4c, 0xc0, 0x49, 0x00, 0x93, 0x20, 0x92, 0x70, 0x92, 0x20
};

// Read GPS data for a certain amount of time
void BufferGPSData(const unsigned long duration)
{
  const unsigned long start = millis();

  do 
  {
    while (softwareSerial.available())
    {
      GPS.encode(softwareSerial.read());
    }
  } while ((millis() - start) < duration);
}

// Show a message on the display
void DisplayMessage(const char* message, const bool bShowOnSerial = true)
{
#ifdef DEBUG
  // When debugging, we can send the info to Serial
  if (bShowOnSerial)
  {
    DebugPrintln(message);
  }
#endif

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(message);
  display.display();
  BufferGPSData(1000);    // Display the message for one second, but continue to buffer GPS data
}

#include "Calibration.h"

// Returns a tilt corrected bearing given accelerometer data and magnetometer data
// NOTE: Tilt correction doesn't work well if you don't apply hard-iron calibration
// https://github.com/pololu/lsm303-arduino/blob/master/LSM303.h
float CalcTiltCorrectedBearing(const vec3 a, const vec3 m)
{ 
  static const vec3 from = { -1.0, 0.0, 0.0 };

  vec3 east, tiltCorrectedNorth;

  CrossProduct(m, a, east);
  Normalize(east);
  CrossProduct(a, east, tiltCorrectedNorth);
  Normalize(tiltCorrectedNorth);  
  float bearing = atan2(DotProduct(east, from), DotProduct(tiltCorrectedNorth, from));

#ifdef DEBUG_ORIENTATION
  DebugPrintf("\tTilt Corrected Bearing %3.0f\n", bearing * RAD_TO_DEG);
#endif

  return bearing;
}

void GetMyLocation()
{
  // Buffer some GPS data
  BufferGPSData(150);

  // Since GPS data is buffered, we might not have valid location data, so always first check
  if (GPS.location.isValid())
  {
#ifdef DEBUG_LOCATION
    if (!bIsGPSLocationValid)
    {
      DebugPrintln("Invalid => Valid GPS data");
    }
#endif

    bIsGPSLocationValid = true;
    flashMyLocation.Stop();

    double longitude = GPS.location.lng();
    double latitude = GPS.location.lat();

    // Low-pass filter the location
    myLongitude = lerp(myLongitude, longitude, lowPassFilterLerp);
    myLatitude = lerp(myLatitude, latitude, lowPassFilterLerp);

    // Save a last known good location so that the next time the device starts, we can use this location while waiting for the GPS to find
    // an actual location. But, don't save the location after just one valid reading, wait until we have a number of valid data readings
    saveValidGPSLocationCounter++;
    if (saveValidGPSLocationCounter == 10)
    {
      persistedData.begin("FindEachOther", false);
      persistedData.putDouble("lastLongitude", longitude);
      persistedData.putDouble("lastLatitude", latitude);

      sprintf(info, "Current location saved: %.3lf %.3lf\n", longitude, latitude);
      DisplayMessage(info);
      buzzer.Buzz(100);
    }
  }
  else
  {
    // If previously we had a valid location, but not anymore, start flashing the center dot to show that we don't have an actual location from the GPS
    if (bIsGPSLocationValid)
    {
      flashMyLocation.Start();
#ifdef DEBUG_LOCATION
      DebugPrintln("Valid => Invalid GPS data");
#endif
    }

    bIsGPSLocationValid = false;
    saveValidGPSLocationCounter = 0;

#ifdef DEBUG_LOCATION
    DebugPrintf("Invalid location %.3lf, %.3lf\n", myLongitude, myLatitude);
#endif
  }

  // Send my location every N seconds. If we don't have valid GPS location, we should still have a last known good location that was saved previously
  if (timerSendMyLocation.RanOut(now))
  {
    TwoWayComms::SendMyLocation(myLongitude, myLatitude, bIsGPSLocationValid);

#ifdef DEBUG_TWO_WAY_COMMS
    if (!bIsGPSLocationValid)
    {
      DebugPrintln("I don't have a valid GPS location");
    }
#endif
  }
}

// http://www.magnetic-declination.com/
inline float ApplyDeclinationAngle(const float angleRadians)
{
  // Magnetic declination of my location: +15° 18' Positive East
  // 15 + 18/60 => 15.3 Deg = 0.267 Rad
  static const float declinationAngle = 0.267f;
  return angleRadians + declinationAngle;
}

// Calculate compass bearing that points to North
void CalcBearingAngle()
{
  // We can save battery power by doing expensive calculations less frequently. To do this, we use a repeat timer.
  if (!timerCalcBearing.RanOut(now))
  {
    return;
  }

  float ax, ay, az, mx, my, mz;               // Immediate data
  float f_ax, f_ay, f_az, f_mx, f_my, f_mz;   // Filtered data

  sensors_event_t accel;
  sensors_event_t mag;
  
  // Low pass filter of 10 readings
  for (int i = 0; i < 10; i++)
  {
    pAccelerometer->getEvent(&accel);
    magnetometer.getEvent(&mag);

    // uTs
    mx = mag.magnetic.x;
    my = mag.magnetic.y;
    mz = mag.magnetic.z;

    // m/s
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;

    if (i == 0)
    {
      // Init the filter
      f_ax = ax;
      f_ay = ay;
      f_az = az;
      f_mx = mx;
      f_my = my;
      f_mz = mz;
    }
    else
    {
      // Low-Pass filter
      f_ax = lerp(f_ax, ax, lowPassFilterLerp);
      f_ay = lerp(f_ay, ay, lowPassFilterLerp);
      f_az = lerp(f_az, az, lowPassFilterLerp);
      f_mx = lerp(f_mx, mx, lowPassFilterLerp);
      f_my = lerp(f_my, my, lowPassFilterLerp);
      f_mz = lerp(f_mz, mz, lowPassFilterLerp);
    }

    delay(5);
  }

  // To save battery, only calculate roll and pitch when timer runs out
  if (timerCalcPithRoll.RanOut(now))
  {
    roll = atan2(f_ay, sqrt((f_ax * f_ax) + (f_az * f_az)));
    pitch = atan2(f_ax, sqrt((f_ay * f_ay) + (f_az * f_az)));

#ifdef APPLY_PITCH_ROLL_CALIBRATION
    ApplyPitchRollCalibration(pitch, roll);
#endif
  }

#ifdef SAVE_POWER
  // We can further save power by not doing calculations if the display isn't even on. But, since the display is turned
  // on/off by looking at the pitch and roll, we first need to calculate pitch and roll
  if (!bIsDisplayOn)
  {
    return;
  }
#endif

#ifdef APPLY_HARD_IRON_CALIBRATION
  ApplyHardIronCalibration(f_mx, f_my, f_mz);
#endif

#ifdef APPLY_TILT_CORRECTION
  vec3 a = { f_ax, f_ay, f_az };
  vec3 m = {-f_mx, f_my, f_mz };
  float currentBearing = CalcTiltCorrectedBearing(a, m);
#else
  float currentBearing = atan2(-f_my, f_mx);
#endif

#ifdef APPLY_SOFT_IRON_CALIBRATION
  currentBearing = ApplySoftIronCalibration(currentBearing);
#endif

#ifndef CALIBRATE_SOFT_IRON
  currentBearing = ApplyDeclinationAngle(currentBearing);
  currentBearing = PositiveAngleRadians(currentBearing);
#endif

  // Low-pass filter the bearing angle
  bearingAngle = lerpAngleRadians(bearingAngle, currentBearing, lowPassFilterLerp);

#ifdef DEBUG_ORIENTATION
   DebugPrintf("Yaw %3.0lf\tPitch %3.0lf\tRoll %3.0lf\n", currentBearing * RAD_TO_DEG, pitch * RAD_TO_DEG, roll * RAD_TO_DEG);
#endif
}

// Draw a needle on the face, e.g. the needle pointing to the other device or North, and potentially a dot at the end of the needle
void DrawNeedle(const int centerX, const int centerY, const float angle, const int radius, const int size, const bool bDrawDot = true)
{
  // Calc end position of needle
  int positionX = int(centerX + (sin(angle) * radius));
  int positionY = int(centerY - (cos(angle) * radius));

  display.drawLine(centerX, centerY, positionX, positionY, WHITE);

  if (bDrawDot)
  {
    display.fillCircle(positionX, positionY, size, WHITE);
  }
}

// Draw the face, this includes
//    - four shorter needles pointing to North, East, South, West
//    - long needle pointing to the other device
//    - center dot which can flash if the GPS location isn't valid
//    - dot at the end of the long needle which can flash if the other device location isn't recent
//    - distance to other device
//    - icon showing if devices are connected, which can flash if they are not
void DrawFace()
{
  float courseAngleToTarget = TinyGPSPlus::courseTo(myLatitude, myLongitude, targetLatitude, targetLongitude) * DEG_TO_RAD;
  double distanceToTarget = TinyGPSPlus::distanceBetween(myLatitude, myLongitude, targetLatitude, targetLongitude);

#ifndef CALIBRATE_SOFT_IRON
  courseAngleToTarget = ApplyDeclinationAngle(courseAngleToTarget);
  courseAngleToTarget = PositiveAngleRadians(courseAngleToTarget);
#endif

#ifdef DEBUG_LOCATION
  if (distanceToTarget > 1000)
  {
    DebugPrintf("%s (%lf, %lf) => (%lf, %lf) = %lf\n", bIsGPSLocationValid ? "Recent" : "Loaded", myLongitude, myLatitude, targetLongitude, targetLatitude, distanceToTarget);
  }
#endif

#ifdef CALIBRATE_SOFT_IRON
  // Show bearing
  sprintf(info, "%d d\n", int(bearingAngle * RAD_TO_DEG));
#else
  // Show distance
  sprintf(info, "%d m\n", int(distanceToTarget));
#endif

  display.setCursor(2, 0);
  display.setTextSize(2);
  display.print(info);

  static const int dotRadius = 3;
  static const int radius = SCREEN_HEIGHT / 2;

#ifdef CALIBRATE_SOFT_IRON
  static const int smallRadius = radius / 2;
#else
  static const int smallRadius = radius / 2.5;
#endif

  static const int centerX = SCREEN_HEIGHT / 2;
  static const int centerY = (SCREEN_WIDTH / 2) + radius - dotRadius;

  // Flash the center dot if we don't yet have a valid GPS location
  if (flashMyLocation.IsOn())
  {
    display.fillCircle(centerX, centerY, dotRadius, WHITE);
  }

#ifndef CALIBRATE_SOFT_IRON
  // Draw needle pointing towards target
  float angle = courseAngleToTarget - bearingAngle;
  DrawNeedle(centerX, centerY, angle, radius, dotRadius, flashTargetLocation.IsOn());
#endif

  static const float quadrand = M_PI / 2.0f;

  // Need to draw needle in the opposite direction, so use negative angle
  float north = bearingAngle;

#ifdef CALIBRATE_SOFT_IRON
  DrawNeedle(centerX, centerY, -north, radius, 2, true);
#else
  DrawNeedle(centerX, centerY, -north, smallRadius, 2, true);
#endif

  float east = bearingAngle + quadrand;
  DrawNeedle(centerX, centerY, -east, smallRadius, 1, false);

  float south = bearingAngle + quadrand * 2.0f;
  DrawNeedle(centerX, centerY, -south, smallRadius, 1, false);

  float west = bearingAngle + quadrand * 3.0f;
  DrawNeedle(centerX, centerY, -west, smallRadius, 1, false);

  if (flashConnected.IsOn())
  {
    display.drawBitmap(SCREEN_HEIGHT - 8, 0, connectedIcon, 12, 12, SSD1306_WHITE);
  }
}

// We can save battery power by switching the display off when device is too far rotated
void SwitchDisplayOnOff()
{
  static const float rollThreshold = 30.0f * DEG_TO_RAD;
  static const float pitchThreshold = 30.0f * DEG_TO_RAD;

  if (abs(roll) > rollThreshold ||
      abs(pitch) > pitchThreshold)
  {
    if (bIsDisplayOn)
    {
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      bIsDisplayOn = false;
    }
  }
  else
  {
    if (!bIsDisplayOn)
    {
      display.ssd1306_command(SSD1306_DISPLAYON);
      bIsDisplayOn = true;
    }
  }
}

void setup()
{
  // Open hardware serial port to communicate with PC
#ifdef DEBUG
  {
    Serial.begin(115200);
    delay(200);
    DebugPrintln("\n\n********* FindEachOther for ESP32-C3 *********\n");
    DebugPrintln("Hardware serial port connected");
  }
#endif

  // Open software serial port to communicate with GPS
  {
    softwareSerial.begin(GPSBaud);
    DebugPrintln("Software serial port connected");
  }

  // Initiailze display
  {
    if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS_SSD1306_OLED))
    {
      DebugPrintln("ERROR: OLED initialization failed!");
      while(true);
    }

    display.setTextColor(WHITE);
    display.ssd1306_command(SSD1306_DISPLAYON);

    DebugPrintln("Display initialized successfully");
  }

  DisplayMessage("Initializing ...");

  // Initialize comms
  {
    if (!TwoWayComms::Setup())
    {
      DisplayMessage("ERROR: 2-Way comms setup failed!");
      while(true);
    }

    DisplayMessage("2-Way comms started");
  }

  // Initialize accelerometer
  {
    if (!mpu.begin())
    {
      DisplayMessage("ERROR: MPU not detected!");
      while (true);
    }

    pAccelerometer = mpu.getAccelerometerSensor();

    if (!pAccelerometer)
    {
      DisplayMessage("ERROR: Accelerometer not detected!");
      while (true);
    }

    DisplayMessage("Accelerometer initialized");
  }

  // Initialize magnetometer
  {
    if (!magnetometer.begin())
    {
      DisplayMessage("ERROR: Magnetometer not detected!");
      while(true);
    }

    DisplayMessage("Magnetometer initialized");
  }

  // Read the last known good GPS location as a valid starting position while GPS is trying to find satelites
  {
    persistedData.begin("FindEachOther", false);
    myLongitude = persistedData.getDouble("lastLongitude", 0.0);
    myLatitude = persistedData.getDouble("lastLatitude", 0.0);
    targetLongitude = persistedData.getDouble("targetLastLong", 0.0);
    targetLatitude = persistedData.getDouble("targetLastLat", 0.0);
    persistedData.end();

    sprintf(info, "Prev location loaded: %.3lf %.3lf", myLongitude, myLatitude);
    DisplayMessage(info);
  }

  bIsGPSLocationValid = false;
  flashMyLocation.Start();

  bIsTargetLocationRecent = false;
  flashTargetLocation.Start();

  bIsConnected = false;
  flashConnected.Start();

  display.setRotation(1);   // Portrait mode
  display.setTextColor(WHITE);

  // Initialize buzzer
  {
    buzzer.Initialize(BUZZER_PIN);
    DisplayMessage("Buzzer initialized");
    buzzer.Buzz(100);
  }

#ifdef RESET_HARD_IRON_CALIBRATION
  ResetHardIronCalibration();
#endif

#ifdef CALIBRATE_HARD_IRON
  CalibrateHardIron();
#endif
}

void loop()
{ 
  display.clearDisplay();

  // All timers use millis(), no need to call it multiple times
  now = millis();

  // Update timers
  flashConnected.Update(now);
  flashMyLocation.Update(now);
  flashTargetLocation.Update(now);

  buzzer.Update(now);

  TwoWayComms::UpdateStatus(now);
  
  if (TwoWayComms::IsConnected())
  {
    // If we were previously not connected, but now we are, stop flashing the connected icon and buzz
    if (!bIsConnected)
    {
      bIsConnected = true;
      flashConnected.Stop();
      buzzer.Buzz(100);
      DisplayMessage("Connection established");
    }
  }
  else
  {
    // If we were previosly connected, but not anymore, start flashing the connected icon and buzz
    if (bIsConnected)
    {
      bIsConnected = false;
      flashConnected.Start();
      buzzer.Buzz(100);
      DisplayMessage("Connection lost");
    }   
  }

  if (TwoWayComms::bDataReceived)
  {
    targetLongitude = TwoWayComms::otherDeviceData.longitude;
    targetLatitude = TwoWayComms::otherDeviceData.latitude;

    // If we receive a location, it doesn't mean the location was acutually obtained by the other device's GPS
    // If bIsLocationRecent == false, it means that the other device loaded it's own previously saved location and sent that
    if (TwoWayComms::otherDeviceData.bIsLocationRecent && !bIsTargetLocationRecent)
    {
      bIsTargetLocationRecent = true;
      flashTargetLocation.Stop();
    }

    // Save a last known good target location so that the next time the device starts, we can use this location while waiting for the other device
    // to connect and send a location. But, don't save the location after just one received target location, wait until we have a number of them
    saveValidTargetLocationCounter++;
    if (saveValidTargetLocationCounter == 10)
    {
      persistedData.begin("FindEachOther", false);
      persistedData.putDouble("targetLastLong", targetLongitude);
      persistedData.putDouble("targetLastLat", targetLatitude);
      persistedData.end();

      sprintf(info, "Current target location saved: %.2lf %.2lf\n", targetLongitude, targetLatitude);
      DisplayMessage(info);
    }
  }
  else
  {
    saveValidTargetLocationCounter = 0;

    // We previously recevied a valid target position, but now we don't have one anymore
    if (bIsTargetLocationRecent)
    {
      flashTargetLocation.Start();
      bIsTargetLocationRecent = false;
    }
  }
  
  GetMyLocation();
  CalcBearingAngle();
  DrawFace();

#ifdef DEBUG
  // If GPS didn't give data for 5 seconds, something is probably wrong
  if (now > 5000 && GPS.charsProcessed() < 10)
  {
    DisplayMessage("ERROR: No GPS data received. Check wiring.");
  }
#endif

#ifdef DEBUG_TWO_WAY_COMMS
  {
    if (TwoWayComms::IsConnected())
    {
      sprintf(info, "Connected\nping %d", TwoWayComms::receivedDataCounter);
    }
    else
    {
      sprintf(info, "NOT Connected");
    }

    display.setCursor(0, 32);
    display.setTextSize(1);
    display.print(info);
  }
#endif

  display.display();

#ifdef ACCELEROMETER_DRIVES_DISPLAY
  SwitchDisplayOnOff();
#endif

}
