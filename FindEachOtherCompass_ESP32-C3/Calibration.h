// The magnetometer on the BN-880 is 100% useless without calibration. We require both hard-iron and soft-iron calibration
// otherwise the device will simply point in a very wrong direction.

#ifndef _CALIBRATION
#define _CALIBRATION

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accelerometer
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

// Placing the device completely flat, doesn't show zero pitch and roll, therefore we offset the values to get zero when it's flat
// Method: Place device flat, print the pitch and roll values on Serial and use that as offset
void ApplyPitchRollCalibration(float &pitch, float &roll)
{
#if DEVICE == 0
   pitch -= 6.0f * DEG_TO_RAD;
   roll += 3.0f * DEG_TO_RAD;
#else
  pitch -= 10.0f * DEG_TO_RAD;
  roll -= 3.0f * DEG_TO_RAD;
#endif
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hard-iron Calibration for Magnetometer
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

// Compass tilt correction doesn't work without hard-iron calibration
// We simply need to find the min and max magnetometer values in all rotations
// Method:
//    1) Define RESET_HARD_IRON_CALIBRATION and boot the device. This will reset min/max values on the EPROM
//    2) Define CALIBRATE_HARD_IRON then slowly rotate the device around each axis, i.e. yaw, pitch and roll which will record min/max values
//    3) Copy the min/max values from the Serial output
//    4) Define these values in the constants below
//    5) Define APPLY_HARD_IRON_CALIBRATION to apply the calibration values

#ifdef CALIBRATE_HARD_IRON
// During calibration we'll determine these values, so they are variables
vec3 minHardIronCalibration;
vec3 maxHardIronCalibration;
#else
// After calibration we define the values as constants for each device
#if DEVICE == 0
static const vec3 minHardIronCalibration = { -68.82, -71.82, -77.24 };
static const vec3 maxHardIronCalibration = {  87.27, 53.18, 73.47 };
#else
static const vec3 minHardIronCalibration = { -69.64, -61.45, -123.78 };
static const vec3 maxHardIronCalibration = {  68.64, 58.73, 27.04 };
#endif
#endif

#ifdef CALIBRATE_HARD_IRON
void ResetHardIronCalibration()
{
  static const vec3 resetMin = { 32000, 32000, 32000 };
  static const vec3 resetMax = { -320000, -32000, -32000 };

  // Write reset min/max values to EPROM
  persistedData.begin("FindEachOther", false);
  persistedData.putDouble("calMagMinX", resetMin.x);
  persistedData.putDouble("calMagMinY", resetMin.y);
  persistedData.putDouble("calMagMinZ", resetMin.z);
  persistedData.putDouble("calMagMaxX", resetMax.x);
  persistedData.putDouble("calMagMaxY", resetMax.y);
  persistedData.putDouble("calMagMaxZ", resetMax.z);
  persistedData.end();

  DebugPrintf("Magnetometer calibration data saved: %3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\n", resetMin.x, resetMin.y, resetMin.z, resetMax.x, resetMax.y, resetMax.z);
}

void CalibrateHardIron()
{
  unsigned long start = millis();
  DisplayMessage("Calibrating...");

  // Load previous calibration values
  {
    persistedData.begin("FindEachOther", false);
    minHardIronCalibration.x = persistedData.getDouble("calMagMinX", 0.0);
    minHardIronCalibration.y = persistedData.getDouble("calMagMinY", 0.0);
    minHardIronCalibration.z = persistedData.getDouble("calMagMinZ", 0.0);
    maxHardIronCalibration.x = persistedData.getDouble("calMagMaxX", 0.0);
    maxHardIronCalibration.y = persistedData.getDouble("calMagMaxY", 0.0);
    maxHardIronCalibration.z = persistedData.getDouble("calMagMaxZ", 0.0);
    persistedData.end();

    DebugPrintf("Magnetometer calibration data loaded: %3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\n",
                  minHardIronCalibration.x, minHardIronCalibration.y, minHardIronCalibration.z, maxHardIronCalibration.x, maxHardIronCalibration.y, maxHardIronCalibration.z);

    // If calibration values are beyond this value, then something went wrong, so we'll reset
    const double checkCalibration = 200.0;
    if (minHardIronCalibration.x < -checkCalibration || minHardIronCalibration.y < -checkCalibration || minHardIronCalibration.z < -checkCalibration ||
        maxHardIronCalibration.x > checkCalibration || maxHardIronCalibration.y > checkCalibration || maxHardIronCalibration.z > checkCalibration)
    {
      ResetHardIronCalibration();
    }
  }

  // Loop until min/max values don't changes for 15 seconds. This gives us enough time to slowely rotate the device in all directions
  while (millis() - start < 15000UL)
  {
    sensors_event_t event;
    magnetometer.getEvent(&event);

    double mx = event.magnetic.x;
    double my = event.magnetic.y;
    double mz = event.magnetic.z;

    if (mx < minHardIronCalibration.x)
    {
      minHardIronCalibration.x = mx;
      start = millis();
    }

    if (mx > maxHardIronCalibration.x)
    {
      maxHardIronCalibration.x = mx;
      start = millis();
    }

    if (my < minHardIronCalibration.y)
    {
      minHardIronCalibration.y = my;
      start = millis();
    }

    if (my > maxHardIronCalibration.y)
    {
      maxHardIronCalibration.y = my;
      start = millis();
    }

    if (mz < minHardIronCalibration.z)
    {
      minHardIronCalibration.z = mz;
      start = millis();
    }

    if (mz > maxHardIronCalibration.z)
    {
      maxHardIronCalibration.z = mz;
      start = millis();
    }
  }

  DisplayMessage("Calibrating done");

  // Save updated calibration values
  {
    persistedData.begin("FindEachOther", false);
    persistedData.putDouble("calMagMinX", minHardIronCalibration.x);
    persistedData.putDouble("calMagMinY", minHardIronCalibration.y);
    persistedData.putDouble("calMagMinZ", minHardIronCalibration.z);
    persistedData.putDouble("calMagMaxX", maxHardIronCalibration.x);
    persistedData.putDouble("calMagMaxY", maxHardIronCalibration.y);
    persistedData.putDouble("calMagMaxZ", maxHardIronCalibration.z);
    persistedData.end();

    DebugPrintf("Magnetometer calibration data saved:\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\n",
                  minHardIronCalibration.x, minHardIronCalibration.y, minHardIronCalibration.z, maxHardIronCalibration.x, maxHardIronCalibration.y, maxHardIronCalibration.z);
  }
}
#endif

void ApplyHardIronCalibration(float &mx, float &my, float &mz)
{
  mx = (mx - minHardIronCalibration.x) / (maxHardIronCalibration.x - minHardIronCalibration.x) * 2.0 - 1.0;
  my = (my - minHardIronCalibration.y) / (maxHardIronCalibration.y - minHardIronCalibration.y) * 2.0 - 1.0;
  mz = (mz - minHardIronCalibration.z) / (maxHardIronCalibration.z - minHardIronCalibration.z) * 2.0 - 1.0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Soft-iron Calibration for Magnetometer
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

// Soft-iron calibration is normally done directly on the magnetometer's values. But, in this case, we do the
// non-linear calibration on the final bearing angle, which ensures we get close to 100% accuracy.
// Method:
//    1) Use an actual compass and determine true North
//    2) Define CALIBRATE_SOFT_IRON
//    3) Attach device to square item and align with true North
//    4) Make a note of the degrees shown on device and correlate it to zero degrees, i.e. measuredDegrees[] shows what you see
//        on the device and correctDegrees[] is what you are supposed to see.
//    5) Do this for each 45 degrees
//    6) As an example for Device 0, pointing to true North it actually showed 18 degrees instead of zero degrees,
//        and it showed 88 degrees instead of 90 degrees
//    7) Define APPLY_SOFT_IRON_CALIBRATION which will correlate the correct values and interpolate it between each 45 degrees

// Soft-iron calibration
static const int numLookupPoints = 12;

// A simple lookup table defining actual measurements and correct correct measurements. See documentation for
// how to find the values in these tables
#if DEVICE == 0
static const float measuredDegrees[numLookupPoints] = { -101, -66, -16, 44, 88, 128, 176, 223, 259, 294, 344, 404 };  // Magnetometer says this
static const float correctDegrees[numLookupPoints] = { -90, -45, 0, 45, 90, 135, 180, 225, 270, 315, 360, 405 };      // Actual compass says this
#else
static const float measuredDegrees[numLookupPoints] = { -52, -15, 18, 45, 69, 92, 118, 209, 308, 345, 378, 405 };     // Magnetometer says this
static const float correctDegrees[numLookupPoints] = { -90, -45, 0, 45, 90, 135, 180, 225, 270, 315, 360, 405 };      // Actual compass says this
#endif

// Apply soft-iron calibration to correct the magnetometer reading
float ApplySoftIronCalibration(const float inputAngleRadians)
{
  // Convert angle from [-180..180] to [0..360]
  float inputAngleDegrees = PositiveAngleDegrees(inputAngleRadians * RAD_TO_DEG);

  // Find the index into the lookup table
  int lookupTableIndex = 1;
  for (lookupTableIndex = 1; lookupTableIndex < numLookupPoints; lookupTableIndex++)
  {
    if (inputAngleDegrees < measuredDegrees[lookupTableIndex])
    {
      break;
    }
  }

  // Use uncalibrated input angle to determine the interpolation factor between two points in the lookup table
  float t = 1.0f - (measuredDegrees[lookupTableIndex] - inputAngleDegrees) / (measuredDegrees[lookupTableIndex] - measuredDegrees[lookupTableIndex - 1]);

  // Interpolate between the two correlating correct points in the lookup table
  float correctedAngle = lerpAngleDegrees(correctDegrees[lookupTableIndex - 1], correctDegrees[lookupTableIndex], t);

#ifdef CALIBRATE_SOFT_IRON
  DebugPrintf("Input Angle = %3.0f\tIndex = %d\tt = %1.1f\tCorrectedAngle = %3.0f\n", inputAngleDegrees, lookupTableIndex, t, correctedAngle);
#endif

  return correctedAngle * DEG_TO_RAD;
}

#endif