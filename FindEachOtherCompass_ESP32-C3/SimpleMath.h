#ifndef _SIMPLE_MATH
#define _SIMPLE_MATH

struct vec3
{
  double x, y, z;
};

inline double DotProduct(const vec3 A, const vec3 B)
{
  return (A.x * B.x) + (A.y * B.y) + (A.z * B.z);
}

void CrossProduct(const vec3 A, const vec3 B, vec3 &C)
{
  C.x = (A.y * B.z) - (A.z * B.y);
  C.y = (A.z * B.x) - (A.x * B.z);
  C.z = (A.x * B.y) - (A.y * B.x);
}

inline double Length(const vec3 A)
{
  return sqrt(DotProduct(A, A));
}

void Normalize(vec3 &A)
{
  const double norm = 1.0 / Length(A);
  A.x *= norm;
  A.y *= norm;
  A.z *= norm;
}

// Conevert from [-180..180] to [0..260]
float PositiveAngleRadians(const float angleRad)
{
  float angle = angleRad;

  if (angle < 0.0f)
  {
    angle += 2.0f * M_PI;
  }
    
  if (angle > 2.0f * M_PI)
  {
    angle -= 2.0f * M_PI;
  }
  
  return angle;
}

// Conevert from [-180..180] to [0..260]
float PositiveAngleDegrees(const float angleDeg)
{
  float angle = angleDeg;
  
  if (angle < 0.0f)
  {
    angle += 360.0f;
  }
    
  if (angle > 360.0f)
  {
    angle -= 360.0f;
  }
  
  return angle;
}

inline double lerp(const double A, const double B, const double t)
{
  return (A * (1.0 - t)) + (B * t);
}

float lerpAngleRadians(float A, float B, const float t)
{
  if ((B - A) < -M_PI)
  {
    A -= 2.0f * M_PI;
  }

  if ((B - A) > M_PI)
  {
    A += 2.0f * M_PI;
  }

  float interpolatedAngle = lerp(A, B, t);

  return PositiveAngleRadians(interpolatedAngle);
}

float lerpAngleDegrees(float A, float B, const float t)
{
  if ((B - A) < -180.0f)
  {
    A -= 360.0f;
  }

  if ((B - A) > 180.0f)
  {
    A += 360.0f;
  }

  float interpolatedAngle = lerp(A, B, t);

  return PositiveAngleDegrees(interpolatedAngle);
}

#endif