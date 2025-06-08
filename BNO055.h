#ifndef BNO055_HPP
#define BNO055_HPP

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/* Define Pin for BNO055 */
#define BNO055_SCL_PIN 22
#define BNO055_SDA_PIN 21

extern Adafruit_BNO055 BNO055;
std::array<float, 3> Get_BNO055_BasePoint();
std::array<float, 3> Read_Angle_ThreeAxes(float &Yaw, float &Pitch, float &Roll, std::array<float, 3> Base_Point);
std::array<float, 3> Read_Radian_ThreeAxes(float &Yaw, float &Pitch, float &Roll, std::array<float, 3> Base_Point);
void BNO055_Setup();
float degtorad(float degree);
float normalizeYaw(float yaw_deg);
void LimitedOfRangeYaw(float value, float centerYaw, float threshold);
void LimitedOfRangePitch(float value, float centerPitch, float threshold);
void LimitedOfRangeRoll(float value, float centerRoll, float threshold);
void PrintCalibrationStatus();
void SetBaseForwardVector();
int GetPitchDirectionFromQuat();
#endif