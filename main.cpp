//#include <Joystick.h>
//#include <Gimbal_High_Planner.h>
//#include <Motor_Control.h>
#include <BNO055.h>
#include <simpleKalmanFilter.h>
// Set up parameters for the BNO055 IMU
std::array<float, 3> BasePoint;
float Yaw, Pitch, Roll;

void setup() {
    Serial.begin(115200);
    BNO055_Setup();                             // IMU Initialization
    BasePoint = Get_BNO055_BasePoint();         // Get the base point of the IMU
    Serial.println("Đã lấy base point.");
    SetBaseForwardVector();
}

void loop() {
    // Read angle of IMU
    Read_Angle_ThreeAxes(Yaw, Pitch, Roll, BasePoint); // Read angles from the IMU and apply Kalman filter
    // Read Radian of IMU
    
    //Read_Radian_ThreeAxes(Yaw, Pitch, Roll, BasePoint); // Read angles from the IMU and apply Kalman filter
    PrintCalibrationStatus();
    LimitedOfRangeYaw(Yaw, 0.0, 4.0f);
    LimitedOfRangePitch(Pitch, 90.0, 1.0f);
    LimitedOfRangeRoll(Roll, 0.0, 3.0f);
    
    int dir = GetPitchDirectionFromQuat();
    if (dir == 1) {
        Serial.println("PITCH: +1 (Cúi xuống)");
    } else if (dir == -1) {
        Serial.println("PITCH: -1 (Ngửa lên)");
    } else {
        Serial.println("PITCH:  0 (Ổn định)");
    }

 
    delay(500); // Delay to avoid flooding the serial output
}
