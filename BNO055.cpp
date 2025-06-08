#include <BNO055.h>
#include <SimpleKalmanFilter.h>
#include <cmath>
#include <math.h>

//-----------------------------//
//     CẤU HÌNH PHẦN CỨNG      //
//-----------------------------//
#define BNO055_SDA_PIN 21
#define BNO055_SCL_PIN 22

Adafruit_BNO055 BNO055 = Adafruit_BNO055(55, 0x28);

//-----------------------------//
//     KALMAN CHO TỪNG TRỤC    //
//-----------------------------//
SimpleKalmanFilter kalmanYaw(0.02, 0.1, 0.5);
SimpleKalmanFilter kalmanPitch(0.02, 0.1, 0.5);
SimpleKalmanFilter kalmanRoll(0.02, 0.1, 0.5);

//-----------------------------//
//        HÀM TIỆN ÍCH         //
//-----------------------------//

float degtorad(float degree) {
    return degree * PI / 180.0;
}

float normalizeYaw(float yaw_deg) {
    if (yaw_deg > 180.0f)
        return yaw_deg - 360.0f;
    else
        return yaw_deg;
}

//-----------------------------//
//     KHỞI TẠO CẢM BIẾN       //
//-----------------------------//
void BNO055_Setup() {
    Wire.begin(BNO055_SDA_PIN, BNO055_SCL_PIN);

    if (!BNO055.begin()) {
        Serial.println("Failed to connect, please verify the connections!");
        while (1);
    }

    delay(1000);
    BNO055.setExtCrystalUse(true);
    Serial.println("BNO055 initialized successfully!");
}

//-----------------------------//
//      LẤY GÓC GỐC BAN ĐẦU    //
//-----------------------------//
std::array<float, 3> Get_BNO055_BasePoint() {
    std::array<float, 3> Base_Point;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);

    Base_Point[0] = euler.z();                        // Roll
    Base_Point[1] = euler.y();                        // Pitch
    Base_Point[2] = normalizeYaw(euler.x());          // Yaw (đã chuẩn hóa)
    return Base_Point;
}

//---------------------------------------------//
//      ĐỌC GÓC ĐỘ SAU KALMAN + SO VỚI GỐC GỐC //
//---------------------------------------------//
std::array<float, 3> Read_Angle_ThreeAxes(float &Yaw, float &Pitch, float &Roll, std::array<float, 3> Base_Point) {
    std::array<float, 3> Current_Rotation;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);

    float rawYaw   = normalizeYaw(euler.x()) - Base_Point[2];
    float rawPitch = euler.y() - Base_Point[1];
    float rawRoll  = euler.z() - Base_Point[0];

    Yaw   = kalmanYaw.update(rawYaw);
    Pitch = kalmanPitch.update(rawPitch);
    Roll  = kalmanRoll.update(rawRoll);

    Current_Rotation[0] = Yaw;
    Current_Rotation[1] = Pitch;
    Current_Rotation[2] = Roll;

    Serial.print("YAW: ");   Serial.println(Yaw);
    Serial.print("PITCH: "); Serial.println(Pitch);
    Serial.print("ROLL: ");  Serial.println(Roll);

    return Current_Rotation;
}

//---------------------------------------------//
//      ĐỌC GÓC RADIAN SAU LỌC KALMAN          //
//---------------------------------------------//
std::array<float, 3> Read_Radian_ThreeAxes(float &Yaw, float &Pitch, float &Roll, std::array<float, 3> Base_Point) {
    std::array<float, 3> Current_Rotation;
    imu::Vector<3> euler = BNO055.getVector(Adafruit_BNO055::VECTOR_EULER);

    float rawYaw   = normalizeYaw(euler.x()) - Base_Point[2];
    float rawPitch = euler.y() - Base_Point[1];
    float rawRoll  = euler.z() - Base_Point[0];

    Yaw   = degtorad(kalmanYaw.update(rawYaw));
    Pitch = degtorad(kalmanPitch.update(rawPitch));
    Roll  = degtorad(kalmanRoll.update(rawRoll));

    Current_Rotation[0] = Yaw;
    Current_Rotation[1] = Pitch;
    Current_Rotation[2] = Roll;

    Serial.print("YAW (rad): ");   Serial.println(Yaw);
    Serial.print("PITCH (rad): "); Serial.println(Pitch);
    Serial.print("ROLL (rad): ");  Serial.println(Roll);

    return Current_Rotation;
}

//----------------------------------------------//
//       GIỚI HẠN GÓC & XÁC ĐỊNH CHIỀU QUAY      //
//----------------------------------------------//
void LimitedOfRangeYaw(float value, float centerYaw, float threshold) {
    float diff = value - centerYaw;

    if (abs(value) > 176.0f) {
        Serial.println("Yaw out of range!");
    }

    // Xử lý wrap-around
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    if (diff < -threshold) {
        Serial.println("Yaw: -1");  // Ngược chiều kim
    } else if (diff > threshold) {
        Serial.println("Yaw: +1");  // Cùng chiều kim
    } else {
        Serial.println("Yaw: 0");   // Trong ngưỡng
    }
}

void LimitedOfRangePitch(float value, float centerPitch, float threshold) {
    if (abs(value) > 86.0f) {
        Serial.println("Pitch out of range!");
    }
    float diff = value - centerPitch;
    if (diff < -threshold) {
        Serial.println("Pitch: -1");  // Cúi xuống
    } else if (diff > threshold) {
        Serial.println("Pitch: +1");  // Ngửa lên
    } else {
        Serial.println("Pitch: 0");   // Ổn định
    }
}

void LimitedOfRangeRoll(float value, float centerRoll, float threshold) {
    if (abs(value) > 176.0f) {
        Serial.println("Roll out of range!");
    }
    if (value < (centerRoll - threshold))
    {
        Serial.println("Roll: quay phai");  // Ngược chiều kim
    } else if (value > (centerRoll + threshold)) {
        Serial.println("Roll: quay trai");  // Cùng chiều kim
    } else {
        Serial.println("Roll: can bang");   // Trong ngưỡng
    }
}

//----------------------------------------------//
//     HIỂN THỊ TRẠNG THÁI HIỆU CHUẨN BNO055    //
//----------------------------------------------//
void PrintCalibrationStatus() {
    uint8_t sys, gyro, accel, mag;
    BNO055.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("Calibration >> SYS:");
    Serial.print(sys);
    Serial.print(" GYRO:");
    Serial.print(gyro);
    Serial.print(" ACC:");
    Serial.print(accel); 
    Serial.print(" MAG:");
    Serial.println(mag);
}
// Biến toàn cục lưu vector hướng chuẩn
float fwd_base[3] = {0.0f, 0.0f, 0.0f};

// Gọi hàm này 1 lần sau khi hệ thống ổn định
void SetBaseForwardVector() {
    imu::Quaternion quat = BNO055.getQuat();
    float x = 2 * (quat.x() * quat.z() + quat.w() * quat.y());
    float y = 2 * (quat.y() * quat.z() - quat.w() * quat.x());
    float z = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());

    fwd_base[0] = x;
    fwd_base[1] = y;
    fwd_base[2] = z;

    Serial.println("Base forward vector set.");
}
// Hàm xác định chiều Pitch hiện tại so với Base_Point
// Trả về -1 (ngửa), +1 (cúi), 0 (không đổi)
int GetPitchDirectionFromQuat() {
    float threshold = 0.05f;
    imu::Quaternion quat = BNO055.getQuat();
    float x = 2 * (quat.x() * quat.z() + quat.w() * quat.y());
    float y = 2 * (quat.y() * quat.z() - quat.w() * quat.x());
    float z = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());

    float deltaY = y - fwd_base[1];

    if (deltaY > threshold) {
        return +1;  // Cúi xuống
    } else if (deltaY < -threshold) {
        return -1;  // Ngửa lên
    } else {
        return 0;   // Không thay đổi đáng kể
    }
}
