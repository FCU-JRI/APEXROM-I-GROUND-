#ifndef INTER_CORE_COMM_HPP
#define INTER_CORE_COMM_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include <stdio.h>
#include <string.h>

// 1. 定義資料型別
enum SensorDataType : uint8_t {
    DATA_TYPE_IMU,
    DATA_TYPE_BMP,
    DATA_TYPE_GPS,
    DATA_TYPE_LOG
};

// 2. 定義共用標頭
struct SensorPacketHeader {
    SensorDataType type;
    TickType_t timestamp;
};

// 3. 定義不同的資料結構
struct ImuData {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz; // 加入磁力計數據
};

struct BmpData {
    float pressure, temperature;
};

struct GpsData {
    double lat, lon;
    float alt;
};

struct LogData {
    char message[64];
};

// 4. Kalman 資料型別與標頭
enum KalmanDataType : uint8_t {
    KALMAN_TYPE_QUATERNION,
    KALMAN_TYPE_GPS,
    KALMAN_TYPE_ALTITUDE
};

struct KalmanPacketHeader {
    KalmanDataType type;
    TickType_t timestamp;
};

struct KalmanQuaternionData { float q[4]; };
struct KalmanGpsData { double lat, lon; float velN, velE; };
struct KalmanAltitudeData { float alt, vz, az; };

class InterCoreComm {
public:
    InterCoreComm();
    bool begin();

    bool sendRawImu(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    bool sendRawBmp(float pressure, float temp);
    bool sendRawGps(double lat, double lon, float alt);
    bool sendLog(const char* msg);

    bool sendKalmanQuaternion(const float q[4]);
    bool sendKalmanGps(double lat, double lon, float velN, float velE);
    bool sendKalmanAltitude(float alt, float vz, float az);

    void* receiveRawData(size_t* size, TickType_t waitTicks);
    void returnRawData(void* item);

    void* receiveKalmanData(size_t* size, TickType_t waitTicks);
    void returnKalmanData(void* item);

private:
    RingbufHandle_t _rawRingBuf;
    RingbufHandle_t _kalmanRingBuf;
};

#endif
