#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include "../Communication/InterCoreComm.hpp"
#include "../Kalman/KalmanFilter.hpp"

enum SensorType {
    SENSOR_ICM20948,
    SENSOR_BMP388,
    SENSOR_MAX_M10S
};

class SensorManager {
public:
    struct TaskParams {
        SensorManager* instance;
        SensorType type;
    };

    SensorManager(InterCoreComm* comm, KalmanFilter* kalman); // 透過建構子注入通訊與濾波物件
    void begin();

    void enableIcm();  void disableIcm();
    void enableBmp();  void disableBmp();
    void enableGps();  void disableGps();

    static void taskWrapper(void* pvParameters);

    SemaphoreHandle_t permitIcm;
    SemaphoreHandle_t permitBmp;
    SemaphoreHandle_t permitGps;

private:
    void icm20948Task();
    void bmp388Task();
    void maxM10sTask();

    bool _icmActive;
    bool _bmpActive;
    bool _gpsActive;

    InterCoreComm* _comm;     // 儲存通訊物件指標
    KalmanFilter* _kalman;    // 儲存濾波物件指標
    TaskHandle_t _icmTaskHandle;
    TaskHandle_t _bmpTaskHandle;
    TaskHandle_t _gpsTaskHandle;

    SemaphoreHandle_t _i2cMutex; // I2C 互斥鎖
};

#endif
