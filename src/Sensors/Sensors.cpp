#include "Sensors.hpp"
#include <sys/time.h>
#include <time.h>

SensorManager::SensorManager(InterCoreComm* comm, KalmanFilter* kalman) : 
    _icmActive(false), _bmpActive(false), _gpsActive(false),
    _comm(comm), _kalman(kalman),
    _icmTaskHandle(NULL), _bmpTaskHandle(NULL), _gpsTaskHandle(NULL) {
    
    permitIcm = xSemaphoreCreateBinary();
    permitBmp = xSemaphoreCreateBinary();
    permitGps = xSemaphoreCreateBinary();
    _i2cMutex = xSemaphoreCreateMutex();
}

void SensorManager::begin() {
    static SensorManager::TaskParams icmParams = {this, SENSOR_ICM20948};
    static SensorManager::TaskParams bmpParams = {this, SENSOR_BMP388};
    static SensorManager::TaskParams gpsParams = {this, SENSOR_MAX_M10S};

    xTaskCreatePinnedToCore(taskWrapper, "ICM_Task", 4096, &icmParams, 5, &_icmTaskHandle, 0);
    xTaskCreatePinnedToCore(taskWrapper, "BMP_Task", 4096, &bmpParams, 5, &_bmpTaskHandle, 0);
    xTaskCreatePinnedToCore(taskWrapper, "GPS_Task", 4096, &gpsParams, 5, &_gpsTaskHandle, 0);
}

void SensorManager::enableIcm() { _icmActive = true; xSemaphoreGive(permitIcm); }
void SensorManager::enableBmp() { _bmpActive = true; xSemaphoreGive(permitBmp); }
void SensorManager::enableGps() { _gpsActive = true; xSemaphoreGive(permitGps); }

void SensorManager::disableIcm() { _icmActive = false; }
void SensorManager::disableBmp() { _bmpActive = false; }
void SensorManager::disableGps() { _gpsActive = false; }

void SensorManager::taskWrapper(void* pvParameters) {
    SensorManager::TaskParams* params = (SensorManager::TaskParams*)pvParameters;
    SensorManager* instance = params->instance;
    switch (params->type) {
        case SENSOR_ICM20948: instance->icm20948Task(); break;
        case SENSOR_BMP388:   instance->bmp388Task();   break;
        case SENSOR_MAX_M10S: instance->maxM10sTask(); break;
    }
}

void SensorManager::icm20948Task() {
    while (1) {
        if (!_icmActive) {
            xSemaphoreTake(permitIcm, portMAX_DELAY);
        }
        
        // 獲取 I2C 鎖
        if (xSemaphoreTake(_i2cMutex, portMAX_DELAY) == pdTRUE) {
            // 模擬獲取資料並傳送至 Ring Buffer
            float ax=0, ay=0, az=9.8, gx=0, gy=0, gz=0;
            float mx=1.0f, my=0.0f, mz=0.0f; // 給予模擬磁力計數值，避免被 Kalman 過濾
            if (_comm) {
                _comm->sendRawImu(ax, ay, az, gx, gy, gz, mx, my, mz);
            }

            // --- Kalman 濾波計算 ---
            if (_kalman && _comm) {
                _kalman->updateImu(ax, ay, az, gx, gy, gz, mx, my, mz);
                float q[4];
                _kalman->getQuaternion(q);
                _comm->sendKalmanQuaternion(q);
            }

            // 釋放 I2C 鎖
            xSemaphoreGive(_i2cMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void SensorManager::bmp388Task() {
    while (1) {
        if (!_bmpActive) {
            xSemaphoreTake(permitBmp, portMAX_DELAY);
        }
        
        // 獲取 I2C 鎖
        if (xSemaphoreTake(_i2cMutex, portMAX_DELAY) == pdTRUE) {
            float pressure=1013.25, temp=25.0;
            if (_comm) {
                _comm->sendRawBmp(pressure, temp);
            }

            // --- Kalman 濾波計算 ---
            if (_kalman && _comm) {
                _kalman->updateBmp(pressure, temp);
                float alt, vz, az;
                _kalman->getAltitude(alt, vz, az);
                _comm->sendKalmanAltitude(alt, vz, az);
            }

            // 釋放 I2C 鎖
            xSemaphoreGive(_i2cMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void SensorManager::maxM10sTask() {
    while (1) {
        if (!_gpsActive) {
            xSemaphoreTake(permitGps, portMAX_DELAY);
        }
        
        double lat=25.03, lon=121.56; float alt=500.0;
        
        // --- GPS 時間校正功能 ---
        // 模擬獲取 GPS UTC 時間 (2026-05-01 12:00:00)
        struct tm t;
        t.tm_year = 2026 - 1900; // 年份自 1900 起算
        t.tm_mon = 5 - 1;        // 月份 0-11
        t.tm_mday = 1;
        t.tm_hour = 12;
        t.tm_min = 0;
        t.tm_sec = 0;
        
        time_t now = mktime(&t);
        struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        
        static bool timeSynced = false;
        if (!timeSynced) {
            printf("[GPS] System time synchronized to GPS UTC: 2026-05-01 12:00:00\n");
            timeSynced = true;
        }

        if (_comm) {
            _comm->sendRawGps(lat, lon, alt);
        }

        // --- Kalman 濾波計算 ---
        if (_kalman && _comm) {
            _kalman->updateGps(lat, lon, alt);
            double kLat, kLon; float vN, vE;
            _kalman->getGps(kLat, kLon, vN, vE);
            _comm->sendKalmanGps(kLat, kLon, vN, vE);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // GPS 通常為 1Hz 更新
    }
}
