#include "DataManager.hpp"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "DataManager";

DataManager::DataManager(InterCoreComm& comm, StateMachine& sm, StorageCommManager& scm) 
    : _comm(comm), _sm(sm), _scm(scm), _rawTaskHandle(NULL), _kalmanTaskHandle(NULL), 
      _currentKfAlt(0.0f), _len256(0) {
    
    _lock256 = xSemaphoreCreateMutex();
    memset(_buffer256, 0, sizeof(_buffer256));
}

bool DataManager::begin() {
    xTaskCreatePinnedToCore(rawDataTask, "RawDataTask", 4096, this, 5, &_rawTaskHandle, 1);
    xTaskCreatePinnedToCore(kalmanDataTask, "KalmanDataTask", 4096, this, 5, &_kalmanTaskHandle, 1);
    return (_rawTaskHandle != NULL && _kalmanTaskHandle != NULL);
}

void DataManager::updateBuffers(void* data, size_t size) {
    // --- 單一 256Byte 緩衝區 (存儲與地面傳輸累計) ---
    if (size <= 256) {
        if (xSemaphoreTake(_lock256, pdMS_TO_TICKS(10)) == pdTRUE) {
            // 檢查剩餘空間是否足夠
            if (_len256 + size > 256) {
                // 空間不足：存入儲存裝置 並 透過 LoRa 傳輸至地面端
                _scm.saveToStorage(_buffer256, _len256);
                _scm.sendViaRadio(_buffer256, _len256);
                _len256 = 0;
            }
            
            // 累計寫入
            memcpy(_buffer256 + _len256, data, size);
            _len256 += size;

            // 若存滿至 256 bytes (或超過預設閾值) 則主動發送並儲存
            if (_len256 >= 256) {
                _scm.saveToStorage(_buffer256, _len256);
                _scm.sendViaRadio(_buffer256, _len256);
                _len256 = 0;
            }
            xSemaphoreGive(_lock256);
        }
    }
}

void DataManager::rawDataTask(void* pvParameters) {
    DataManager* manager = static_cast<DataManager*>(pvParameters);
    manager->processRawData();
}

void DataManager::kalmanDataTask(void* pvParameters) {
    DataManager* manager = static_cast<DataManager*>(pvParameters);
    manager->processKalmanData();
}

void DataManager::processRawData() {
    size_t size;
    while (true) {
        void* item = _comm.receiveRawData(&size, portMAX_DELAY);
        if (item != NULL) {
            updateBuffers(item, size);

            SensorPacketHeader* header = (SensorPacketHeader*)item;
            switch (header->type) {
                case DATA_TYPE_IMU: {
                    ImuData* data = (ImuData*)((uint8_t*)item + sizeof(SensorPacketHeader));
                    // 健康檢查
                    _health.updateImuHealth(data->ax, data->ay, data->az, data->gx, data->gy, data->gz);
                    _sm.processImuForTrigger(data->ax, data->ay, data->az, data->gx, data->gy, data->gz);
                    break;
                }
                case DATA_TYPE_BMP: {
                    BmpData* data = (BmpData*)((uint8_t*)item + sizeof(SensorPacketHeader));
                    // 健康檢查：利用當前 Kalman 高度進行 Innovation Check
                    float pressureAlt = 44330.0f * (1.0f - powf((data->pressure * 100.0f) / 101325.0f, 0.1903f));
                    _health.updateBmpHealth(pressureAlt, _currentKfAlt);
                    break;
                }
                case DATA_TYPE_GPS: {
                    GpsData* data = (GpsData*)((uint8_t*)item + sizeof(SensorPacketHeader));
                    // 健康檢查
                    _health.updateGpsHealth(data->alt, _currentKfAlt);
                    break;
                }
                case DATA_TYPE_LOG: {
                    LogData* data = (LogData*)((uint8_t*)item + sizeof(SensorPacketHeader));
                    ESP_LOGI(TAG, "Log Record: %s", data->message);
                    break;
                }
                default: break;
            }
            _comm.returnRawData(item);
        }
    }
}

void DataManager::processKalmanData() {
    size_t size;
    while (true) {
        void* item = _comm.receiveKalmanData(&size, portMAX_DELAY);
        if (item != NULL) {
            updateBuffers(item, size);

            KalmanPacketHeader* header = (KalmanPacketHeader*)item;
            switch (header->type) {
                case KALMAN_TYPE_GPS: {
                    KalmanGpsData* data = (KalmanGpsData*)((uint8_t*)item + sizeof(KalmanPacketHeader));
                    _sm.updateGpsPosition(data->lat, data->lon);
                    break;
                }
                case KALMAN_TYPE_ALTITUDE: {
                    KalmanAltitudeData* data = (KalmanAltitudeData*)((uint8_t*)item + sizeof(KalmanPacketHeader));
                    _currentKfAlt = data->alt; // 更新當前 Kalman 高度
                    _sm.processKalmanForTrigger(data->alt, data->vz, data->az);
                    break;
                }
                default: break;
            }
            _comm.returnKalmanData(item);
        }
    }
}

void DataManager::getBuffer256(uint8_t* out, size_t* len) {
    if (xSemaphoreTake(_lock256, portMAX_DELAY) == pdTRUE) {
        memcpy(out, _buffer256, _len256);
        *len = _len256;
        xSemaphoreGive(_lock256);
    }
}
