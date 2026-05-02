#include "StateMachine.hpp"
#include "../DataManager/DataManager.hpp"
#include <stdio.h>
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"

StateMachine::StateMachine(SensorManager* sensors, RecoveryManager* recovery, InterCoreComm* comm) 
    : _sensors(sensors), _recovery(recovery), _comm(comm), _dataManager(NULL), _taskHandle(NULL), 
      _motorSeparated(false), _startTime(0.0f) {
    permitKalmanFilter = xSemaphoreCreateBinary();
    stateQueue = xQueueCreate(10, sizeof(StateEvent));
    _state = STBY_IDLE;
    _lastPushedState = STBY_IDLE;
    _lastLat = 0; _lastLon = 0;

    nvs_handle_t handle;
    if (nvs_open("sm_storage", NVS_READONLY, &handle) == ESP_OK) {
        uint8_t s;
        if (nvs_get_u8(handle, "state", &s) == ESP_OK) {
            _state = (STATENUM)s;
            _lastPushedState = (STATENUM)s;
        }
        uint8_t m;
        if (nvs_get_u8(handle, "motor", &m) == ESP_OK) _motorSeparated = (m != 0);
        size_t sz = sizeof(double) * 2;
        nvs_get_blob(handle, "pos", &_lastLat, &sz);
        nvs_close(handle);
    }
}

void StateMachine::begin() {
    xTaskCreatePinnedToCore(taskWrapper, "SM_Task", 4096, this, 10, &_taskHandle, 1);
}

void StateMachine::taskWrapper(void* pvParameters) {
    StateMachine* instance = (StateMachine*)pvParameters;
    instance->stateMachineTask();
}

void StateMachine::stateMachineTask() {
    StateEvent event;
    const TickType_t timeoutTicks = pdMS_TO_TICKS(60000);

    while (1) {
        if (xQueueReceive(stateQueue, &event, portMAX_DELAY) == pdTRUE) {
            if (switchValid(event.state)) {
                transitionTo(event.state);
            } else {
                TickType_t now = xTaskGetTickCount();
                if ((now - event.timestamp) <= timeoutTicks) {
                    xQueueSend(stateQueue, &event, 0);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }
    }
}

void StateMachine::pushState(STATENUM newState) {
    // 防止重複推播相同狀態導致 Queue 溢位
    if (stateQueue != NULL && newState != _lastPushedState) {
        StateEvent event = {newState, xTaskGetTickCount()};
        if (xQueueSend(stateQueue, &event, 0) == pdTRUE) {
            _lastPushedState = newState;
        }
    }
}

bool StateMachine::switchValid(STATENUM newState) {
    if (_state == FLIGHT_P12_TERMINATE) return false;
    
    // 飛行狀態差時檢查 (時間保護)
    if (_startTime > 0.0f) {
        float currentTime = esp_timer_get_time() / 1000000.0f;
        float diff = currentTime - _startTime;
        
        // 額外的安全性檢查：確保不會過早進入回收狀態
        if (newState == FLIGHT_P8_9_APOGEE && diff < 10.91f) return false;
        if (newState == FLIGHT_P11_MAIN_CHUTE_DEPLOY && diff < 227.17f) return false;
    }

    if (_state >= FLIGHT_P5_IGNITION && _state < FLIGHT_P12_TERMINATE) {
        if (_state == FLIGHT_P10_DESCENT) {
            return (newState == FLIGHT_P11_MAIN_CHUTE_DEPLOY || newState == FLIGHT_P11_SKIP_MAIN_CHUTE);
        }
        if (_state == FLIGHT_P11_MAIN_CHUTE_DEPLOY || _state == FLIGHT_P11_SKIP_MAIN_CHUTE) {
            return (newState == FLIGHT_P12_TERMINATE);
        }
        // 確保不會跳過狀態或往回跳
        return (newState == (STATENUM)((int)_state + 1));
    }
    return true;
}

void StateMachine::transitionTo(STATENUM newState) {
    char logBuf[64];
    snprintf(logBuf, sizeof(logBuf), "State: %d -> %d", _state, newState);
    _comm->sendLog(logBuf); 

    if (newState == FLIGHT_P6_POWERED) {
        _startTime = esp_timer_get_time() / 1000000.0f;
        _comm->sendLog("EVENT: Powered flight start time recorded");
    }

    _state = newState;
    _lastPushedState = newState; // 同步狀態
    
    switch (newState) {
        case FLIGHT_P8_9_APOGEE:
            _recovery->deployDrogue();
            _comm->sendLog("ACTION: Drogue Deployed");
            break;
        case FLIGHT_P11_MAIN_CHUTE_DEPLOY:
            _motorSeparated = true;
            _recovery->deployMain();
            _comm->sendLog("ACTION: Motor Sep & Main Chute");
            break;
        case FLIGHT_P11_SKIP_MAIN_CHUTE:
            _motorSeparated = false;
            _comm->sendLog("ACTION: Main Chute (No Sep)");
            break;
        case FLIGHT_P12_TERMINATE:
            _comm->sendLog("EVENT: Mission Terminated");
            _recovery->powerOffSystem();
            break;
        default:
            break;
    }

    if (newState >= FLIGHT_P5_IGNITION && newState <= FLIGHT_P12_TERMINATE) {
        _sensors->enableIcm();
        _sensors->enableBmp();
        _sensors->enableGps();
        xSemaphoreGive(permitKalmanFilter);
    }
    saveStateToNVM(newState);
}

void StateMachine::saveStateToNVM(STATENUM state) {
    nvs_handle_t handle;
    if (nvs_open("sm_storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u8(handle, "state", (uint8_t)state);
        nvs_set_u8(handle, "motor", _motorSeparated ? 1 : 0);
        nvs_set_blob(handle, "pos", &_lastLat, sizeof(double) * 2);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void StateMachine::processKalmanForTrigger(float alt, float vz, float az) {
    if (!_dataManager) return;
    HealthMonitor& health = _dataManager->getHealthMonitor();

    // 計算自點火起的飛行時間
    float flightTime = (_startTime > 0) ? (esp_timer_get_time() / 1000000.0f - _startTime) : 0;

    // --- 關鍵修正：加入與 _startTime 差時大於 35 秒的強制頂點觸發 ---
    if (_startTime > 0 && _state < FLIGHT_P8_9_APOGEE && flightTime > TIMEOUT_APOGEE) {
        _comm->sendLog("TRIGGER: Global Time Safety - Forcing Apogee (35s)");
        pushState(FLIGHT_P8_9_APOGEE);
        return; 
    }

    switch (_state) {
        case FLIGHT_P5_IGNITION:
            if (health.isImuHealthy()) {
                if (az > 10.0f) {
                    _comm->sendLog("TRIGGER: Motor Thrust detected (IMU)");
                    pushState(FLIGHT_P6_POWERED);
                }
            } else if (health.isBmpHealthy() && alt > 10.0f) {
                _comm->sendLog("TRIGGER: Altitude gain detected (BMP Backup)");
                pushState(FLIGHT_P6_POWERED);
            }
            break;

        case FLIGHT_P6_POWERED:
            if (health.isImuHealthy()) {
                if (az < 2.0f) {
                    _comm->sendLog("TRIGGER: Burnout detected (IMU)");
                    pushState(FLIGHT_P7_INERTIAL);
                }
            } else if (flightTime > 5.0f) { 
                _comm->sendLog("TRIGGER: Burnout assumed (Time Backup)");
                pushState(FLIGHT_P7_INERTIAL);
            }
            break;

        case FLIGHT_P7_INERTIAL:
            if (vz < -0.2f && (health.isBmpHealthy() || health.isImuHealthy())) {
                _comm->sendLog("TRIGGER: Apogee detected (Sensors)");
                pushState(FLIGHT_P8_9_APOGEE);
            }
            // 這裡的定時觸發已被上方全局檢查涵蓋，但保留作為雙重保險
            break;

        case FLIGHT_P8_9_APOGEE:
            // 進入頂點後，若 1 秒後數據仍顯示下降或時間超過安全閾值，進入下降段
            if (az < 2.0f || flightTime > TIMEOUT_APOGEE + 1.0f) {
                _comm->sendLog("TRIGGER: Descent detected");
                pushState(FLIGHT_P10_DESCENT);
            }
            break;

        case FLIGHT_P10_DESCENT:
            if (alt < 200.0f || flightTime > TIMEOUT_MAIN_CHUTE) {
                if (!health.isGpsHealthy()) {
                    _comm->sendLog("DECISION: GPS Failed, forcing Main Chute (Safety First)");
                    pushState(FLIGHT_P11_MAIN_CHUTE_DEPLOY);
                } else {
                    float dist = calculateDistanceToTarget();
                    if (dist < GPS_THRESHOLD) pushState(FLIGHT_P11_MAIN_CHUTE_DEPLOY);
                    else pushState(FLIGHT_P11_SKIP_MAIN_CHUTE);
                }
            }
            break;

        default:
            break;
    }
}





void StateMachine::processImuForTrigger(float ax, float ay, float az, float gx, float gy, float gz) {
    if (_state == STBY_IDLE && az > 19.6f) {
        _comm->sendLog("TRIGGER: Launch G-force detected");
        pushState(STBY_BIT);
        pushState(FLIGHT_P5_IGNITION);
    }
}

void StateMachine::updateGpsPosition(double lat, double lon) {
    _lastLat = lat; _lastLon = lon;
}

float StateMachine::calculateDistanceToTarget() {
    double dLat = (_lastLat - TARGET_LAT) * 111000.0;
    double dLon = (_lastLon - TARGET_LON) * 111000.0 * cos(TARGET_LAT * M_PI / 180.0);
    return sqrt(dLat * dLat + dLon * dLon);
}

STATENUM StateMachine::getCurrentState() { return _state; }

void StateMachine::processImuForTrigger(float ax, float ay, float az, float gx, float gy, float gz) {
    if (_state == STBY_IDLE && az > 19.6f) {
        _comm->sendLog("TRIGGER: Launch G-force detected");
        pushState(STBY_BIT);
        pushState(FLIGHT_P5_IGNITION);
    }
}
