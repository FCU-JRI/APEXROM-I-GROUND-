#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Communication/InterCoreComm.hpp"
#include "Kalman/KalmanFilter.hpp"
#include "Sensors/Sensors.hpp"
#include "StateMachine/StateMachine.hpp"
#include "Recovery/RecoveryManager.hpp"
#include "DataManager/DataManager.hpp"
#include "StorageComm/StorageCommManager.hpp"
#include "esp_log.h"
#include "nvs_flash.h"

extern "C" void app_main(void) {
    // 1. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("=== Rocket Avionics System Starting ===\n");

    // 2. 建立通訊與濾波物件 (基礎層)
    InterCoreComm* comm = new InterCoreComm();
    KalmanFilter* kalman = new KalmanFilter();
    comm->begin();

    // 3. 建立執行物件 (管理層)
    RecoveryManager* recovery = new RecoveryManager();
    SensorManager* sensors = new SensorManager(comm, kalman);
    StorageCommManager* scm = new StorageCommManager();
    
    // 4. 建立核心邏輯 (決策層)
    StateMachine* sm = new StateMachine(sensors, recovery, comm);
    DataManager* dm = new DataManager(*comm, *sm, *scm);
    
    // 連結決策層與管理層
    sm->setDataManager(dm);
    scm->begin(sm->stateQueue); // 連結 LoRa 接收與狀態機隊列

    // 5. 啟動各任務
    sensors->begin();
    dm->begin();
    sm->begin();

    printf("=== All Systems Operational ===\n");
    
    while(1) {
        vTaskDelay(portMAX_DELAY);
    }
}
