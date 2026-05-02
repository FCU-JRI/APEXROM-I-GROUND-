#include "StorageCommManager.hpp"
#include "esp_log.h"
#include "freertos/task.h"

// 注意：我們不需要 include StateMachine.hpp，只需要知道 StateEvent 的結構
// 為了避免重複定義，我們可以使用 extern 或直接在 .cpp 裡定義一個相容的指令格式
struct RawStateEvent {
    uint8_t state;
    TickType_t timestamp;
};

static const char* TAG = "StorageComm";

StorageCommManager::StorageCommManager() : _smQueue(NULL), _rxTaskHandle(NULL) {
    _radioMutex = xSemaphoreCreateMutex();
}

void StorageCommManager::begin(QueueHandle_t smQueue) {
    _smQueue = smQueue;
    
    // 建立無線電接收任務 (Core 1)
    xTaskCreatePinnedToCore(radioRxTask, "RadioRxTask", 4096, this, 6, &_rxTaskHandle, 1);
    ESP_LOGI(TAG, "StorageCommManager Initialized with RX Task");
}

void StorageCommManager::radioRxTask(void* pvParameters) {
    StorageCommManager* instance = static_cast<StorageCommManager*>(pvParameters);
    instance->processRxLogic();
}

void StorageCommManager::processRxLogic() {
    while (true) {
        // 進入無線電操作臨界區
        if (xSemaphoreTake(_radioMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 1. 模擬接收無線電硬體資料 (例如 UART 或 SPI)
            // char cmd = wait_for_radio_data();
            
            // 2. 假設收到了「強制切換至狀態 12」的指令
            bool forceTerminateReceived = false; // 這裡改為您的硬體判斷

            if (forceTerminateReceived && _smQueue != NULL) {
                ESP_LOGW(TAG, "Remote Command: Force Terminate Mission!");
                RawStateEvent evt = {12, xTaskGetTickCount()}; // 12 = FLIGHT_P12_TERMINATE
                xQueueSend(_smQueue, &evt, 0);
            }
            
            xSemaphoreGive(_radioMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 接收任務的輪詢頻率
    }
}

void StorageCommManager::sendViaRadio(const uint8_t* data, size_t len) {
    // 傳輸時獲取鎖，確保接收任務在此期間等待
    if (xSemaphoreTake(_radioMutex, portMAX_DELAY) == pdTRUE) {
        ESP_LOGD(TAG, "Radio TX Start: Sending %d bytes...", len);
        
        // 模擬硬體傳輸延遲
        // hardware_send(data, len);
        
        ESP_LOGD(TAG, "Radio TX Finished.");
        xSemaphoreGive(_radioMutex);
    }
}

void StorageCommManager::saveToStorage(const uint8_t* data, size_t len) {
    ESP_LOGD(TAG, "Storage: Saving %d bytes...", len);
}
