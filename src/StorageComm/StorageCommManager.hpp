#ifndef STORAGE_COMM_MANAGER_HPP
#define STORAGE_COMM_MANAGER_HPP

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

class StorageCommManager {
public:
    StorageCommManager();
    void begin(QueueHandle_t smQueue); // 傳入狀態機的 Queue

    // 發送介面
    void sendViaRadio(const uint8_t* data, size_t len);
    void saveToStorage(const uint8_t* data, size_t len);

private:
    QueueHandle_t _smQueue;
    TaskHandle_t _rxTaskHandle;
    SemaphoreHandle_t _radioMutex; // LoRa 傳輸/接收 鎖

    static void radioRxTask(void* pvParameters);
    void processRxLogic();
};

#endif
