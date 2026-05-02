#ifndef RECOVERY_MANAGER_HPP
#define RECOVERY_MANAGER_HPP

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"


class RecoveryManager {
public:
    RecoveryManager();
    void begin();

    // 依照邏輯表格設定 IO 狀態
    void deployDrogue();        // 副傘部署 (P8): 1, 1, 1
    void deployMain();      // 主傘部署 (P11): 0, 1, 0
    void powerOffSystem();      // 斷電系統 (P12前10s): 0, 1, 1

private:
    const gpio_num_t PIN_27 = GPIO_NUM_27;
    const gpio_num_t PIN_14 = GPIO_NUM_14;
    const gpio_num_t PIN_5  = GPIO_NUM_5;

    void applyLogic(int io27, int io14, int io5);
};

#endif
