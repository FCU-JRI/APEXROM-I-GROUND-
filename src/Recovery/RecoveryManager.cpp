#include "RecoveryManager.hpp"

static const char* TAG = "Recovery";

RecoveryManager::RecoveryManager() {}

void RecoveryManager::begin() {
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_27) | (1ULL << PIN_14) | (1ULL << PIN_5);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    // 初始設為 0, 0, 0 (或根據您的硬體預設值)
    applyLogic(0, 0, 0);
}

void RecoveryManager::deployDrogue()   { applyLogic(1, 1, 1); }
void RecoveryManager::deployMain() { applyLogic(0, 1, 0); }
void RecoveryManager::powerOffSystem() { applyLogic(0, 1, 1); }

void RecoveryManager::applyLogic(int io27, int io14, int io5) {
    ESP_LOGI(TAG, "Logic Apply -> IO27:%d, IO14:%d, IO5:%d", io27, io14, io5);
    gpio_set_level(PIN_27, io27);
    gpio_set_level(PIN_14, io14);
    gpio_set_level(PIN_5, io5);
}
