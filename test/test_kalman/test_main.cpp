#include <unity.h>
#include "KalmanFilter.hpp"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// 輔助函式
void assert_quaternion_normalized(float q[4]) {
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, norm);
}

void setUp(void) {
    // 每個測試前初始化 NVS，確保 NVM 測試環境乾淨
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

void tearDown(void) {
    nvs_flash_deinit();
}

// 1. 基本姿態測試
void test_kalman_imu_attitude(void) {
    KalmanFilter kf;
    float q[4];
    
    // 模擬靜止 1 秒 (100Hz)
    for(int i = 0; i < 100; i++) {
        kf.updateImu(0, 0, 9.81f, 0, 0, 0, 1.0f, 0, 0); // 磁北在 X 軸
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    kf.getQuaternion(q);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, q[0]); // qw 應接近 1
    assert_quaternion_normalized(q);
}

// 2. 高度與速度融合測試
void test_kalman_altitude_fusion(void) {
    KalmanFilter kf;
    float alt, vz, az_w;

    // 初始狀態應為 0
    kf.getAltitude(alt, vz, az_w);
    TEST_ASSERT_EQUAL_FLOAT(0, alt);

    // 模擬 1 秒的穩定上升 (氣壓計顯示 100m)
    for(int i = 0; i < 50; i++) {
        kf.updateBmp(89800.0f, 25.0f); // 模擬氣壓對應約 1000m 高度
        kf.updateImu(0, 0, 9.81f, 0, 0, 0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    kf.getAltitude(alt, vz, az_w);
    // 高度應收斂至正值且不為 0
    TEST_ASSERT_GREATER_THAN(100.0f, alt);
    // 因為高度在增加，垂直速度 vz 應大於 0
    TEST_ASSERT_GREATER_THAN(0.0f, vz);
}

// 3. 磁力計偏航修正測試
void test_kalman_mag_yaw_correction(void) {
    KalmanFilter kf;
    float q_start[4], q_end[4];

    // 讓濾波器先穩定
    for(int i = 0; i < 20; i++) kf.updateImu(0, 0, 9.8f, 0, 0, 0, 1, 0, 0);
    kf.getQuaternion(q_start);

    // 模擬磁場突然旋轉 90 度 (磁北移到 Y 軸)
    // 濾波器應該會修正四元數以對準新的磁北
    for(int i = 0; i < 50; i++) {
        kf.updateImu(0, 0, 9.8f, 0, 0, 0, 0, 1.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    kf.getQuaternion(q_end);

    // qz (旋轉軸) 應該發生變化
    TEST_ASSERT_NOT_EQUAL(q_start[3], q_end[3]);
}

// 4. NVM 儲存與恢復測試
void test_kalman_nvm_persistence(void) {
    {
        KalmanFilter kf_save;
        // 模擬一些數據讓狀態偏離初始值
        kf_save.updateGps(25.1234, 121.5678, 100.0f);
        kf_save.saveToNVM();
    } // kf_save 對象銷毀

    // 建立新對象，應自動從 NVM 讀回
    KalmanFilter kf_load;
    double lat, lon;
    float vn, ve;
    kf_load.getGps(lat, lon, vn, ve);

    TEST_ASSERT_EQUAL_DOUBLE(25.1234, lat);
    TEST_ASSERT_EQUAL_DOUBLE(121.5678, lon);
}

extern "C" void app_main() {
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    UNITY_BEGIN();
    RUN_TEST(test_kalman_imu_attitude);
    RUN_TEST(test_kalman_altitude_fusion);
    RUN_TEST(test_kalman_mag_yaw_correction);
    RUN_TEST(test_kalman_nvm_persistence);
    UNITY_END();
}
