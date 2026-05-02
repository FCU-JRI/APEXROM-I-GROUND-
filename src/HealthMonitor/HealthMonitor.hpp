#ifndef HEALTH_MONITOR_HPP
#define HEALTH_MONITOR_HPP

#include <stdint.h>
#include <stdbool.h>
#include "esp_timer.h"

struct SensorStatus {
    bool isOk;
    float lastValue;
    uint32_t lastUpdateTime;
    uint16_t consecutiveNormalCount;
    uint16_t frozenCount;
};

class HealthMonitor {
public:
    HealthMonitor();

    // 方案一：Kalman Innovation Check (殘差檢查)
    bool checkInnovation(float measured, float predicted, float threshold, SensorStatus& status);

    // 方案二：Physical Limits & Frozen Check (物理極限與凍結檢查)
    bool checkPhysicalLimits(float value, float min, float max, float maxRate, SensorStatus& status);

    // 方案三：Cross-Sensor Voting (跨感測器比對)
    bool checkConsistency(float valA, float valB, float threshold, SensorStatus& statusA, SensorStatus& statusB);

    // 取得感測器整體健康狀態
    bool isImuHealthy() { return _imu.isOk; }
    bool isBmpHealthy() { return _bmp.isOk; }
    bool isGpsHealthy() { return _gps.isOk; }

    // 更新各感測器狀態
    void updateImuHealth(float ax, float ay, float az, float gx, float gy, float gz);
    void updateBmpHealth(float pressureAlt, float kfAlt);
    void updateGpsHealth(float gpsAlt, float kfAlt);

private:
    SensorStatus _imu, _bmp, _gps;
    const uint16_t RECOVERY_THRESHOLD = 20; // 需要連續 20 次正常才算回歸
    const uint16_t FROZEN_LIMIT = 15;       // 連續 15 次數值完全相同判定為凍結
};

#endif
