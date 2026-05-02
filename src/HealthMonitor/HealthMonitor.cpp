#include "HealthMonitor.hpp"
#include <math.h>

HealthMonitor::HealthMonitor() {
    _imu = {true, 0, 0, 0, 0};
    _bmp = {true, 0, 0, 0, 0};
    _gps = {true, 0, 0, 0, 0};
}

bool HealthMonitor::checkInnovation(float measured, float predicted, float threshold, SensorStatus& status) {
    float innovation = fabsf(measured - predicted);
    bool currentSampleOk = (innovation < threshold);

    if (currentSampleOk) {
        status.consecutiveNormalCount++;
        // 回歸正常判定：殘差縮小到閾值的一半，且維持足夠次數
        if (!status.isOk && innovation < (threshold * 0.5f) && status.consecutiveNormalCount >= RECOVERY_THRESHOLD) {
            status.isOk = true;
        }
    } else {
        status.isOk = false;
        status.consecutiveNormalCount = 0;
    }
    return status.isOk;
}

bool HealthMonitor::checkPhysicalLimits(float value, float min, float max, float maxRate, SensorStatus& status) {
    uint32_t now = esp_timer_get_time();
    float dt = (now - status.lastUpdateTime) / 1000000.0f;
    
    bool outOfBounds = (value < min || value > max);
    bool frozen = (value == status.lastValue);
    bool rateExceeded = (dt > 0) && (fabsf(value - status.lastValue) / dt > maxRate);

    if (frozen) status.frozenCount++;
    else status.frozenCount = 0;

    bool currentSampleOk = !outOfBounds && (status.frozenCount < FROZEN_LIMIT) && !rateExceeded;

    if (currentSampleOk) {
        status.consecutiveNormalCount++;
        if (!status.isOk && status.consecutiveNormalCount >= RECOVERY_THRESHOLD) {
            status.isOk = true;
        }
    } else {
        status.isOk = false;
        status.consecutiveNormalCount = 0;
    }

    status.lastValue = value;
    status.lastUpdateTime = now;
    return status.isOk;
}

bool HealthMonitor::checkConsistency(float valA, float valB, float threshold, SensorStatus& statusA, SensorStatus& statusB) {
    float diff = fabsf(valA - valB);
    bool consistent = (diff < threshold);

    if (!consistent) {
        // 如果兩者不一致，判定較不穩定的那個失效 (這裡簡化處理)
        statusA.consecutiveNormalCount = 0;
        statusB.consecutiveNormalCount = 0;
    }
    return consistent;
}

void HealthMonitor::updateImuHealth(float ax, float ay, float az, float gx, float gy, float gz) {
    // 加速度計檢查：-20G ~ 20G, 最大變化率 500m/s^3
    checkPhysicalLimits(az, -196.0f, 196.0f, 500.0f, _imu);
}

void HealthMonitor::updateBmpHealth(float pressureAlt, float kfAlt) {
    // 方案一：殘差檢查 (閾值 15m)
    checkInnovation(pressureAlt, kfAlt, 15.0f, _bmp);
    // 方案二：物理限制 (高度 -500m ~ 10000m, 爬升率 300m/s)
    checkPhysicalLimits(pressureAlt, -500.0f, 10000.0f, 300.0f, _bmp);
}

void HealthMonitor::updateGpsHealth(float gpsAlt, float kfAlt) {
    // 方案三：跨感測器比對 (GPS 與 Kalman 高度應在 30m 內)
    checkInnovation(gpsAlt, kfAlt, 30.0f, _gps);
}
