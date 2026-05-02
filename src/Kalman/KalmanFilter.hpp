#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <stdio.h>
#include <stdint.h>

class KalmanFilter {
public:
    KalmanFilter();

    // 更新介面
    void updateImu(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void updateBmp(float pressure, float temperature);
    void updateGps(double lat, double lon, float alt);

    // 取得目前濾波結果
    void getQuaternion(float q[4]);
    void getGps(double& lat, double& lon, float& velN, float& velE);
    void getAltitude(float& alt, float& vz, float& az);

    // NVM 儲存與讀取
    void saveToNVM();
    void loadFromNVM();

private:
    // --- 姿態 EKF 狀態 ---
    float _q[4];           // 狀態：四元數 [w, x, y, z]
    float _bias[3];        // 狀態：陀螺儀偏差 [x, y, z]
    float _P[7][7];        // 狀態協方差矩陣
    
    // 噪聲參數
    const float Q_gyro = 0.001f;
    const float Q_bias = 0.00001f;
    const float R_accel = 0.01f;
    const float R_mag = 0.1f; // 磁力計雜訊

    uint32_t _lastMicros;

    // --- 高度/GPS 狀態 ---
    double _lat, _lon;
    float _velN, _velE;
    
    float _x_alt[3]; 
    float _P_alt[3][3];
    float _az_world;

    // 內部方法
    void predict(float gx, float gy, float gz, float dt);
    void predictAltitude(float dt);
    void updateAccel(float ax, float ay, float az);
    void updateMag(float mx, float my, float mz); // 新增
    void normalizeQuaternion();
};

#endif
