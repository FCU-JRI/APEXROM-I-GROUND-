#include "KalmanFilter.hpp"
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

KalmanFilter::KalmanFilter() {
    _q[0] = 1.0f; _q[1] = 0.0f; _q[2] = 0.0f; _q[3] = 0.0f;
    _bias[0] = 0.0f; _bias[1] = 0.0f; _bias[2] = 0.0f;
    memset(_P, 0, sizeof(_P));
    for (int i = 0; i < 7; i++) _P[i][i] = 0.1f;
    _lastMicros = esp_timer_get_time();
    _lat = 0; _lon = 0; _velN = 0; _velE = 0;
    _x_alt[0] = 0; _x_alt[1] = 0; _x_alt[2] = 0;
    memset(_P_alt, 0, sizeof(_P_alt));
    _P_alt[0][0] = 10.0f; _P_alt[1][1] = 10.0f; _P_alt[2][2] = 0.01f;
    _az_world = 0;
    loadFromNVM();
}

void KalmanFilter::saveToNVM() {
    nvs_handle_t handle;
    if (nvs_open("kf_storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_blob(handle, "q", _q, sizeof(_q));
        nvs_set_blob(handle, "bias", _bias, sizeof(_bias));
        nvs_set_blob(handle, "x_alt", _x_alt, sizeof(_x_alt));
        nvs_set_blob(handle, "lat_lon", &_lat, sizeof(double) * 2);
        nvs_commit(handle); nvs_close(handle);
    }
}

void KalmanFilter::loadFromNVM() {
    nvs_handle_t handle;
    if (nvs_open("kf_storage", NVS_READONLY, &handle) == ESP_OK) {
        size_t sz;
        sz = sizeof(_q); nvs_get_blob(handle, "q", _q, &sz);
        sz = sizeof(_bias); nvs_get_blob(handle, "bias", _bias, &sz);
        sz = sizeof(_x_alt); nvs_get_blob(handle, "x_alt", _x_alt, &sz);
        sz = sizeof(double)*2; nvs_get_blob(handle, "lat_lon", &_lat, &sz);
        nvs_close(handle);
    }
}

void KalmanFilter::updateImu(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    uint32_t now = esp_timer_get_time();
    float dt = (now - _lastMicros) / 1000000.0f;
    _lastMicros = now;
    if (dt <= 0 || dt > 0.5f) dt = 0.01f;

    predict(gx, gy, gz, dt);

    float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];
    _az_world = 2.0f*(qx*qz - qw*qy)*ax + 2.0f*(qy*qz + qw*qx)*ay + (qw*qw - qx*qx - qy*qy + qz*qz)*az - 9.80665f;
    predictAltitude(dt);

    float accMag = sqrtf(ax*ax + ay*ay + az*az);
    if (accMag > 8.0f && accMag < 12.0f) {
        updateAccel(ax, ay, az);
    }
    
    // 磁力計修正 (如果磁力計有數值)
    float magMag = sqrtf(mx*mx + my*my + mz*mz);
    if (magMag > 0.1f) {
        updateMag(mx, my, mz);
    }

    normalizeQuaternion();

    static uint32_t saveCounter = 0;
    if (++saveCounter >= 500) { saveToNVM(); saveCounter = 0; }
}

void KalmanFilter::predict(float gx, float gy, float gz, float dt) {
    float wx = gx - _bias[0], wy = gy - _bias[1], wz = gz - _bias[2];
    float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];
    _q[0] += 0.5f * (-qx * wx - qy * wy - qz * wz) * dt;
    _q[1] += 0.5f * ( qw * wx + qy * wz - qz * wy) * dt;
    _q[2] += 0.5f * ( qw * wy - qx * wz + qz * wx) * dt;
    _q[3] += 0.5f * ( qw * wz + qx * wy - qy * wx) * dt;
    for (int i = 0; i < 7; i++) _P[i][i] += (i < 4 ? Q_gyro : Q_bias) * dt;
}

void KalmanFilter::updateAccel(float ax, float ay, float az) {
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (mag < 0.01f) return;
    ax /= mag; ay /= mag; az /= mag;
    float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];
    float vx = 2.0f * (qx*qz - qw*qy), vy = 2.0f * (qw*qx + qy*qz), vz = qw*qw - qx*qx - qy*qy + qz*qz;
    float ex = (ay*vz - az*vy), ey = (az*vx - ax*vz), ez = (ax*vy - ay*vx);
    float K = 0.5f;
    _q[0] += 0.5f * (-qx*ex - qy*ey - qz*ez) * K;
    _q[1] += 0.5f * ( qw*ex + qy*ez - qz*ey) * K;
    _q[2] += 0.5f * ( qw*ey - qx*ez + qz*ex) * K;
    _q[3] += 0.5f * ( qw*ez + qx*ey - qy*ex) * K;
}

void KalmanFilter::updateMag(float mx, float my, float mz) {
    // 磁力計歸一化
    float mag = sqrtf(mx*mx + my*my + mz*mz);
    if (mag < 0.01f) return;
    mx /= mag; my /= mag; mz /= mag;

    // 將磁力計投影到水平面以計算 Yaw 誤差
    float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];
    
    // 計算目前的 Heading (從四元數)
    float hx = mx * (qw*qw + qx*qx - qy*qy - qz*qz) + my * (2.0f*(qx*qy - qw*qz)) + mz * (2.0f*(qx*qz + qw*qy));
    float hy = mx * (2.0f*(qx*qy + qw*qz)) + my * (qw*qw - qx*qx + qy*qy - qz*qz) + mz * (2.0f*(qy*qz - qw*qx));
    
    // 這裡我們假設目標 Heading (磁北) 應該在投影後的 X 軸
    // 誤差角 error_z = atan2(hy, hx)
    float ez = atan2f(hy, hx);
    
    // 修正四元數 (繞 Z 軸旋轉 -ez)
    float K_mag = 0.2f; // 磁力計修正強度
    float half_ez = ez * 0.5f * K_mag;
    float dq[4];
    dq[0] = cosf(half_ez); dq[1] = 0; dq[2] = 0; dq[3] = -sinf(half_ez);

    // 四元數乘法：_q = _q * dq
    float nq[4];
    nq[0] = _q[0]*dq[0] - _q[3]*dq[3];
    nq[1] = _q[1]*dq[0] + _q[2]*dq[3];
    nq[2] = _q[2]*dq[0] - _q[1]*dq[3];
    nq[3] = _q[3]*dq[0] + _q[0]*dq[3];
    memcpy(_q, nq, sizeof(_q));
}

void KalmanFilter::predictAltitude(float dt) {
    float accel = _az_world - _x_alt[2];
    _x_alt[0] += _x_alt[1] * dt + 0.5f * accel * dt * dt;
    _x_alt[1] += accel * dt;
    _P_alt[0][0] += _P_alt[1][1] * dt * dt + 0.001f; 
    _P_alt[1][1] += _P_alt[2][2] * dt * dt + 0.01f;
    _P_alt[2][2] += 0.0001f;
}

void KalmanFilter::updateBmp(float pressure, float temperature) {
    float baroAlt = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));
    float y = baroAlt - _x_alt[0];
    float S = _P_alt[0][0] + 2.0f;
    float K[3] = {_P_alt[0][0]/S, _P_alt[1][0]/S, _P_alt[2][0]/S};
    _x_alt[0] += K[0] * y; _x_alt[1] += K[1] * y; _x_alt[2] += K[2] * y;
    _P_alt[0][0] -= K[0] * _P_alt[0][0]; _P_alt[1][1] -= K[1] * _P_alt[0][1];
}

void KalmanFilter::updateGps(double lat, double lon, float alt) {
    _lat = lat; _lon = lon;
    float y = alt - _x_alt[0];
    float S = _P_alt[0][0] + 10.0f;
    float K[3] = {_P_alt[0][0]/S, _P_alt[1][0]/S, _P_alt[2][0]/S};
    _x_alt[0] += K[0] * y; _x_alt[1] += K[1] * y; _x_alt[2] += K[2] * y;
}

void KalmanFilter::getQuaternion(float q[4]) { memcpy(q, _q, sizeof(float) * 4); }
void KalmanFilter::getGps(double& lat, double& lon, float& vN, float& vE) { lat=_lat; lon=_lon; vN=_velN; vE=_velE; }
void KalmanFilter::getAltitude(float& alt, float& vz, float& az) { alt=_x_alt[0]; vz=_x_alt[1]; az=_az_world; }
void KalmanFilter::normalizeQuaternion() {
    float mag = sqrtf(_q[0]*_q[0] + _q[1]*_q[1] + _q[2]*_q[2] + _q[3]*_q[3]);
    _q[0] /= mag; _q[1] /= mag; _q[2] /= mag; _q[3] /= mag;
}
