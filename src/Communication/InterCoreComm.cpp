#include "InterCoreComm.hpp"
#include <string.h>

InterCoreComm::InterCoreComm() : _rawRingBuf(NULL), _kalmanRingBuf(NULL) {}

bool InterCoreComm::begin() {
    _rawRingBuf = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
    _kalmanRingBuf = xRingbufferCreate(2048, RINGBUF_TYPE_NOSPLIT);
    return (_rawRingBuf != NULL && _kalmanRingBuf != NULL);
}

bool InterCoreComm::sendRawImu(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    struct { SensorPacketHeader h; ImuData d; } pkt;
    pkt.h = {DATA_TYPE_IMU, xTaskGetTickCount()};
    pkt.d = {ax, ay, az, gx, gy, gz, mx, my, mz};
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendRawBmp(float pressure, float temp) {
    struct { SensorPacketHeader h; BmpData d; } pkt;
    pkt.h = {DATA_TYPE_BMP, xTaskGetTickCount()};
    pkt.d = {pressure, temp};
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendRawGps(double lat, double lon, float alt) {
    struct { SensorPacketHeader h; GpsData d; } pkt;
    pkt.h = {DATA_TYPE_GPS, xTaskGetTickCount()};
    pkt.d = {lat, lon, alt};
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendLog(const char* msg) {
    struct { SensorPacketHeader h; LogData d; } pkt;
    pkt.h = {DATA_TYPE_LOG, xTaskGetTickCount()};
    strncpy(pkt.d.message, msg, sizeof(pkt.d.message) - 1);
    pkt.d.message[sizeof(pkt.d.message) - 1] = '\0';
    return xRingbufferSend(_rawRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendKalmanQuaternion(const float q[4]) {
    struct { KalmanPacketHeader h; KalmanQuaternionData d; } pkt;
    pkt.h = {KALMAN_TYPE_QUATERNION, xTaskGetTickCount()};
    memcpy(pkt.d.q, q, sizeof(float)*4);
    return xRingbufferSend(_kalmanRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendKalmanGps(double lat, double lon, float velN, float velE) {
    struct { KalmanPacketHeader h; KalmanGpsData d; } pkt;
    pkt.h = {KALMAN_TYPE_GPS, xTaskGetTickCount()};
    pkt.d = {lat, lon, velN, velE};
    return xRingbufferSend(_kalmanRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

bool InterCoreComm::sendKalmanAltitude(float alt, float vz, float az) {
    struct { KalmanPacketHeader h; KalmanAltitudeData d; } pkt;
    pkt.h = {KALMAN_TYPE_ALTITUDE, xTaskGetTickCount()};
    pkt.d = {alt, vz, az};
    return xRingbufferSend(_kalmanRingBuf, &pkt, sizeof(pkt), 0) == pdTRUE;
}

void* InterCoreComm::receiveRawData(size_t* size, TickType_t waitTicks) {
    return xRingbufferReceive(_rawRingBuf, size, waitTicks);
}

void InterCoreComm::returnRawData(void* item) {
    vRingbufferReturnItem(_rawRingBuf, item);
}

void* InterCoreComm::receiveKalmanData(size_t* size, TickType_t waitTicks) {
    return xRingbufferReceive(_kalmanRingBuf, size, waitTicks);
}

void InterCoreComm::returnKalmanData(void* item) {
    vRingbufferReturnItem(_kalmanRingBuf, item);
}
