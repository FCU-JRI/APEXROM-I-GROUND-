#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "../Sensors/Sensors.hpp"
#include "../Recovery/RecoveryManager.hpp"
#include "../Communication/InterCoreComm.hpp"

class DataManager; // 前向宣告

enum STATENUM {
    STBY_IDLE ,    
    STBY_BIT  ,           

    CAL_GYRO  ,            
    CAL_ACCEL ,            
    CAL_MAG   ,            
    CAL_BARO  ,            
    CAL_TEMP  ,            

    FLIGHT_P5_IGNITION   , 
    FLIGHT_P6_POWERED    , 
    FLIGHT_P7_INERTIAL   , 
    FLIGHT_P8_9_APOGEE   , 
    FLIGHT_P10_DESCENT   , 
    FLIGHT_P11_MAIN_CHUTE_DEPLOY, // 分離發動機並開大傘
    FLIGHT_P11_SKIP_MAIN_CHUTE,   // 跳過主傘 (GPS位置不允許)
    FLIGHT_P12_TERMINATE  , 

    DBG_COMM    ,           
    DBG_SENSOR  ,           
    DBG_STORAGE ,           
};

class StateMachine {
public:
    struct StateEvent {
        STATENUM state;
        TickType_t timestamp;
    };

    StateMachine(SensorManager* sensors, RecoveryManager* recovery, InterCoreComm* comm);
    void setDataManager(DataManager* dm) { _dataManager = dm; }
    void begin();
    
    void transitionTo(STATENUM newState);
    void pushState(STATENUM newState);
    void processImuForTrigger(float ax, float ay, float az, float gx, float gy, float gz); 
    void processKalmanForTrigger(float alt, float vz, float az);
    void updateGpsPosition(double lat, double lon);
    STATENUM getCurrentState();

    static void taskWrapper(void* pvParameters);
    
    SemaphoreHandle_t permitKalmanFilter;
    QueueHandle_t stateQueue;

private:
    void stateMachineTask();
    bool switchValid(STATENUM newState);
    float calculateDistanceToTarget();

    STATENUM _state;
    STATENUM _lastPushedState; // 防止重複推播相同狀態到 Queue
    SensorManager* _sensors;
    RecoveryManager* _recovery;
    InterCoreComm* _comm;
    DataManager* _dataManager; // 新增 DataManager 指標
    TaskHandle_t _taskHandle;

    float _startTime; // 紀錄動力上升開始時間 (秒)
    bool _motorSeparated; 
    double _lastLat, _lastLon;
    const double TARGET_LAT = 25.0330; 
    const double TARGET_LON = 121.5654;
    const float GPS_THRESHOLD = 500.0f;

    // 安全定時防線 (最後手段)
    const float TIMEOUT_APOGEE = 35.0f;     // 點火後 35 秒強制頂點
    const float TIMEOUT_MAIN_CHUTE = 250.0f; // 點火後 250 秒強制開傘

    void saveStateToNVM(STATENUM state);
};

#endif
