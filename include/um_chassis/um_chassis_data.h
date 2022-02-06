/*
 * @Author: DahlMill
 * @Date: 2020-10-02 12:54:37
 * @LastEditors  : Please set LastEditors
 * @LastEditTime : 2022-01-26 11:33:13
 */

#ifndef _UM_CHASSIS_DATA_H
#define _UM_CHASSIS_DATA_H


#define GYRO_SENSITIVITY_RECIPROCAL (1/131.2F)
#define ACC_SENSITIVITY_RECIPROCAL (1/1024)

typedef int int32_t;
typedef short int16_t;
// typedef char int8_t;

typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

enum CHASSIS_CMD
{
    CHASSIS_CMD_NONE = 0,
    CHASSIS_CMD_ROAD = 1,
    CHASSIS_CMD_OTA = 2,
};

typedef struct
{
    uint32_t RoboteTime;        //时间戳
    int16_t X_AccOriginal;      //X加速度
    int16_t Y_AccOriginal;      //Y加速度
    int16_t Z_AccOriginal;      //Z加速度
    int16_t X_GyroOriginal;     //X角速度
    int16_t Y_GyroOriginal;     //Y角速度
    int16_t Z_GyroOriginal;     //Z角速度
    int16_t X_AngleOriginal;    //X角度
    int16_t Y_AngleOriginal;    //Y角度
    int16_t Z_AngleOriginal;    //Z角度 
    int32_t Initialized;        //陀螺仪初始化标志   
    int32_t AddAngle;           //累计角度
    int16_t LeftWheel_Speed;    //左轮转速
    int16_t RightWheel_Speed;   //右轮转速
    uint16_t ldsCliff;          //地检信号
    uint8_t WallDirection;      //沿墙方向
    uint8_t dockSignal;         //回充+虚拟墙+配网信号
    int8_t Bump_Motor;         //碰撞+尘盒水箱+全方位
    uint8_t batVolume;         //电池电量
    int8_t robotMode;          //扫地机模式
    uint8_t cleanMode;          //扫地机清扫模式
    uint8_t alertNum;           //警报序号
    uint8_t fanSpeed;           //风量档位
    uint8_t waterSpeed;         //水量档位
    uint16_t voiceNum;          //语音序号
    uint16_t leftAlongWallValue;    //左沿墙开关灯差值
    uint16_t rightAlongWallValue;   //右沿墙开关灯差值
    int leftGeologicalDetect_index;                   //左地检触发
    int middleGeologicalDetect_index;                 //中地检触发
    int rightGeologicalDetect_index;                  //右地检触发
    int mcuLeftCliff;
    int mcuRightCliff;
    int mcuLeftMidCliff;
    int mcuRightMidCliff;
    int leftGeologicalDetect_index_on;                   //左地检开灯值
    int middleLeftGeologicalDetect_index_on;             //中左地检开灯值
    int middleRightGeologicalDetect_index_on;            //中右地检开灯值
    int rightGeologicalDetect_index_on;                  //右地检开灯值
    int leftGeologicalDetect_index_off;                  //左地检关灯值
    int middleLeftGeologicalDetect_index_off;            //中左地检关灯值
    int middleRightGeologicalDetect_index_off;           //中右地检关灯值
    int rightGeologicalDetect_index_off;                 //右地检关灯值

    uint16_t  leftOmnibearingSlow_index;              //左全方位减速触发
    uint16_t  middleOmnibearingSlow_index;            //中全方位减速触发
    uint16_t  rightOmnibearingSlow_index;             //右全方位减速触发
    uint16_t  leftOmnibearingTurn_index;              //左全方位转向触发
    uint16_t  middleOmnibearingTurn_index;            //中全方位转向触发
    uint16_t  rightOmnibearingTurn_index;             //右全方位转向触发       
    uint16_t leftOmnibearingOn_index;              //左全方位开灯值
    uint16_t midOmnibearingOn_index;            //中全方位开灯值
    uint16_t rightOmnibearingOn_index;             //右全方位开灯值
    uint16_t leftOmnibearingOff_index;             //左全方位关灯值
    uint16_t midOmnibearingOff_index;           //中全方位关灯值
    uint16_t rightOmnibearingOff_index;            //右全方位关灯值 
    
    uint32_t left_virtulwall;              // 左虚拟墙
    uint32_t right_virtulwall;             // 右虚拟墙
    uint32_t frontLeft_virtulwall;         // 前左虚拟墙
    uint32_t frontRight_virtulwall;        // 前右虚拟墙
    uint32_t behindLeft_virtulwall;        // 后左虚拟墙
    uint32_t behindRight_virtulwall;       // 后右虚拟墙

    int leftInfrared_index;            //左红外信号
    int rightInfrared_index;           //右红外信号
    int leftFrontInfrared_index;       //左前红外信号
    int rightFrontInfrared_index;      //右前红外信号

    uint32_t rechargeShrapnel_index;            //充电弹片
    
    float leftWheelElectricity_index;           //左轮电流
    float rightWheelElectricity_index;          //右轮电流

    uint16_t leftSideBrushElectricity;
    uint16_t rightSideBrushElectricity;

    uint16_t rollBrushElectricity;
    
    // uint16_t runTime;
}SensorData_t;

// typedef struct
// {
//     uint32_t RoboteTime;        // 时间戳 (0.1ms)
//     float X_AccOriginal;        // X加速度
//     float Y_AccOriginal;        // Y加速度
//     float Z_AccOriginal;        // Z加速度
//     float X_GyroOriginal;       // X角速度
//     float Y_GyroOriginal;       // Y角速度
//     float Z_GyroOriginal;       // Z角速度
//     int16_t IMU_Temperature;    // IMU温度
// }IMU_t;

typedef struct
{
    uint8_t len;
    uint8_t data[32];
    uint8_t cmd;
}ControlData_t;

typedef enum {SHM_WRITE_SENSOR,SHM_UPDATE_SENSOR,SHM_READ_SENSOR,SHM_OUTDATE_SENSOR} shm_states_sensor_t;
typedef struct share_use_sensor_t{
    volatile shm_states_sensor_t status; //4
    long long RoboteTime;        //时间戳
    int16_t X_AccOriginal;      //X加速度
    int16_t Y_AccOriginal;      //Y加速度
    int16_t Z_AccOriginal;      //Z加速度
    int16_t X_GyroOriginal;     //X角速度
    int16_t Y_GyroOriginal;     //Y角速度
    int16_t Z_GyroOriginal;     //Z角速度
    int32_t AddAngle;           //累计角度
    int16_t LeftWheel_Speed;    //左轮转速
    int16_t RightWheel_Speed;   //右轮转速
    uint16_t ldsCliff;          //地检信号
    uint8_t WallDirection;      //沿墙方向
    uint8_t dockSignal;         //回充+虚拟墙+配网信号
    uint8_t Bump_Motor;         //碰撞+尘盒水箱+全方位
    uint8_t BatVoltage;         //电池电量
    uint8_t robotMode;          //扫地机模式
    uint8_t cleanMode;          //扫地机清扫模式
    uint8_t alertNum;           //警报序号
    uint8_t fanSpeed;           //风量档位
    uint8_t waterSpeed;         //水量档位
    uint16_t voiceNum;          //语音序号
}share_use_sensor_t;


typedef struct QNode{
    //int data;
    share_use_sensor_t sensor_data;
    struct QNode * next;
}QNode;
//QNode * initQueue(){
//    QNode * queue=(QNode*)malloc(sizeof(QNode));
//    queue->next=NULL;
//    return queue;
//}
//QNode* enQueue(QNode * rear,share_use_sensor_t data){
//    QNode * enElem=(QNode*)malloc(sizeof(QNode));
//    enElem->sensor_data=data;
//    enElem->next=NULL;
//    //使用尾插法向链队列中添加数据元素
//    rear->next=enElem;
//    rear=enElem;
//    return rear;
//}
//QNode* DeQueue(QNode * top,QNode * rear){
//    if (top->next==NULL) {
//        printf("\nIMU队列为空");
//        return rear;
//    }
//    QNode * p=top->next;
//    //printf("%d ",p->data);
//    top->next=p->next;
//    if (rear==p) {
//        rear=top;
//    }
//    free(p);
//    return rear;
//}


#endif

