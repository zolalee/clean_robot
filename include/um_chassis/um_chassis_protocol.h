#ifndef __UM_CHASSIS_PROTOCOL_H__
#define __UM_CHASSIS_PROTOCOL_H__


//按键键值定义
#define KEY_IR_IDLE                     (0)         //空闲
#define KEY_IR_AUTOCLEAN                (1)         //弓字形清扫
#define KEY_IR_AUTOCLEANSTOP            (2)         //停止弓字形清扫
#define KEY_IR_RECHARGE                 (3)         //回充
#define KEY_IR_RECHARGESTOP             (4)         //停止回充
#define KEY_IR_ALONGWALLCLEAN           (5)         //沿墙清扫
#define KEY_IR_ALONGWALLCLEANSTOP       (6)         //停止沿墙清扫
#define KEY_IR_FIXEDPOINTCLEAN          (7)         //定点清扫
#define KEY_IR_FIXEDPOINTCLEANSTOP      (8)         //停止定点清扫
#define KEY_IR_OKSUSPEND                (9)         //开始、暂停
#define KEY_IR_FRONT_KEY                (10)        //前按键
#define KEY_IR_LEFT_KEY                 (11)        //左按键
#define KEY_IR_RIGHT_KEY                (12)        //右按键
#define KEY_IR_BEHIND_KEY               (13)        //后按键
#define KEY_IR_FINDSWEEPER              (14)        //寻找扫地机
#define KEY_IR_WATERFANGEARLOW          (15)        //水量风机低
#define KEY_IR_WATERFANGEARMID          (16)        //水量风机中
#define KEY_IR_WATERFANGEARHIGH         (17)        //水量风机高
#define KEY_IR_RANDOMCLEAN              (18)        //随机清扫
#define KEY_IR_RANDOMCLEANSTOP          (19)        //停止随机清扫
#define KEY_IR_APPOINTMENTCLEAR         (20)        //预约记录清除

#define KEY_IR_TEST_RUN                 (21)        //跑机台测试
#define KEY_IR_TEST_MOTOR               (22)        //堵转测试
#define KEY_IR_PLUS_VOLU                (25)        //遥控器上+ 风量水箱加
#define KEY_IR_LESS_VOLU                (26)        //遥控器上- 风量水箱减

#define KEY_IR_ARCHCLEAN                (27)        //弓字形
#define KEY_IR_MAP_RECHARGE             (28)        //地图回充
#define KEY_IR_SET_TIME                 (88)        //遥控器设置当前时间或者设置当前时间

#define KEY_HW_WORK                     (100)       //工作按键
#define KEY_HW_HOME                     (101)       //主按键
#define KEY_HW_FIXEDPOINT               (102)       //定点按键

//按键事件类型定义
#define EVENT_KEY_SHORT_PRESS       (0)         //短按
#define EVENT_KEY_DOUBLE_PRESS      (1)         //双击
#define EVENT_KEY_LONG_PRESS        (2)         //长按

//回充座信息状态定义
#define IR_VIRTUAL_WALL_NO          (0x00)      //无
#define IR_VIRTUAL_WALL_LEFT        (0x01)      //左
#define IR_VIRTUAL_WALL_RIGHT       (0x02)      //右
#define IR_VIRTUAL_WALL_CENTER      (0x03)      //中
#define IR_VIRTUAL_WALL_LR          (0x04)      //左+右
#define IR_VIRTUAL_WALL_LF          (0x05)      //左+中
#define IR_VIRTUAL_WALL_RF          (0x06)      //右+中
#define IR_VIRTUAL_WALL_LRF         (0x07)      //左+右+中

//类型定义
typedef int                 int32_t;
typedef short               int16_t;
//typedef char              int8_t;
typedef unsigned int        uint32_t;
typedef unsigned short      uint16_t;
typedef unsigned char       uint8_t;


//陀螺仪数据
typedef struct
{
    int16_t X_AccOriginal;                      //X加速度
    int16_t Y_AccOriginal;                      //Y加速度
    int16_t Z_AccOriginal;                      //Z加速度
    int16_t X_GyroOriginal;                     //X角速度
    int16_t Y_GyroOriginal;                     //Y角速度
    int16_t Z_GyroOriginal;                     //Z角速度
    int16_t X_AngleOriginal;                    //X角度
    int16_t Y_AngleOriginal;                    //Y角度
    int16_t Z_AngleOriginal;                    //Z角度
    int32_t AddAngle;                           //累计角度
    int32_t Initialized;                        //陀螺仪初始化标志
    int32_t SerialNumber;                       //序列号 (监控数据跳变)
} GyroData_t;

//主轮数据
typedef struct
{
    int16_t LeftWheel_Speed;                    //左轮转速
    int16_t RightWheel_Speed;                   //右轮转速
    float leftWheelElectricity;                 //左轮电流
    float rightWheelElectricity;                //右轮电流
} MainWheelData_t;

//警报模式
typedef enum
{
    ALERT_NONE = 0,                             //无
    ALERT_WHEEL_FUALT,                          //轮子异常
    ALERT_SIDEBRUSH_FUALT,                      //边刷异常
    ALERT_FAN_FUALT,                            //风机异常
    ALERT_ROLLBRUSH_FUALT,                      //滚刷异常
    ALERT_BOXTANK_FUALT,                        //尘盒水箱
    ALERT_OMNIBEARING_FUALT,                    //全方位异常
    ALERT_ALONGWALL_FUALT,                      //沿墙异常
    ALERT_COLLISION_FUALT,                      //碰撞异常
    ALERT_GRDCHECK_FUALT,                       //地检异常
    ALERT_ESCAPEJAIL_FAIL,                      //脱困失败
    ALERT_LOWPOWER_LEVEL1,                      //一级低电
    ALERT_ADAPTER_FUALT,                        //适配器异常
    ALERT_DIP_FAIL,                             //探底失败
    ALERT_GYRO_FUALT,                           //陀螺仪异常
    ALERT_POWER_UNDER50,                        //电量低于50%OTA
    ALERT_LOWPOWER_LEVEL2,                      //二级低电
    ALERT_SOFT_POWEROFF,                        //软关机
    ALERT_CHARGING,                             //充电警报
    ALERT_CLEAN_OVER,                           //清扫完成
    ALERT_RECHARGE_HOLDER_HID,                  //未找到回充座
    ALERT_DIP_ESCAPEJAIL_FAIL,                  //探底脱困失败
    ALERT_MOPSTENTS_REMOVE,                     //拖布支架移除
    ALERT_TANKMODE_RUNRANDOM,                   //水箱模式运行随机
    ALERT_SLAM_FUALT,                           //Slam异常
    ALERT_PROCESS_CRASHES,                      //进程死机
    ALERT_RADAR_FUALT,                          //雷达异常
} AlertMode_t;

//扫地机模式
typedef enum
{
    ROBOT_MODE_NONE = 0,                        //待机
    ROBOT_MODE_AUTO,                            //自动
    ROBOT_MODE_FIXED_POINT,                     //定点
    ROBOT_MODE_ALONG_WALL,                      //沿墙
    ROBOT_MODE_RANDOM,                          //随机
    ROBOT_MODE_RECHARGE,                        //回充
    ROBOT_MODE_CHARGE,                          //充电
    ROBOT_MODE_REMOTE,                          //遥控
    ROBOT_MODE_SLEEP,                           //休眠
    ROBOT_MODE_SEARCHCHARGER = 11,              //地图寻找回充座
} RobotMode_t;

//沿墙数据
typedef struct
{
    uint16_t leftAlongWallOnVal;                //左沿墙开灯值
    uint16_t rightAlongWallOnVal;               //右沿墙开灯值
    uint16_t leftAlongWallOffVal;               //左沿墙关灯值
    uint16_t rightAlongWallOffVal;              //右沿墙关灯值
} AlongWallData_t;

//全方位数据
typedef struct
{
    //MCU原始数据
    uint16_t leftOmnibearingOnVal;              //左全方位开灯值
    uint16_t middleOmnibearingOnVal;            //中全方位开灯值
    uint16_t rightOmnibearingOnVal;             //右全方位开灯值
    uint16_t leftOmnibearingOffVal;             //左全方位关灯值
    uint16_t middleOmnibearingOffVal;           //中全方位关灯值
    uint16_t rightOmnibearingOffVal;            //右全方位关灯值
    //内部处理结果
    uint16_t  leftOmnibearingSlow;              //左全方位减速触发
    uint16_t  middleOmnibearingSlow;            //中全方位减速触发
    uint16_t  rightOmnibearingSlow;             //右全方位减速触发
    uint16_t  leftOmnibearingTurn;              //左全方位转向触发
    uint16_t  middleOmnibearingTurn;            //中全方位转向触发
    uint16_t  rightOmnibearingTurn;             //右全方位转向触发
} OmnibearingData_t;

//地检数据
typedef struct
{
    uint16_t leftGeologicalDetectOnVal;         //左地检开灯值
    uint16_t middleLeftGeologicalDetectOnVal;   //中左地检开灯值
    uint16_t middleRightGeologicalDetectOnVal;  //中右地检开灯值
    uint16_t rightGeologicalDetectOnVal;        //右地检开灯值
    uint16_t leftGeologicalDetectOffVal;        //左地检关灯值
    uint16_t middleLeftGeologicalDetectOffVal;  //中左地检关灯值
    uint16_t middleRightGeologicalDetectOffVal; //中右地检关灯值
    uint16_t rightGeologicalDetectOffVal;       //右地检关灯值
    /* 内部处理结果 */
    int leftGeologicalDetect;                   //左地检触发
    int middleGeologicalDetect;                 //中地检触发
    int rightGeologicalDetect;                  //右地检触发
    /* MCU处理结果 */
    int cliffLeft;                              //mcu左地检触发
    int cliffMiddleLeft;                        //mcu中左地检触发
    int cliffMiddleRight;                       //mcu中右地检触发
    int cliffRight;                             //mcu右地检触发
} GeologicalDetect_t;

//边刷数据
typedef struct
{
    uint16_t leftSideBrushElectricity;          //左边刷电流
    uint16_t rightSideBrushElectricity;         //右边刷电流
} SideBrushData_t;

//滚刷数据
typedef struct
{
    uint16_t rollBrushElectricity;              //滚刷电流
} RollBrushData_t;

//供电信息
typedef struct
{
    uint16_t machineElectricity;                //整机电流
    uint16_t chargingElectricity;               //充电电流
    uint16_t adapterVoltage;                    //充电电压
    uint16_t batVoltage;                        //电池电压
    uint8_t batVolume;                          //电池电量
} PowerSupplyData_t;

//红外数据
typedef struct
{
    uint32_t leftInfraredData;                  //左红外数据
    uint32_t rightInfraredData;                 //右红外数据
    uint32_t frontLeftInfraredData;             //前左红外数据
    uint32_t frontRightInfraredData;            //前右红外数据
    uint32_t behindLeftInfraredData;            //后左红外数据
    uint32_t behindRightInfraredData;           //后右红外数据
} InfraredData_t;

//唤醒/定时
typedef struct
{
    uint32_t rechargeSeatWakeup;                //回充座唤醒
    uint32_t keyWakeup;                         //按键唤醒
    uint32_t irWakeup;                          //红外唤醒
    uint32_t rtcWakeup;                         //RTC唤醒
    uint32_t timing;                            //定时
} WakeupTiming_t;

//外设检测状态
typedef struct
{
    uint32_t rechargeShrapnel;                  //充电弹片
    uint32_t waterTank;                         //水箱
    uint32_t onOff;                             //开关
    uint32_t dustBox;                           //尘盒
    uint32_t dc;                                //DC
} PeripheralState_t;

//电源检测状态
typedef struct
{
    uint32_t wifi;                              //WIFI
    uint32_t vcc3v3;                            //3V3
    uint32_t fan;                               //风机
    uint32_t omnibearing;                       //全方位
    uint32_t dip;                               //探底
} PowerState_t;

//预约响应
typedef struct
{
    uint32_t appointmentTimeClear;              //清除预约时间
    uint32_t appointmentTimeCancel;             //预约时间取消
    uint32_t appointmentTimeSet;                //预约时间设定
    uint32_t currentTimeSet;                    //当前时间设定
} AppointmentAction_t;

//IMU数据
typedef struct
{
    uint32_t RoboteTime;        // 时间戳 (0.1ms)
    float X_AccOriginal;        // X加速度
    float Y_AccOriginal;        // Y加速度
    float Z_AccOriginal;        // Z加速度
    float X_GyroOriginal;       // X角速度
    float Y_GyroOriginal;       // Y角速度
    float Z_GyroOriginal;       // Z角速度
    int16_t IMU_Temperature;    // IMU温度
} IMU_t;

// 虚拟墙信号
typedef struct
{
    uint32_t left;              // 左虚拟墙
    uint32_t right;             // 右虚拟墙
    uint32_t frontLeft;         // 前左虚拟墙
    uint32_t frontRight;        // 前右虚拟墙
    uint32_t behindLeft;        // 后左虚拟墙
    uint32_t behindRight;       // 后右虚拟墙
} VirtualWall_t;

// 碰撞信号
typedef struct
{
    int left;       // 左碰撞信号
    int right;      // 右碰撞信号
} BumperData_t;

//各模块数据回调函数
typedef void (*GyroDataCallback)(GyroData_t *data);
typedef void (*MainWheelDataCallback)(MainWheelData_t *data);
typedef void (*AlertModeCallback)(AlertMode_t *data);
typedef void (*RobotModeCallback)(RobotMode_t *data);
typedef void (*AlongWallDataCallback)(AlongWallData_t *data);
typedef void (*OmnibearingDataCallback)(OmnibearingData_t *data);
typedef void (*GeologicalDetectCallback)(GeologicalDetect_t *data);
typedef void (*SideBrushDataCallback)(SideBrushData_t *data);
typedef void (*RollBrushDataCallback)(RollBrushData_t *data);
typedef void (*PowerSupplyDataCallback)(PowerSupplyData_t *data);
typedef void (*InfraredDataCallback)(InfraredData_t *data);
typedef void (*FanPulseCallback)(uint16_t data);
typedef void (*WakeupTimingCallback)(WakeupTiming_t *data);
typedef void (*PeripheralDetectCallback)(PeripheralState_t *data);
typedef void (*PowerDetectCallback)(PowerState_t *data);
typedef void (*AppointmentRespCallback)(AppointmentAction_t *data);
typedef void (*KeyEventCallback)(int key, int evt);
typedef void (*ImuDataCallback)(IMU_t *data);
typedef void (*BumperDataCallback)(BumperData_t *data);
typedef void (*VirtualWallCallback)(VirtualWall_t *data);

//SensorData数据回调结构体
typedef struct
{
    GyroDataCallback gyro_data_cb;
    MainWheelDataCallback mainwheel_data_cb;
    AlertModeCallback alert_mode_cb;
    RobotModeCallback robot_mode_cb;
    AlongWallDataCallback alongwall_data_cb;
    OmnibearingDataCallback omnibearing_data_cb;
    GeologicalDetectCallback grdcheck_data_cb;
    SideBrushDataCallback sidebrush_data_cb;
    RollBrushDataCallback rollbrush_data_cb;
    PowerSupplyDataCallback ps_data_cb;
    InfraredDataCallback ir_data_cb;
    FanPulseCallback fan_pulse_cb;
    WakeupTimingCallback wakeup_timing_cb;
    PeripheralDetectCallback peripheral_detect_cb;
    PowerDetectCallback power_detect_cb;
    AppointmentRespCallback appointment_resp_cb;
    KeyEventCallback key_event_cb;
    ImuDataCallback imu_data_cb;
    BumperDataCallback bumper_data_cb;
    VirtualWallCallback virtual_wall_cb;
} SensorDataCallback_t;


#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************
 * 功能描述： 协议初始化 (API接口函数使用前必须调用，且仅一次)
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_ChassisProtocolInit(void);


/**********************************************************************
 * 功能描述： 数据回调注册
 * 输入参数： [callback]-用户自定义传感器数据函数
 * 输出参数： 无
 * 返 回 值： 无
 **********************************************************************/
extern void UMAPI_ChassisCallbackRegister(SensorDataCallback_t callback);


/**********************************************************************
 * 功能描述： 检查内部IPC连接状态 (提供给上层用户判断，在连接状态下发送请求命令)
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 0 - 断开, 1 - 连接
 **********************************************************************/
extern int UMAPI_CheckIpcState(void);

/**********************************************************************
 * 功能描述： 是否触发超过1秒钟以上没有收到MCU数据更新
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 0 - 不休眠, 1 - 休眠
 **********************************************************************/
extern int UMAPI_CheckMcuSleep(void);


/**********************************************************************
 * 功能描述： 固件版本命令
 * 输入参数： 版本信息异步回调函数
 * 输出参数： [hwver]-硬件版本，占2字节
              [swver]-软件版本，占2字节
              [mcuid]-MCU ID，占12字节
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_GetFirmwareVersion(void (*ack_cb)(short hwver, short swver, unsigned int mcuid));


/**********************************************************************
 * 功能描述： 主轮速度控制
 * 输入参数： [mode]-工作模式 0:其他跑机模式 1:标准跑机模式 2:地检急刹模式 3:产测模式 4:扫地机初始化阶段 5:警报
 *          [lspeed]-左轮速度  （单位mm/s）
 *          [rspeed]-右轮速度  （单位mm/s）
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_MainWheelSpeed(int mode, short lspeed, short rspeed);


/**********************************************************************
 * 功能描述： 风机控制
 * 输入参数： [gear]-风机档位 0 关闭  1低档  2中档  3高档
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlWindMotor(int gear);


/**********************************************************************
 * 功能描述： 边刷控制
 * 输入参数： [gear]-边刷档位 0-1000PWM控制（越高转速越快）
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlSideBrush(int gear);


/**********************************************************************
 * 功能描述： 滚刷控制
 * 输入参数： [gear]-滚刷档位 0-1000PWM控制（越高转速越快）
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlRollBrush(int gear);


/**********************************************************************
 * 功能描述： LED设置
 * 输入参数： [type]-LED类型 1:电源指示 2:回充指示 3:double 4:wifi指示
 *          [color]-LED颜色 1:紫色 2:蓝色 3:红色
 *          [mode]-LED模式 0:常灭 1:常亮 2:半亮 3:呼吸 4:快闪 5:慢闪
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlLed(int type, int color, int mode);


/**********************************************************************
 * 功能描述： 外设开关控制
 * 输入参数： [wifi]-WIFI  0:关闭  1:打开
 *          [ps5v]-5V    0:关闭  1:打开
 *          [ps3v3]-3V3  0:关闭  1:打开
 *          [fan]-风机    0:关闭  1:打开
 *          [qfw]-全方位  0:关闭  1:打开
 *          [dip]-探底    0:关闭  1:打开
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlSwitch(int wifi, int ps5v, int ps3v3, int fan, int qfw, int dip);


/**********************************************************************
 * 功能描述： 固件升级 （目前从OTA进程自行发送，此接口暂时不用）
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_EnterMcuOta(void);


/**********************************************************************
 * 功能描述： 行走状态下发
 * 输入参数： [state]-行走状态 1:沿墙 2:直行 3:半圆 4:曲线跟踪 5:自旋 6:后退 7:脱困
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlWalkState(int state);


/**
 * 障碍物信息（定义用于下发到MCU）
 */
typedef struct {
    int left_up_distance;          //左上区域距离 0-50，单位cm
    int left_up_angle;             //左上区域角度 0-60度
    int middle_up_distance;        //中上区域距离 0-50，单位cm
    int middle_up_angle;           //中上区域角度 0-60度
    int right_up_distance;         //右上区域距离 0-50，单位cm
    int right_up_angle;            //右上区域角度 0-60度
    int left_down_distance;        //左下区域距离 0-50，单位cm
    int left_down_angle;           //左下区域角度 0-60度
    int middle_down_distance;      //中下区域距离 0-50，单位cm
    int middle_down_angle;         //中下区域角度 0-60度
    int right_down_distance;       //右下区域距离 0-50，单位cm
    int right_down_angle;          //右下区域角度 0-60度
} BarrierInfo_t;

/**********************************************************************
 * 功能描述： 障碍物信息下发
 * 输入参数： [info]-障碍物信息  详见BarrierInfo_t定义
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_BarrierInfoIssue(BarrierInfo_t info);


/**********************************************************************
 * 功能描述： 激光标定
 * 输入参数： [state]-标定状态  0：未标定  1：标定成功  2：标定失败
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_LaserCalibrate(int state);


/**********************************************************************
 * 功能描述： 扫地机模式控制
 * 输入参数： [mode]-标定状态
 *           0：待机
 *           1：自动
 *           2：定点
 *           3：沿墙
 *           4：随机
 *           5：回充
 *           6：充电
 *           7：遥控
 *           8：休眠
 *           11:地图回充
 *
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_RobotMode(int mode);


/**********************************************************************
 * 功能描述： 蜂鸣器控制
 * 输入参数： [onoff]     -开关
 *          [freq]      -频率
 *          [ring_cnt]  -响声次数
 *          [repeat_cnt]-循环次数
 *          [time_len]  -时长
 *
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlBuzzer(int onoff, int freq, int ring_cnt, int repeat_cnt, int time_len);


/**********************************************************************
 * 功能描述：  陀螺仪初始化/校准
 * 输入参数1: [action] - 控制选择
 *           0: 不初始化/不校准
 *           1: 初始化
 *           2: 校准
 * 输出参数： [async_stat_cb] - 控制结果回调函数(异步)
 *           0: 失败
 *           1: 成功
 * 输出参数： [sync_stat] - 控制结果输出(同步)
 *           0: 失败
 *           1: 成功
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
int UMAPI_CtrlGyro(int action, void(*async_stat_cb)(int), int *sync_stat);


/**********************************************************************
 * 功能描述： 水箱控制指令下发
 * 输入参数1: [OnMilliseconds],    水箱开启时间,0-60000毫秒
 * 输入参数2: [OffMilliseconds],   水箱关闭时间,0-60000毫秒
 *           OnMilliseconds==0 && OffMilliseconds==0,水箱关闭.
 * 输出参数： 无
 * 返 回 值： 0 - 成功, -1 - 失败
 **********************************************************************/
extern int UMAPI_CtrlWaterTime(int OnMilliseconds, int OffMilliseconds);

#ifdef __cplusplus
}
#endif

#endif  /* __UM_CHASSIS_PROTOCOL_H__ */
