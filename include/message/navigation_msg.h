#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <vector>
#include <msgpack.hpp>

#include <core/header.h>


enum RobotCleanScene
{
    WHOLETRAVERSEROOMS= 0,      //全屋逐房遍历
    WHOLETRAVERSEBLOCKS= 1,     //区块遍历
    SLAMPlANNING = 2 ,          //建图清扫
    TRAVERSESELECTEDROOMS = 3,  //指定房间遍历
    GOTOCLEANPOINT = 4,         //指定点清扫
    GOTOBLOCKCLEAN = 5,         //指定区块清扫
    CHARGE = 6,                 //回充
    ALONGWALL = 7,              //沿墙
    RELOCALIZATION = 8,         //重定位
    STOP = 9,                   //停止任务
    PAUSE = 10,                 //暂停任务
    RESUME = 11,                //恢复任务
    GAMEPAD = 12,               //APP遥控器控制
    REMOTECONTROL = 13,         //遥控器控制
    CONDITION = 14,             //工况
    LEFT_CHARGER = 15,       //下回充座
    RONDOM =16,              //随机清扫
    FORBIDDEN_INFO_UPDATE =17, //禁区信息更新
    RESUME_BROKEN_CLEAN =18, //　断点续扫
};

enum RobotPlanningState
{
    IDLE = 0,                   //空闲中
    LOCALIZATION_FAILED = 1,    //定位失败
    GOAL_NOT_SAFE = 2,          //目的地有障碍
    UNREACHABLE = 3,            //目的地不可达
    REACHED = 4,                //目的地到达
    HEADING = 5,                //指定区块清扫中
    PLANNING= 6,                //规划中
    TOO_CLOSE_TO_OBSTCALES = 7, //离障碍太近
    CLEANNING = 8,              //清扫中
    AVOIDING_OBSTACLE = 9,      //避障中
    SEARCHING_CHARGER = 10,     //寻找充电桩
    RECHARGING =11,             //充电中
    SUSPEND =12,                //暂停
    PREPARED_PLANNING =13, //下充电座成功
    TASK_FAILED =14, //下回充座失败
    CLEAN_FINISHED = 15, //清扫完成
    RECHARGER_REACHED =16,//到达充电座附近
    BROKEN_CLEAN_RECHARGE =17, //断点续扫回充
    ESCAPE_FAILED = 18,       //脱困失败
    SIDE_BRUSH_ABNORMAL =19, //边刷异常
    SIDE_BRUSH_NORMAL =20,   //边刷恢复正常
    ESCAPING =21, //脱困中
    ESCAPE_SUCCESS = 22, //脱困成功
    SIDE_BRUSH_ALARM =23, //边刷被困报警
    WHEEL_ABNORMAL_ALARM =24, //左右轮电流异常报警
    UPLIFT_ALARM =25, //抬起地检报警
    ROUND_BRUSH_ALARM = 26, //滚刷异常报警
    GYO_CALIBRATION_SUCCESS = 27, //陀螺仪矫正成功
    GYO_CALIBRATION_FAILED = 28 ,//陀螺仪矫正失败
};

enum GamepadCommand {
    GAMEPAD_IDLE = 0,
    GAMEPAD_GO,
    GAMEPAD_TURN_LEFT,
    GAMEPAD_TURN_RIGHT,
    GAMEPAD_TURN_BACK,
};

enum RemoteControlCommand {
    REMOTE_CONTROL_IDLE = 0,
    REMOTE_CONTROL_GO,
    REMOTE_CONTROL_TURN_LEFT,
    REMOTE_CONTROL_TURN_RIGHT,
    REMOTE_CONTROL_TURN_BACK,
};

enum RobotConditionCommand 
{
    ROBOT_PULL_UP = 1,          //抬起
    ROBOT_WHEEL_LOCKED_ROTOR,   //轮子堵转
    ROBOT_LOCK_ON_CHAIR,        //椅子底部卡死
    ROBOT_WIRE_WOUND,           //电线缠绕
};

struct PointCoordinate
{
    int x;
    int y;

    MSGPACK_DEFINE(x, y);
};

struct BlockCorner
{
    PointCoordinate topLeftCorner;  //区域左上角坐标
    PointCoordinate topRightCorner; //区域右上角坐标
    PointCoordinate bottomLeftCorner; //区域左下角坐标
    PointCoordinate bottomRightCorner; //区域右下角坐标

    MSGPACK_DEFINE(topLeftCorner, topRightCorner, bottomLeftCorner, bottomRightCorner);
};

struct RobotCleanData
{
    int scene;
    int roomNumber; //遍历房间数量
    int blockNumber; //遍历区块数量
    int cleanTimes; //清扫次数
    std::vector<BlockCorner> blockCorners; //遍历区块角点
    std::vector<int> roomCornerIds; //遍历多个房间ID
    BlockCorner blockCorner;       //指定区块角点
    int roomCornerId;                 //指定房间角点
    PointCoordinate selectPoint; //指定点
    int gamepadCommand;         // 遥控命令
    int remoteControlCommand; //遥控器命令
    int conditions;            //工况
    int chargeMode;            //充电模式
    MSGPACK_DEFINE(scene, roomNumber, blockNumber, cleanTimes ,blockCorners, roomCornerIds, blockCorner, roomCornerId, selectPoint, gamepadCommand,remoteControlCommand,conditions,chargeMode);
};


// struct RobotCleanData
// {
//     int scene;
//     int roomNumber; //遍历房间数量
//     int blockNumber; //遍历区块数量
//     std::vector<BlockCorner> blockCorners; //遍历区块角点
//     std::vector<int> roomCornerIds; //遍历多个房间ID
//     BlockCorner blockCorner;       //指定区块角点
//     int roomCornerId;                 //指定房间角点
//     PointCoordinate selectPoint; //指定点
//     int gamepadCommand;         // 遥控命令
//     int remoteControlCommand; //遥控器命令
//     int conditions;            //工况
//     MSGPACK_DEFINE(scene, roomNumber, blockNumber,blockCorners, roomCornerIds, blockCorner, roomCornerId, selectPoint, gamepadCommand,remoteControlCommand,conditions);
// };

enum ChargeMode
{
    LOW_POWER_CHARGE = 0,
    EXTERNAL_CMD_CHARGE,
};
struct RobotPlanningStateData
{
    int planningState;
    MSGPACK_DEFINE(planningState);

};
struct RobotPlanningAreaData
{
    int clean_area;
    MSGPACK_DEFINE(clean_area);

};

enum navigation_msg_type {
    kNavigationMsgRobotCleanData = 1,
    kNavigationMsgRobotPlanningStateData,
    kNavigationMsgRobotCleanArea,
};

struct RobotCleanReq {
    struct core::header header;
    struct RobotCleanData data;
};

struct RobotCleanRes {
    struct core::header header;
    int success;
};

struct RobotPlanningStateReq {
    struct core::header header;
    struct RobotPlanningStateData data;
};

struct RobotPlanningStateRes {
    struct core::header header;
    int success;
};

#endif
