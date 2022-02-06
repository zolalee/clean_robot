/*
 * Copyright (C) 2021 Useerobot. All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-25 17:10:50
 * @Project      : UM_path_planning
 */

#pragma once
#ifndef PATH_PLANNING_INTERFACE_H
#define PATH_PLANNING_INTERFACE_H
// #include <boost/bind.hpp>
#include  <string>
// #include  "lidar/lidar_shm.h"
#include  "um_chassis/shmCom.h"
#include  "semaphore.h"
#include  <sys/shm.h>
#include  <sys/sem.h>
#include  "um_chassis/uart.h"
#include  "common_function/logger.h"
#include  "common_function/MotionControl.h"
#include  "navigation_algorithm/GlobalPlanning.h"
#include  "common_function/Parse.h"
#include  "message/navigation_msg.h"
#include  "message/navigation_rpc_server.h"
#include  "message/navigation_server_handler.h"
#include  "core/log.h"
#include  "core/utils.h"
#include "navigation_algorithm/UTurnPlanning.h"
#include "common_function/ExceptionHanding.h"
// #include "common_function/lz4.h"
#include "navigation_algorithm/AlongWall.h"
#include "RemoteControllerTask.h"
#include "navigation_algorithm/InternalSpiralPlanning.h"
#include "um_chassis/logger.h"
#include "um_chassis/logger_rc.h"
// #include "boost/heap/d_ary_heap.hpp"


using namespace core;

namespace useerobot
{
extern SensorData_t sensordata;
extern current_pose_t* current_pose;
extern share_path_planning_t* current_path_planning;
extern current_pose_t share_pose;
extern WallFallowPara_t WallFallowPara;
extern bool cleanTaskOverIndex;
extern RoadAim record_charge_aim;
// extern share_map_t* current_full_map;
extern PRO process;
extern CHASSIS_CONTROL GLOBAL_CONTROL;
extern bool getBlockInfoIdenx;
extern bool getPointInfoIndex;
extern bool getForbbidenInfoIndex;
extern RoadAim road_aim;
extern int calibration_time;
extern bool gyro_stat_resp_state;
typedef void(*fun)(struct RobotCleanReq &rev);
// typedef std::function<void(struct RobotCleanReq *)> robot_clean_cb;


extern cornerIdex corner_index;
struct SystemStatusData
{
    double battery; //电池电量
    double batteryVoltage; //电池电压
    bool chargerStatus; //充电状态
    double speed; //机器人实时速度
    double totalMileage;//机器人总里程
    double uptime;//机器人运行时间
};


enum MAP_TYPE
{
  MAP = 1,
  RECORDMAP=2,
  LOCALMAP =3,
};
struct Robotstate 
{
  int robot_state;
};
class PathPlanningInterface
{
  public:
    PathPlanningInterface();
    ~PathPlanningInterface();
    /**
     * @description: 初始化清扫周期
     * @event: 
     * @param {*}
     * @return  {0 -成功}
     */
    void cleanCycleInit(); 
    /**
     * @description: 清扫开始
     * @event: 
     * @param {*}
     * @return {0 -成功}
     */
    void cleanStartRun(); 
    /**
     * @description: 结束清扫周期
     * @event: 
     * @param {*}
     * @return {0 -成功}
     */
    void cleanCycleDestroy();
    void AppControl();
    void handle_navigation_reqeust(struct RobotCleanReq *req);
    void cleanPlanningRun(struct RobotCleanReq &rev); //清扫服务运行
    void SLAMPathPLanning();//建图清扫
    void Init();
    void InitRecordMap();// 初始化recordmap;
    void SetMap(int cols,int rows,int maptype);
    void getRobotState();
    void stop_task();
    void clear_state();
    void suspendClean();//暂停任务
    void resumeClean();//回复任务
    void stopClean();//停止任务
    int SignManage(Sensor sensor,Grid cur);
    void gamepadControl();
    void divideRoomToBlock(BlockCorner &selectBlockCorner,RobotType &robotShape); //区块分解
    void judgePosition();//判断当前位置在那个区块
    Point* judgeBlockCorner(const Point &startPoint, const BlockCorner &selectBlockCorner, RobotType &robotShape);//判断离区块的哪个角点最近
    void pointClean(Point &sweepPoint,RobotType &robotShape);  //定点清扫
    void autoClean(RobotType &robotShape);//自动清扫
    void reLocalization(RobotType &robotShape);//重定位规划
    void appointRoomClean(BlockCorner &selectBlockCorner,RobotType &robotShape); //指定房屋清扫
    void appointBlockClean(BlockCorner &selectBlockCorner,RobotType &robotShape); //指定区块清扫
    void movingCharge();  //回充
    int  chargeFunction(int charger_x,int charger_y); //回充动作
    void alongWallClean();//沿墙清扫
    void updateRobotstate();
    void ClearRecodrdMap();
    void LeaveCharger();//下回充座
    void randomClean();
    void escapescene();//脱困处理
    void setChargeMode(RoadAim charge_aim);
    void rechargeTask(); // 回充处理
    RobotPlanningState getRobotPlanningState();
    static void sensorMemCallBack(SensorData_t sensorData);
    void RectangleTest(Sensor sensor,Grid cur);
    void uploadArea();
    void InitGlobalMap();
    // void _SensorDataCallback(UserSensorData_t *data);
    // void _ImuDataCallback(UserImuData_t *data);
    useerobot::MotionControl *motion_control;
    void *shm = NULL;
    // share_use_sensor_t* shared = NULL;
    std::string server_address;
    std::shared_ptr<NavigationRpcServer> server;
    std::shared_ptr<NavigationServerHandler> handler;
    std::shared_ptr<RemoteControllerTask> remote_controller;
    int count;
    bool success;
    RobotType cur_RobotType;
    UTurnPlanning arch;
    blockPlanning block;
    RoadPlanning road;
    MotionControl motion;
    EscapePlan escape;
	  chassisBase chassis;
    AlongWall _wall;
    InternalSpiralPlanning internal_planning;
    
    bool autodividerooms;
    RobotType *_robotShape;
    GAMEPAD_CONTROL CURRENT_GAMEPAD_STATE;
    
    aStar astar;
    dwa dwaRun;
    useerobot::MapFunction   map_function;
    // SensorData_t *sensordata; 

    Robotstate *robotstate;
    int rechargeindex;
    bool blockCleanIndex = false;
    bool pointCleanIdex = false;
    bool left_charger_index =false;
    pointCoordinate blockCenterPoint;
    pointCoordinate pointCleanPosition;
    RoadAim tmp_aim;
    int last_map_area;
    bool escape_index  =false;
    bool last_escape_index = false;
  private:
    RobotPlanningStateData robot_current_state;
    RobotPlanningAreaData  current_clean_area;
    RobotCleanScene robot_current_clean_scene;
    RobotCleanReq rev_message;
    bool exit_ ;
    Grid current_pos;
    Sensor current_sensor;
    Planning_info current_planning_info;
    AlongWall Alongwall_task;
    SystemStatusData *system_status_data;
    // share_map_t* current_full_map;
    bool thread_index = false;
    pthread_t task_thread;
    std::shared_ptr<std::thread> prepare_thread_ = nullptr;
    std::shared_ptr<std::thread> slamPathPLanning_thread_ = nullptr;
    std::shared_ptr<std::thread> rondomPLanning_thread_ = nullptr;
    std::shared_ptr<std::thread> leaveCharger_thread_ = nullptr;
    bool slamPathPLanning_state_index =false;
    bool leaveCharger_task_state = false;
    bool fisrt_correcting_map = true;
    bool last_cleanTaskOverIndex =false;
    bool remap_recharge_index = false;
    bool escape_fail_index = false;
    bool correct_map_resume_recharge_index = false;
    bool gyo_calibration_index =true;
    bool gyo_message_ok_index = true;
    int gyro_stat_resp_cycles = 0;
    // bool batvolume_Index = true;

};
}
#endif
