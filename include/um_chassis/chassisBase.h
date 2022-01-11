/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-17 10:48:29
 * @LastEditTime : 2022-01-10 09:19:27
 * @Project      : UM_path_planning
 */
/*******
1.轮子速度控制范围 0-1000  单位 mm/s
2.需要实现锁角度的效果
3.缺少写入共享内存部分
*******/
#ifndef CHASSISBASE_H
#define CHASSISBASE_H
#include <string>
#include <math.h>
#include <unistd.h>
#include "um_chassis/um_chassis_data.h"
#include "common_function/logger.h"
#include "lidar/lidar_shm.h"
#include "common_function/MapFunction.h"
// extern "C" {
#include "um_chassis/um_chassis_protocol.h"
// }


using namespace std;
extern float angle_;
extern float gyo_angle_;
static float _Pi = 3.1415926;
namespace useerobot {


//位姿共享内存
typedef struct current_pose{
  double x;
  double y;
  double theta;
  int is_correcting;
  int is_relocat_done;
}current_pose_t;

enum CHASSIS_CONTROL
{
    WHEEL_RUN,
    WHEEL_PAUSE,
    WHEEL_STOP,
    WHEEL_INVALID
};
enum GAMEPAD_CONTROL
{
    GAMEPAD_CONTROL_ON,
    GAMEPAD_CONTROL_OFF    
};

//轮子状态
enum WHEELSTATE{
    WHEELSTOP = 0,       //停止
    WHEELFRONT,          //前进
    WHEELBEHIND,         //后退
    WHEELLEFTSPIN,      //左自旋
    WHEELRIGHTSPIN,     //右自旋
    WHEELNUILROTRIGHT,         //右单边选    
    WHEELNUILROTLEFT           //左单边旋
};
int round_doubel(double number);
class chassisBase 
{
public:
        int jumpingCount;
        static chassisBase* getInstance();
		SensorData_t chassisData;
private:
        static chassisBase *m_Instance;
        // ControlData_t controlData;
        ControlData_t controlData;
        GridPose robotPose;
        GridPose targetRobotPose;      
        float  targetAngle;
        float  Diff_angle = 0.03; // 弧度？角度
        float  variance = 0.1;
        enum CHASSIS_CMD
        {
                CHASSIS_CMD_NONE = 0,
                CHASSIS_CMD_ROAD = 1,
                CHASSIS_CMD_OTA = 2,
        };
        enum CHASSIS_MODE
        {
                ELSE_MODE, 
                NORMAL_MODE,
                EMERGNCY_BRAKE_MODE,
                PRODUCT_MODE, 
                INIT_MODE, 
        };
        enum CHASSIS_WAY
        {
                goSTRAIGHT,
                turnRIGHT,
                turnLEFT, 
        };
        

public:
        chassisBase();
        ~chassisBase();
        int outWall;
        Sensor cur_sensor;
        double Conlaser();
        void GetSensor(Sensor* pss);
        void GridPoint(Grid* point);
        void getPlanningInfo(Planning_info *planning_info);
        WHEELSTATE getWheelState();
        void wheelCtrlStop();
        int chassisSpeed(int16_t speed,int way);
        int chassisSpeed(int16_t leftspeed,int16_t rightspeed,int mode);
        int chassisRoadFIX();
        int chassisRecharge();
        void leaveChargerParaInit();
        //底层脱困 不确定
        int escapeJail();

        int alongWall(uint8_t mode,uint8_t direction);
        /**
         * @description: 机器人直线行走
         * @event: 
         * @param {int16_t} speed
         * @return {*}
         */        
        int MakeChassisGoStraight(int16_t speed,GridPose targetPose);
        
        /**
         * @description: 机器人左转接口
         * @event: 
         * @param {int16_t} speed
         * @param {int} angle
         * @return {*}
         */        
        int MakeChassisTurnLeft(int16_t speed,int angle); // angle is negative
        
        /**
         * @description: 机器人右转接口
         * @event: 
         * @param {int16_t} speed
         * @param {int} angle
         * @return {*}
         */        
        int MakeChassisTurnright(int16_t speed,int angle); //angle is positive
        
        /**
         * @description: 机器人回充接口
         * @event: 
         * @param {*}
         * @return {*}
         */    
        int MakeBaseRecharge();
        /**
         * @description: 
         * @event: 
         * @param {int16_t} leftspeed
         * @param {int16_t} rightspeed
         * @return {*}
         */
        int MakeChassisGoTurn(int16_t leftspeed,int16_t rightspeed,int16_t times);   

        /**
         * @description: 机器人脱困接口
         * @event: 
         * @param {*}
         * @return {*}
         */        
        int MakeBaseEscapeJail();

        /**
         * @description: 机器人沿墙接口
         * @event: 
         * @param {uint8_t} mode
         * @param {int} direction
         * @return {*}
         */        
        int MakeBaseAlongWall(uint8_t mode,int direction);
        
        /**
         * @description: 下回充座
         * @event: 
         * @param {*}
         * @return {*}
         */        
        int rechargeRetreat();

        /**
         * @description: 重定位
         * @event: 
         * @param {*}
         * @return {*}
         */   
        void relocation();

        void simulationInit();
        int MakeSmartRecharge();
        int MakeSmartAlongWall();
        int MakeSmartExcapeJail();
        int chassisSimulationSpeed(double leftspeed,double rightspeed,int mode);
        GridPose GetSimulationCurGridPose();
        Maze getSimulationMap();

        // Maze getSimulationMap(const nav_msgs::OccupancyGrid &map_value);
        int right_allwalldirection = 1;
        int left_allwalldirection = 2;
        uint8_t enter_alongwallmode = 00;
        uint8_t exit_alongwallmode = 01;
        string _rosSrvrIp; 
        int feedback;
        Sensor curSensor;
        Grid curGrid;
        // char *rosSrvrIp = "192.168.3.54";
        bool leave_recharge_index =false;
        int  leave_recharge_state = -1;
        // bool leaveCharger_task_state = false;

private:
        uint8_t RightAlongwall = 1;
        uint8_t LeftAlongwall = 2;

        int leaveCharger_backTime = 200;
        int leaveCharger_backsign = 1;
        int leaveCharger_rotateSign = 1;
        float leaveCharger_aimForward,leaveCharger_recordForward;
        int forbbindenAreaNums = 0;
        

        Maze map_maze_temp;
};
}

#endif