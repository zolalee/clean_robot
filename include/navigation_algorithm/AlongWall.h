/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-09-23 14:26:03
 * @Project      : UM_path_planning
 */


#pragma once
#ifndef ALONGWALL_H
#define ALONGWALL_H

#define IR_BEGIN_ALONG     300       //左沿墙红外进入沿墙阈值
#define IR_NO_WALL         600       //判定无沿墙的值
#define IR_FAR_WALL        500       //判定离墙远
#define DEFALUT_WALL_VAL   2900      //默认沿墙红外值


#include "common_function/logger.h"
#include "navigation_algorithm/GlobalPlanning.h"
#include "common_function/MotionControl.h"

#include <unistd.h>

using namespace std;

namespace useerobot
{
    
    extern CHASSIS_CONTROL GLOBAL_CONTROL;
    /*沿墙方向*/
    typedef enum 
    {

        NoDir = 0,          //未指定沿墙方向

        RIGHTAW,              //右方向

        LEFTAW,               //左方向

    }WallFollowDir_t;

    /*PID*/
    typedef struct 
    {

        int16_t PI;

        int16_t P;

        int16_t Last_P;

        int16_t PD;

    }PID_t;
    typedef enum
    {
        LEFT_ROTATE =0,
        RIGHT_ROTATE,

    }Rotate_direction;
    /*碰撞状态*/
    typedef enum 
    {

        BumpNo = 0,           //没有碰撞0

        OBSSlow,              //全方位减速1

        InsideVirtual,        //靠墙一侧有虚拟墙信号2

        FrontVirtual,         //有虚拟墙信号3

        BumpLeft ,            //左碰撞4

        BumpRight,            //右碰撞5

        BumpInter,            //中碰撞6

        CliffLeft,            //左探地7

        CliffInter,           //中探地8

        CliffRight,           //右探地9

        MagVirtualLeft,       //磁性虚拟墙左10

        MagVirtualInter,      //磁性虚拟墙中11

        MagVirtualRight,      //磁性虚拟器右12

        OBSLeft,                //全方位左13

        OBSFront,               //全方位中14

        OBSRight,               //全方位右15

        ForbidFront,            //前禁区

        ForbidLeft,             //左禁区

        ForbidRight             //右禁区

    }BumpList_t;

    /*类碰撞处理状态*/
    typedef enum

    {

        BumpNoAction = 0,

        BumpSignalBack ,                 //机器后退1

        BackWithoutSignal,               //后退直到无信号2

        BumpBackWithoutSignal,           //前撞后退到无信号，特殊处理3

        WarnTurn,                        //旋转报警4

        WaitBack,                        //等待后退完成5

        Virtual5253Turn,                 //绕虚拟墙时收到5253时进行旋转6

        Turn,                            //机器旋转7

        WaitTurn,                        //等待机器自旋完成8

        WaitTurnOrEscape,                //等特定的旋转否则脱困9

        TurnWithOutVirtual,              //虚拟墙旋转直到没有信号10

        TurnVirtual,                     //虚拟墙旋转11

        VirtualTurnAimAngle,             //虚拟墙旋转到目标角度12

        AdjustTurnAimAngle,              //自适应旋转到目标角度13

        FindWallTurn,                    //找墙转弯14

        WallALongTurn,                   //自适应旋转15

        WaitWallALongTurn,               //等待自适应旋转完成16

        WallValueTurn,                   //找墙时对沿墙值进行自旋，一般是跟沿墙方向相反的才会进行操作17

        NoWallGoFolow,                   //无墙前进18

    }WallFollowBumpDealState_t;

    /*沿墙状态机*/
    typedef enum 
    {

        WallFollowStart = 0,//沿墙开始状态

        WallFollowFind,     //1找墙状态

        WallFollowAlong,    //2直线沿墙状态

        WallFollowEnd,      //3结束状态

        WallFollowPause,    //4暂停状态

        WallFollowVIRTUAL,  //5躲避虚拟墙
        
        WallFollowForbid    //6躲避禁区
    }WallFollowState_t;

    /*沿墙速度选择*/
    typedef enum 
    {

        SLOW = 0,   //慢沿墙

        QUICK,      //快沿墙


    }SPEED_t;

    /*沿墙状态状态，用来向外界反馈状态*/
    typedef enum 

    {

        WallFollow_Stop = 0,        //沿墙停止

        WallFollow_Run ,            //正在沿墙

        WallFollow_Pause,           //暂停沿墙

    }WallFollowRunState_t;

    /*类碰撞警报*/
    typedef enum 
    {

        NoWarn,

        CliffWarn,

        BumpWarn,

        MagVirtual,

    }Turn_Warn_t;

    /*沿墙模式*/
    typedef enum 
    {

        NoNeedWallFollow = 0,     //不沿墙标志

        RemoteWallFollow ,        //1单沿墙（app或者遥控触发）

        UncleanWallFollow,        //2沿墙找未清扫区域（工字型结束后执行）

        FiledWAllFollow,          //3工字型沿墙（工字型中碰撞触发）

        DockWallFollow,           //4回充沿墙

        RandomWallFollow,         //随机沿墙

    }WallFollowModel_t;


    /*沿墙参数,方便清空*/
    typedef struct 
    {
        // bool alongwalk_run_index;       //沿墙模式启动标志位
        
        bool aroundFindWallFlag;                         //记录判定为绕柱后找墙的标志

        WallFollowDir_t roundWallPreDir;                //记录判定为绕柱之前沿墙的方向

        int16_t roundFindCnt;                           //记录判定为绕柱后找墙的次数

        int32_t roundFindStartAngle;                    //绕柱找墙开始角度

        int32_t findWallDistan;                          //记录判定为绕柱后找墙的距离

        uint32_t Time;                                   //沿墙时间

        uint32_t FindWallTime;                           //单边旋找墙时间

        uint32_t ShiJian;                                //用来进行检测沿墙操作的时间

        int32_t StartTrunAngle;                          //旋转找墙

        int32_t ValueErrStartTrunAngle;                  //自适应失败起始旋转角度

        int32_t StartAngle;                              //沿墙起始角度

        int32_t AddAngle;                                //沿墙绕柱累积角度

        int32_t LastAddAngle;                            //上次累加角度

        int32_t AllAngle;                                //总共沿墙的角度

        uint32_t AimTrunAngle;                           //目标旋转角度

        PID_t PID;                                       //沿墙PID相关参数

        uint16_t Dis;                                    //沿墙距离

        int16_t Value;                                   //当前沿墙值

        uint16_t NoWallCnt;                              //无墙时间

        uint16_t AloneTime;                              //用来计算是否丢失墙时要往前走一段距离

        uint8_t BumpFlag;                                //进入类碰撞处理状态，置一进入碰撞处理，不进入沿墙处理

        uint8_t StopFlag;                                //停止标志，后退完成会进入结束沿墙

        uint8_t PauseFlag;                               //暂停标志，旋转完成会进入暂停沿墙

        uint8_t BumpCnt;                                 //碰撞计数，为类碰撞做计时，防止动作被打断卡死

        uint8_t GoFollowFlag;                            //标志丢失墙是否需要往前走

        uint8_t FanEscapeFlag;                           //标志用来绕风扇座的标志

        uint8_t VTurnCnt;                                //靠虚拟墙计数

        uint8_t TurnErrCnt;                              //旋转未达到目标角度次数，连续两次没通过直接报警

        BumpList_t BumpRecord;                           //碰撞记录，处理完了会清零，实时更新

        BumpList_t BumpState;                            //碰撞状态，无检测碰撞发生会清零

        WallFollowBumpDealState_t  BumpDealState;        //碰撞处理状态机

        WallFollowState_t          State;                //沿墙动作状态机

        WallFollowState_t          PauseState;           //暂停时的状态机

        SPEED_t Speed;                                   //沿墙速度

        WallFollowRunState_t WallFollowRunState;         //沿墙状态标志

        Turn_Warn_t TurnWarn;                            //类碰撞警告

        WallFollowModel_t Model;                         //沿墙模式

        WallFollowDir_t Dir;                             //沿墙方向

        WallFollowDir_t FixedDirection;

    }WallFallowPara_t;

    typedef enum{
      EXIT_WALL,
      LEFT_WALL,
      RIGHT_WALL, 
    }Alongwallstate_t;

    

    Alongwallstate_t IsWall();

    WallFollowDir_t getAlongWallDir();

    /**
     * @name:   StartWallFollow
     * @msg:    沿墙开始
     * @param   Wall_Follow_model:沿墙模式 dir:沿墙方向
     * @return:
     */
    void StartWallFollow(WallFollowModel_t Wall_Follow_model, WallFollowDir_t dir, SPEED_t speed);

    /**
     * @name:   StopWallFollow
     * @msg:    沿墙结束
     * @param
     * @return: 
     */
    void StopWallFollow();

    /**
     * @name:   PauseWallFollow
     * @msg:    暂停沿墙
     * @param
     * @return:
     */
    void PauseWallFollow(void);
    
    /**
     * @name:   ContinueWallFollow
     * @msg:    继续沿墙
     * @param   
     * @return: 
     */
    void ContinueWallFollow(void);

    class AlongWall
    {
    private:
        /* data */
        // Maze _maze;
        Grid current_pos;
        

    public:
        chassisBase _chassis;
        Sensor currentSensor;
        // SensorData_t aw;
        /*沿墙参数*/
        int rotateFlag = 0;
        int deal = 0;
        int recognize = 0;
        int8_t lastWallState;
        AlongWall();
        ~AlongWall();
        /**
         * @description: use the chassis mode for alongwall
         * @event: 
         * @param {*}
         * @return {*}
         */        
        void chassisAlongWall();

        void quickWallAlong();

        /**
         * @name: WallFowllowBumpScan
         * @msg:  沿墙探地、全方位、碰撞扫描
         * @param:  
         * @return: 
         */
        void bumpScan();

        /**
         * @name:  WallFowllowBumpScan
         * @msg:   碰撞处理
         * @param: para 沿墙参数结构体 
         * @return: 
         */
        int bumpDeal();
        
        int wallObsFront();

        int getObsFront();

        int getObsLeft();

        int getObsRight();

        int getAddAngle();

        int recognizeBlackWall();

        void recognizeSmallPlace();

        /**
         * @name:   wallPid
         * @msg:    沿墙pid调节
         * @param
         * @return: 
         */
        int16_t wallPid(uint16_t NowValue,uint16_t AimValue);

        /**
         * @name:   WallFollow
         * @msg:    沿墙处理（20ms调用一次）
         * @param   
         * @return:
         */
        void wallFollowDeal(void);

        /**
         * @name:   INITPID
         * @msg:    沿墙pid初始化
         * @param:  
         * @return: 
         */
        void initPid(void);

        /**
         * @name:   FindWallByValue
         * @msg:    通过沿墙值找墙和自旋
         * @param   
         * @return: 
         */
        void findWallByValue();
        /**
         * @description: 
         * @event: 
         * @param {Rotate_direction} &rotate_direction
         * @return {*}
         */
        bool rotaterobot(float tmpforward, int angel);
        
        /**
         * @name:   wallBack
         * @msg:    后退
         * @param   
         * @return: 
         */
        bool wallBack();

        /**
         * @name:   getBanDis
         * @msg:    计算禁区距离
         * @param   
         * @return: 
         */
        float getBanDis(int dir);

        void fanBaseEscapeDiscern();

        void fanBaseEscapeFlagClear();

        bool isNear(float x, float y);
        bool escapeWallBack();
        void straightFindWall();
        void alongwallStart();
        void alongwallStop();
        chassisBase chassis;
        uint8_t alongWallMode;
        uint8_t alongWallDirection;
        MotionControl _motioncontrol_alongwall;
        std::shared_ptr<std::thread> alongwall_thread_ = nullptr;
        float recordForward;
        bool back_sign= false;
        int escapeFlag = 0;
        long long recordTime;
    };

       
}
#endif

