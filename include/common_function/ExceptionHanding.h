/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-10 20:09:36
 * @Project      : UM_path_planning
 */


#pragma once

#ifndef EscapePlan_H
#define EscapePlan_H

#include "common_function/logger.h"
#include "navigation_algorithm/GlobalPlanning.h"
#include "common_function/MotionControl.h"
#include "navigation_algorithm/LocalPlanning.h"
#include "um_chassis/chassisBase.h"
#include "common_function/MapFunction.h"
#include "navigation_algorithm/AlongWall.h"
#include "sys/time.h"
using namespace std;
extern float gyo_angle_;
namespace useerobot
{
  
  struct SmallArea
  {
      Grid smallPoint;
      int smallWall;
  };
extern CHASSIS_CONTROL GLOBAL_CONTROL;
  //脱困动作枚举
typedef enum
{
    ESCACT_START = 0,       //0 脱困动作开始
    ESCACT_BEHIND,          //1 后退
    ESCACT_BEHIND_WAIT,     //2 后退等待
    ESCACT_SPIN_LEFT,       //3 左自旋
    ESCACT_SPIN_LEFT_WAIT,  //4 左自旋等待
    ESCACT_SPIN_RIGHT,      //5 右自旋
    ESCACT_SPIN_RIGHT_WAIT, //6 右自旋等待
    ESCACT_NUILROT_LEFT,    //7 左单边旋
    ESCACT_NUILROT_LEFT_WAIT, //8 左单边旋等待
    ESCACT_NUILROT_RIGHT,     //9 右单边旋
    ESCACT_NUILROT_RIGHT_WAIT,//10 右单边旋等待
    ESCACT_SUCCESS,           //11脱困成功
    ESCACT_FAILD,           //12脱困失败
    ESCACT_END,               //13脱困结束
    ESCACT_FRONT,             //14前进
    ESCACT_FRONT_WAIT,        //15前进等待
}escapeAction_t;

typedef enum{
    ESC_STEP_START = 0,//脱困开始
    ESC_STEP_0,//1
    ESC_STEP_1,//2
    ESC_STEP_2,//3
    ESC_STEP_3,//4
    ESC_STEP_4,//5
    ESC_STEP_SUCCESS,//6脱困成功啦
    ESC_STEP_FAILD,//7脱困失败啦
    ESC_STEP_ABERRANT,//8异常处理（探地） 
    ESC_STEP_OVERHEAD,//9架空处理
    ESC_STEP_STUCK_HEAD,//10 机器头部卡死(欧式家具卡死)
}escStep_t;

//脱困变量定义
typedef struct{
 
    escapeAction_t escActState; //脱困动作状态机
    /*
    脱困动作步骤说明：（所有步骤的切换都在后退中，既一套动作尝试后，都会后退）
        检测到脱困首先是后退，之后是一连串的躁动
        1：根据脱困前记录的动作，进行尝试动作，速度设置比较低，通过角速度来快速判定动作是否有效无效的后快速切换到另外的动作
            ，左右自旋或者左后右后尝试两次相同的动作。比如两次左后退，两次速度和时间都不一样，再后退一次。
        2：如果1尝试失败，第一个动作避开1的尝试动作，1尝试了左后退，那就右后退，再左自旋，右自旋
    */
    int8_t OverheadTryCnt;//判定为架空脱困尝试架空动作次数
    
    /*escPreTypeRec 通过脱困前后的数据，判定机器困住的类型，作为脱困的部分依据 
      0x2x:在风扇座上架空  
      0x4x:机器后面翘起，卡住在欧式家具底下了 0x40机器头部被压下 0x41左前被压下，0x42右前被压下
    */
    int8_t escPreTypeRec;
    int8_t escActStepRecord;//记录当前只执行的是第几个步骤
    int8_t frontCnt;//前进 动作次数计数
    int8_t behindCnt;//后退 动作次数计数
    int8_t leftSpinCnt;//左自旋 动作次数计数
    int8_t rightSpinCnt;//右自旋 动作次数计数
    int8_t leftBehindCnt;//左后 动作次数计数
    int8_t rightBehindCnt;//右后 动作次数计数
    int8_t escActValidCnt;//脱困动作有效计数
    int8_t enterStep0PreStep;//记录是哪一步切入到后退的，不步骤切入后退，后退处理不一样
    int8_t enterSuccPreStep;//记录是哪一步切入成功的，不步骤切入 ，处理不一样
    int8_t enterStep0Sta;//记录是 动作成功还是失败切入的后退，后退处理不一样
    int8_t escActExecSta;//动作执行情况
    int8_t actionRetDelay;   //动作结果延时信号
    int8_t escSuccessOrFailure;//脱困最终的结果失败或者成功
    int8_t ciffDealCnt;//记录探地处理的次数
    int8_t currOutActionCnt;//电流超出阈值动作计数
    int8_t escActionTryCnt;//动作尝试第几次计数
    int8_t groundPeneDiff;//探地左右探地与前面探地差值较小标识
    int8_t stepContinuNum;//一套动作持续执行次数
    int8_t enterSuccessCnt;//进入脱困成功，自旋失败计数
    int16_t escUnmovableRateCnt;//角速度不可移动计数
    int16_t escUnmovableCurCnt;//电流不可移动计数
    int16_t escAngleMaxChange;//脱困时最大的变化角度
    float escPre10PrePitRol;//记录判定为卡住前10s的机器姿态
    float escPre10PrePit;//记录判定为卡住前10s的机器姿态
    float escPre10PreRol;//记录判定为卡住前10s的机器姿态
    float escStopPrePitRol;//记录判定为卡住时，机器停止时的机器姿态
    float escActionPrePit;  //记录在执行脱困动作前的俯仰数据
    float escActionPreRol; //记录在执行脱困动作前的横滚数据
    long long escActExecuTime;  //动作执行时间
    int32_t leftAngVelAdd;  //左边角速度累加
    int32_t rightAngVelAdd;//右边角速度累加
    int32_t escInterval;//脱困的时间间隔
    int32_t escActPreAngle;//动作开始时的角度
}escapeActionPara_t;

//脱困检测变量定义
typedef struct {
      int16_t angVelLessThreshCnt;//角速度异常计数
      int16_t lastspdDiffThresh;//上一次的角速度阈值
      int16_t arryAvgCurCountL;//滑动平均值数组中元素个数
      int16_t arryAvgCurCountR;//滑动平均值数组中元素个数
      int32_t avgLCurIncrSum; //电流平均值增加总和
      int32_t avgRCurIncrSum; //电流平均值增加总和
  }stuckCheckPara_t;

  typedef enum {
      ESCAPE_STAT_NONE = 0,
      ESCAPE_STAT_PREPARE,                // 准备
      ESCAPE_STAT_START,                  // 开始
      ESCAPE_STAT_ING,                    // 脱困动作执行中
      ESCAPE_STAT_RECOVER,                // 恢复脱困前状态
      ESCAPE_STAT_END,                    // 脱困动作完成
      ESCAPE_STAT_FAIL,                   // 脱困失败处理
      ESCAPE_STAT_TOTAL,
  } escape_mode_stat;                     // 脱困处理流程

  //两轮状态
  typedef struct wheelState
  {
        WHEELSTATE wheelState;
        int wheelLeftState;     //WHEEL_FRONT: 1  STOP: 0  WHEEL_BEHIND: -1
        int wheelRightState;
  }wheelState_t;




  static int lastWheelSta;
  static bool curOutFlagL = false;
  static bool curOutFlagR = false;


  #define STUCK_GO_TIME 200
  #define STUCK_TURN_TIME 200
  #define STUCK_WALL_TIME 200
  #define SIDEBRUSHELECTRICITYINDEX 30
  #define SIDEBRUSHELECTRICTYMAX  400
   
  enum TYPE
  {
    nothing,
    stucks,
    smallarea,
    windcolumn,
  };

  struct Trouble
  {
    enum TYPE type;
    Grid aimPoint;
    mapRange _maprange;
    vector <Grid> rollArray;
  };
  
  enum StuckType
  {
    NOTHTING,
    ESCAPE_EVENT_TYPE_LEFT_OFF,
    ESCAPE_EVENT_TYPE_STUCK
  };
  typedef struct SIDEBRUSH_ERROR_MODE
  {
      uint8_t sideCurrentErr;                  //边刷一级阈值错误
      uint8_t sideLeftCurrentErr;                  //边刷二级阈值错误
      uint8_t sideRightCurrentErr;                //边刷三级阈值错误
      
      uint8_t sideCurrentCountOne;                //边刷一级阈值错误计时
      uint8_t sideCurrentCountTwo;                //边刷二级阈值错误计时
      uint8_t sideCurrentCountThree;              //边刷三级阈值错误计时
      uint8_t sideUpCount;                        //边刷加力脱困计数
      
      uint16_t leftSideCurrentNow;                //当前左刷电流阈值
      uint16_t rightSideCurrentNow;               //当前右刷电流阈值
      
  }sideCheck_t;
  long long getTime();    //获取当前时间 单位:ms

  class EscapePlan
  {
    private:
        vector <Grid> wallPoint;
        vector <Grid> wallAgg;
        int rollsign;
        int stuckTime1;
        int stuckTime2;
        int stuckTime3;
        Grid stuck;
        
        Trouble trouble;
        chassisBase chassisEscape;
        float LeftOffStartX = 0, LeftOffStartY = 0, LeftOffDistance = 0;
        int LeftOffStartAngle = 0;
        bool recordPosFlag = 0;
    public:
        StuckType stuckType;
        long long record_time = 0;
        std::vector<float> recordPit;
        std::vector<float> recordRol;
        std::vector<float> recordPitRol;
        Sensor escapSensor;
        Grid escapGrid;
        long long escStartTime;   //脱困开始时间
        int enterAddAngle;      //进入脱困前的陀螺仪累加角
        float enterGyroAngle;   //进入脱困当前的朝向
        int beforeEscState; //进入脱困前的状态
        int escEnterIntervalCnt;    //记录进入短时间脱困的次数
        int prePit_10s,preRol_10s;  //记录判定为卡住前10s的俯仰 横滚
        int beforeEscPit,beforeEscRol;  //记录执行脱困动作前的俯仰 横滚
        int escbrushcnt = 0;
        sideCheck_t sideCheck;
        bool side_abnomal_index = false;
        bool side_brush_alert = false;
    public:
        EscapePlan(/* args */);
        ~EscapePlan();
        Trouble EscapeRecognition(Sensor sensor,Grid cur);
        void EscapeSmallArea();
        void Init();

        /*
        * 右轮电流滑动平均值
        */
        int r_WheelCurSlidingAvg();

        /*
        * 左轮电流滑动平均值
        */
        int l_WheelCurSlidingAvg();

        /*
        * 加速度滑动平均值
        */
        int gyroAccSlidingAvg();

        /*
        * 俯仰和横滚滑动平均值
        * 机器   pit 前面抬起 负数 后面抬起 正数
        *        rol 右边抬起 负数 左边抬起 正数
        */
        float gyroPitRolSlidingAvg();
        //边刷电流检测
        void SideBrushElectricCheck();
        /*
        * 电流异常检测
        */
        bool abnormalElectricCheck();

        /*
        * 俯仰和横滚异常判定
        * return TRUE 异常
        * return FALSE 正常
        */
        bool abnormalPitRolCheck();

        /*
        * 角速度异常判定
        * modePara 0正常跑机 1脱困
        * return TRUE 卡住
        * return FALSE 未卡住
        */
        bool abnormalAngVelocCheck(int mode_t);

        //获取俯仰 横滚值
        float getPitRol(int num);

        /*
        *获取轮子信息
        */
        wheelState_t getWheelControlState();

        /*
        *获取轮子状态
        */
        int getWheelState();

        /*
        *获取加速度信息
        */
        int getAcc(int num);

        /*
        *获取角速度信息
        */
        int getAngleRate();

        /*
        *获取累加角度
        */
        int getAddAngle();

        /*
        *记录俯仰横滚
        */
        int recordPitRol_10s();

        /*
        * 脱困动作监控陀螺仪数据用于判定是否脱困成功
        * return -1 动作无效
        * return 0  判定中
        * return 1  动作有效
        */
        int monitorIsEscapeSuccess();

        /*
        * 脱困动作电流和角速度监控
        */
        void escActCanMoveCheck();

        /*
        * 根据不同的卡死情况，对后退是否成功进行判定
        */
        bool  escActStepBehind();

        /*
        * 脱困获取相对变化俯仰横滚角度
        */
        float getMEMSRelatAngle(int index);

        /*
        * 进入脱困前和执行脱困动作后的差值
        */
        float getMEMSActChangAngle(int index);

        /*
        * 机器卡死判定
        */
        bool ESCAPE_StuckTrigCheck_My_Test();

        void ESCAPE_LeftOffCheckInit();

        bool ESCAPE_LeftFloorCheck();

        /*
        * 执行脱困动作前对陀螺仪数据分析
        * 分析是哪种卡住，进入不同的脱困动作
        */
        void escStepPreGyroAnal();

        int8_t overheadCheck(float tmpDiff);

        /*
        * 卡死脱困的个套动作分解
        * 0       正在执行动作
        *-1      判定脱困失败
        * 1       判定脱困成功
        * 2       判定脱困成功，并且需要额外处理脱困后状态恢复
        */
        bool escDeal(int state);

        void escActStep_0();

        void escActStep_1();
        
        void escActStep1WaitTmp(int cnt, int lastAct);

        void escActStep_2();

        void escActStep2WaitTmp(int failedAct);

        void escActStep_3();

        void escActStep3WaitTmp(int cnt,int lastAct);

        void escActStep_4();

        void escActStep4WaitTmp();

        // 架空动作 之后的判定中间操作
        int escStepOverheadWait(float tmpDiff);

        int judgeActionOverHead(float tmpDiff);
        int judgeActionStuckHead(float tmpDiff);
        /*
        * 脱困动作之架空脱困尝试
        */
        void escStepOverHead();
        
        /*
        * 机器头部卡死脱困动作(以欧式家具为参考) ESC_STEP_STUCK_HEAD
        */
        void escActStepStuckTheHead();

        /*
        * 头部卡住动作执行等待操作 (欧式家具)
        * tmpDiff 俯仰横滚和10s前的差值
        */
        void stuckActHeadWait(float tmpDiff);

        /*
        *脱困失败
        */
        void escActStepFailed();

        /*
        *脱困成功
        */
        void escActStepSuccess();    

        /*
        * 记录脱困前的动作
        */
        void recordActBeforeEsc();

        /************脱困主要动作**************/
        //按距离退后
        void wheelBackDist(int speed, int dis);

        //按时间前进退后
        void wheelCtrlStraight(int speed,int walkTime);

        //自旋                                         //绝对 相对角度
        void escSpin(int speed, int dir, float angle, int relatAbs);
    
        //单边旋
        void singleRotate(int speedL, int speedR, float angle, int relatAbs);
  };
       
}
#endif
