/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-10 15:58:33
 * @Project      : UM_path_planning
 */


#include "navigation_algorithm/AlongWall.h"
#include "common_function/ExceptionHanding.h"
#define WALL_SPEED 180
#define WHEEL_SLOW_INDEX 0.4 
#define error_index 4
#define WALL_NORMALVALUE 2900
#define BLACK_WALL  1200
#define BACK_TIME 25
#define SPEED_COEFFIC 2.48333f
using namespace useerobot;
EscapePlan escape;
extern useerobot::Maze _maze;
extern float gyo_angle_;
namespace useerobot
{  

    extern SmallArea smallArea;

    bool alongwalk_run_index =false;
    static int backTime = BACK_TIME;
    bool first_flag = true;
    chassisBase tmpchass;
    bool firstAlongWall = false;;
    static Alongwallstate_t wallstate_t;
    static int lastValue = 0;
    static int minValueCnt = 0;
    static int maxValueCnt = 0;
    static int adjustTurnCnt = 0;
    static int lastObs = 0;
    static WHEELSTATE wheelSta;
    static int maxValue = 0;
    static int maxObs = 0;
    static int smallPlaceStartAngle = 0;
    static int smallPlaceFlag = 0, smallPlaceTurnCnt = 0; 
    static int enterSmallPlaceFlag = 0;
    static long long smallPlaceCnt = 0;
    static float FanBaseDownPit = 0.0f, FanBaseDownRol = 0.0f,FanBaseUpPit = 0.0f, FanBaseUpRol = 0.0f,FanBaseInitPit = 0.0f;
    static int FanBaseEscapeCnt = 0,FanBaseEscapeFlag = 0;
    WallFallowPara_t WallFallowPara;
    AlongWall::AlongWall(/* args */)
    {
    }
    
    AlongWall::~AlongWall()
    {
    }

    bool AlongWall::wallBack()
    {
        while(backTime &&(GLOBAL_CONTROL != WHEEL_STOP))
        {
            backTime --;   
            chassis.chassisSpeed(-150,-150,1);
            return 0;
        }
        chassis.chassisSpeed(0,0,1);
        return 1;
    }
    bool AlongWall::escapeWallBack()
    {
        while(backTime &&(GLOBAL_CONTROL != WHEEL_STOP))
        {
            backTime --;   
            chassis.chassisSpeed(-300,-300,1);
            return 0;
        }
        chassis.chassisSpeed(0,0,1);
        return 1;
    }
    long long getCurrentTime()
    {
    struct timeval time;

    gettimeofday(&time,NULL);

    return (long long)time.tv_sec*1000 + (long long)time.tv_usec/1000;
    }

    void StartWallFollow(WallFollowModel_t Wall_Follow_model, WallFollowDir_t dir, SPEED_t speed)
    {
        if((WallFallowPara.Model == Wall_Follow_model)&&(WallFallowPara.WallFollowRunState == WallFollow_Run))//防止反复调用
        {
            WallFallowPara.Speed = speed;
            return;
        }
        FRIZY_LOG(LOG_INFO,"StartWallFollow");
        

        WallFallowPara.TurnErrCnt = 0;
        memset(&WallFallowPara,0,sizeof(WallFallowPara_t));//复位沿墙参数
        // VTurnFlag = 0;
        // First5253Flag = 0;
        WallFallowPara.Speed = speed;
        
        WallFallowPara.Model = Wall_Follow_model;          //沿墙模式赋值
        WallFallowPara.Dir = dir;
        WallFallowPara.WallFollowRunState = WallFollow_Run;
        WallFallowPara.State = WallFollowAlong;  
        if(WallFallowPara.Dir == LEFTAW)
            wallstate_t = LEFT_WALL;
        else if(WallFallowPara.Dir == RIGHTAW)
            wallstate_t = RIGHT_WALL;
        WallFallowPara.Dis = DEFALUT_WALL_VAL;
        WallFallowPara.BumpFlag = 0;
        WallFallowPara.BumpDealState = BumpNoAction;
        alongwalk_run_index = true;
        firstAlongWall = true;
        first_flag = true;
        backTime = BACK_TIME;
        smallPlaceFlag = 0;
        enterSmallPlaceFlag = 0;
        // WallFallowPara.LastAddAngle = DRIV_AddAngleGetNoReset()/10;
        // WallFallowPara.Dis = DEFALUT_WALL_VAL;
        // WallFallowPara.ShiJian = xTaskGetTickCount();
    }

    Alongwallstate_t IsWall()
    {
        if (wallstate_t == EXIT_WALL)
            return EXIT_WALL;
        else if (wallstate_t == LEFT_WALL)
            return LEFT_WALL;
        else if (wallstate_t == RIGHT_WALL)
            return RIGHT_WALL;
    }

    WallFollowDir_t getAlongWallDir()
    {
        return WallFallowPara.Dir;
    }

    //停止沿墙
    void StopWallFollow()
    {

        // if(WallFallowPara.WallFollowRunState == WallFollow_Stop)//防止反复调用
        // {

        //     return;
        // }
        tmpchass.chassisSpeed(0,0,1);
        wallstate_t = EXIT_WALL;
        alongwalk_run_index = false;
        WallFallowPara.WallFollowRunState = WallFollow_Stop;
        WallFallowPara.StopFlag = 1;
        FRIZY_LOG(LOG_INFO, "Stop Wall Follow");
    }

    //继续沿墙
    void ContinueWallFollow(void)
    {
        if(WallFollow_Pause != WallFallowPara.WallFollowRunState)//如果前面没调用暂停，这个程序就不执行
        {
            return;
        }
        WallFallowPara.WallFollowRunState = WallFollow_Run;
        WallFallowPara.State = WallFallowPara.PauseState;
    }
    //暂停沿墙
    void PauseWallFollow(void)
    {
        if(WallFollow_Run != WallFallowPara.WallFollowRunState)     //暂停状态和非沿墙状态下
        {
            return;
        }
        WallFallowPara.PauseState = WallFallowPara.State;
        WallFallowPara.PauseFlag = 1;
    }

    int AlongWall::getObsFront()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.midOmnibearingOn - currentSensor.midOmnibearingOff;
    }

    int AlongWall::getObsLeft()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.leftOmnibearingOn - currentSensor.leftOmnibearingOff;
    }
    
    int AlongWall::getObsRight()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.rightOmnibearingOn - currentSensor.rightOmnibearingOff;
    }

    int AlongWall::getAddAngle()
    {
        // chassis.GetSensor(&currentSensor);
        return currentSensor.addAngle;
    }
    void AlongWall::fanBaseEscapeFlagClear()
    {
        FanBaseUpPit = 0;
        FanBaseUpRol = 0;
        FanBaseDownPit = 0;
        FanBaseDownRol = 0;
        FanBaseEscapeCnt = 0;
        FanBaseEscapeFlag = 0;
        FanBaseInitPit = 0;
    }
    void AlongWall::fanBaseEscapeDiscern()
    {
        if(FanBaseDownPit - FanBaseInitPit < -5.0f && FanBaseUpPit - FanBaseInitPit < -5.0f)
        {
            if(abs(FanBaseUpPit - FanBaseDownPit) < 3.0f)
            {
                FRIZY_LOG(LOG_DEBUG, "abs(FanBaseUpPit - FanBaseDownPit) < 3.0f");
                if(FanBaseDownRol - FanBaseUpRol > 2.5f)
                {
                    FRIZY_LOG(LOG_DEBUG, "FanBaseDownRol - FanBaseUpRol > 2.5f  %d",FanBaseEscapeCnt);
                    FanBaseEscapeCnt ++;
                    if(FanBaseEscapeCnt > 5)
                    {
                        FanBaseEscapeCnt = 0;
                        FanBaseEscapeFlag = 1;
                        FRIZY_LOG(LOG_DEBUG, "FanBase Escaps OK");
                    }
                }
                else
                {
                    if(FanBaseEscapeCnt)
                    {
                        FanBaseEscapeCnt --;
                    }
                }
            }
            else
            {
                if(FanBaseEscapeCnt)
                {
                    FanBaseEscapeCnt --;
                }
            }
        }
        else
        {
            fanBaseEscapeFlagClear();
        }
    }

    int AlongWall::recognizeBlackWall()
    {
        adjustTurnCnt ++;
        // chassis.GetSensor(&currentSensor);
        int angle = currentSensor.addAngle;
        // FRIZY_LOG(LOG_DEBUG, "currentSensor.addAngle:%d",currentSensor.addAngle);
        if(WallFallowPara.Dir == LEFTAW)
        {
            if(currentSensor.leftAlongWall > lastValue && currentSensor.leftAlongWall > 100)
            {
                if(minValueCnt)
                    minValueCnt--;
                if(currentSensor.leftAlongWall > maxValue)
                {
                    maxValueCnt++;
                    maxValue = currentSensor.leftAlongWall;
                }
            }
            else 
                minValueCnt++;
            lastValue = currentSensor.leftAlongWall;
            int leftObs = getObsLeft();
            if(leftObs > lastObs && leftObs > 200)
            {
                if(leftObs > maxObs)
                    maxObs = leftObs;
                // FRIZY_LOG(LOG_DEBUG, "leftobs:%d, lastobs:%d, maxobs:%d", leftObs, lastObs, maxObs);
            }
            lastObs = leftObs;
        }
        else if(WallFallowPara.Dir == RIGHTAW)
        {
            FRIZY_LOG(LOG_DEBUG, "currentSensor.rightAlongWall:%d, lastvalue:%d, maxVal:%d", currentSensor.rightAlongWall, lastValue, maxValue);
            if(currentSensor.rightAlongWall > lastValue && currentSensor.rightAlongWall > 100)
            {
                if(minValueCnt)
                    minValueCnt--;
                if(currentSensor.rightAlongWall > maxValue)
                {
                    maxValueCnt++;
                    maxValue = currentSensor.rightAlongWall;
                }
                FRIZY_LOG(LOG_DEBUG, "maxVal:%d", maxValue);
            }
            else 
                minValueCnt++;
            lastValue = currentSensor.rightAlongWall;
            int rightObs = getObsRight();
            FRIZY_LOG(LOG_DEBUG, "1.rightobs:%d, lastobs:%d, maxobs:%d", rightObs, lastObs, maxObs);
            if(rightObs > lastObs && rightObs > 200)
            {
                if(rightObs > maxObs)
                    maxObs = rightObs;
            }
            lastObs = rightObs;
            FRIZY_LOG(LOG_DEBUG, "2.rightobs:%d, lastobs:%d, maxobs:%d", rightObs, lastObs, maxObs);
        }
        FRIZY_LOG(LOG_DEBUG, "maxValueCnt:%d, minValueCnt:%d, adjustTurnCnt:%d", maxValueCnt, minValueCnt, adjustTurnCnt);
        FRIZY_LOG(LOG_DEBUG, "angle/10:%d, wallpara.starturn:%d, 1-2:%d",angle/10, WallFallowPara.StartTrunAngle, (angle / 10) - WallFallowPara.StartTrunAngle);
        if(maxValueCnt > 3/*minValueCnt > 2*/ && adjustTurnCnt >= 10 &&
        ((WallFallowPara.Dir == LEFTAW && ((angle / 10) - WallFallowPara.StartTrunAngle >= 30)) ||
         (WallFallowPara.Dir == RIGHTAW && ((angle / 10) - WallFallowPara.StartTrunAngle <= -30))))
        {
            FanBaseDownPit = escape.getPitRol(0);
            FanBaseDownRol = escape.getPitRol(1);
            fanBaseEscapeDiscern();
            WallFallowPara.BumpCnt = 0;
            maxValueCnt = 0;
            // minValueCnt = 0;
            FRIZY_LOG(LOG_DEBUG, "maxValue:%d, maxObs:%d", maxValue, maxObs);
            FRIZY_LOG(LOG_DEBUG, "maxValue * 95 /100:%d", maxValue * 95 /100);
            if(maxValue * 95 /100 >= 1000)
            {
                if(maxObs < 800 && maxObs > 200)
                {
                    // isBlackWall = 1;
                    WallFallowPara.Dis = BLACK_WALL;
                }
                else
                {
                    // isBlackWall = 0;
                    // WallFallowPara.Dis = maxValue * 95 / 100;
                    if(WallFallowPara.Dis > 3000)
                        WallFallowPara.Dis = 3000;
                    else 
                        WallFallowPara.Dis = WALL_NORMALVALUE;
                }
                FRIZY_LOG(LOG_DEBUG, "WALL DIS:%d", WallFallowPara.Dis);
            }
            else
            {
                WallFallowPara.Dis = WALL_NORMALVALUE;
                FRIZY_LOG(LOG_DEBUG, "WALL DIS:%d", WALL_NORMALVALUE);
                WallFallowPara.ValueErrStartTrunAngle = angle / 10;
            }
            adjustTurnCnt = 0;
            // WallFallowPara.BumpDealState = WaitTurn;
            WallFallowPara.BumpCnt = 0;
            WallFallowPara.TurnErrCnt = 0;
            return 1;
        }
        return 0;
    }

    int AlongWall::wallObsFront()
    {
        static float lastObsFront = 0;
        static float obsFront = 0;
        static uint32_t hshObsFrontCount = 0;
        float slopeObs = 0;
        static float lastSlopeObs = 0;
        static uint16_t slopeObsCount = 0;
        static uint8_t slopeObsFlag = 0;
        static uint8_t whiteFlag = 0;
        static uint8_t whiteCount = 0;
        
        // chassis.GetSensor(&currentSensor);
        if(WallFallowPara.State != WallFollowAlong)
        {
            hshObsFrontCount = 0;
            lastObsFront = 0;
            obsFront = 0;
            slopeObsCount = 0;
            slopeObsFlag = 0;
            lastSlopeObs = 0;
            whiteFlag = 0;
            whiteCount = 0;
            return 0;
        }
        hshObsFrontCount ++;
        obsFront += getObsFront();
        // FRIZY_LOG(LOG_DEBUG,"hshObsFrontCount:%d,obsFront += getObsFront():%f",hshObsFrontCount,obsFront);
        if(hshObsFrontCount > 20 / 20)
        {
            hshObsFrontCount = 0;
            if(lastObsFront != 0 && whiteFlag == 0)
            {
                obsFront /= (20 / 20) + 1;
                slopeObs = obsFront / lastObsFront;
                // FRIZY_LOG(LOG_DEBUG,"obsFront /= (20 / 20) + 1:%f,slopeObs:%f",obsFront,slopeObs);
                if(slopeObs > 1.02f)
                {
                    slopeObsCount ++;
                    // FRIZY_LOG(LOG_DEBUG,"slopeObsCount:%u",slopeObsCount);
                    if(slopeObsCount > 10)
                    {
                        slopeObsCount = 10;
                        slopeObsFlag = 1;
                    }
                }
                else
                {
                    if(slopeObsCount > 0)
                        slopeObsCount --;
                }
                if(slopeObsFlag == 1)
                {
                    // FRIZY_LOG(LOG_DEBUG,"slopeObsFlag == 1");
                    if(currentSensor.midOmnibearingTurn == 1)
                    {
                        // FRIZY_LOG(LOG_DEBUG,"currentSensor.midOmnibearingTurn");
                        if(slopeObs > 1.2f)
                        {
                            // FRIZY_LOG(LOG_DEBUG,"slopeObs > 1.2f");
                            slopeObsCount = 0;
                            slopeObsFlag = 0;
                            // FRIZY_LOG(LOG_DEBUG, "OBSFRONT OK--1");
                            // FRIZY_LOG(LOG_DEBUG, "slopeObs:%f,obsFront:%f,lastObsFront:%f,slopeObsCount:%d,slopeObsFlag:%d",slopeObs,obsFront,lastObsFront,slopeObsCount,slopeObsFlag);
                            return 1;
                        }
                        else if(lastSlopeObs > slopeObs && whiteFlag == 0)
                        {
                            // FRIZY_LOG(LOG_DEBUG,"lastSlopeObs > slopeObs && whiteFlag == 0");
                            whiteFlag = 1;
                            whiteCount = 0;
                        }
                    }
                    else
                    {
                        whiteFlag = 0;
                        whiteCount = 0;
                    }

                }
                lastSlopeObs = slopeObs;
            }
            else if(whiteFlag == 1)
            {
                whiteCount ++;
                // FRIZY_LOG(LOG_DEBUG,"whiteCount:%d\n",whiteCount);
                if(whiteCount > 1)
                {
                    whiteCount = 0;
                    whiteFlag = 0;
                    slopeObsCount = 0;
                    slopeObsFlag = 0;
                    FRIZY_LOG(LOG_DEBUG,"OBSFRONT OK--1");
                    return 1;
                }
            }
            
            lastObsFront = obsFront;
            obsFront = 0;
        }
        return 0;
    }

    void AlongWall::bumpScan()
    {
        if(WallFallowPara.WallFollowRunState != WallFollow_Run)//不沿墙不做动作
        {
            return;
        }
        uint8_t ObsState = 0;
        WHEELSTATE wheel_sta;
        ObsState = wallObsFront();
        // chassis.GetSensor(&currentSensor);
        wheel_sta = chassis.getWheelState();
        if(wheel_sta != WHEELFRONT)
        {
            currentSensor.midOmnibearingSlow = 0;
            currentSensor.midOmnibearingTurn = 0;
        }
        // FRIZY_LOG(LOG_INFO,"OBSstate:%d, WHHEELSTA:%d", ObsState, wheel_sta);
        //禁区
        if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir == RIGHTAW && (getBanDis(0) < 0.2 || getBanDis(4) < 0.25))
        {
            WallFallowPara.BumpRecord = ForbidFront;
            WallFallowPara.BumpState = ForbidFront;
        }
        else if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir == LEFTAW && (getBanDis(0) < 0.2 || getBanDis(3) < 0.25))
        {
            WallFallowPara.BumpRecord = ForbidFront;
            WallFallowPara.BumpState = ForbidFront;
        }
        else if(WallFallowPara.State == WallFollowForbid && WallFallowPara.Dir == RIGHTAW && (/*getBanDis(1) < 0.06 || */getBanDis(0) < 0.18 || getBanDis(3) < 0.15))
        {
            WallFallowPara.BumpRecord = ForbidRight;
            WallFallowPara.BumpState = ForbidRight;
        }
        else if(WallFallowPara.State == WallFollowForbid && WallFallowPara.Dir == LEFTAW && (/*getBanDis(2) < 0.06 || */getBanDis(0) < 0.18 || getBanDis(4) < 0.15))
        {
            WallFallowPara.BumpRecord = ForbidLeft;
            WallFallowPara.BumpState = ForbidLeft;
        }
        //全方位中
        else if(WallFallowPara.State == WallFollowAlong && ObsState == 1 && wheel_sta == WHEELFRONT)
        {
            WallFallowPara.BumpRecord = OBSFront;
            WallFallowPara.BumpState = OBSFront;
        }
        else if(currentSensor.leftCliff)
        {
            WallFallowPara.BumpRecord = CliffLeft;
            WallFallowPara.BumpState = CliffLeft;
        }
        else if(currentSensor.rightCliff)
        {
            WallFallowPara.BumpRecord = CliffRight;
            WallFallowPara.BumpState = CliffRight;
        }
        else if(currentSensor.midCliff)
        {
            WallFallowPara.BumpRecord = CliffInter;
            WallFallowPara.BumpState = CliffInter;
        }
        else if(currentSensor.bump)
        {
            WallFallowPara.BumpRecord = BumpInter;
            WallFallowPara.BumpState = BumpInter;
        }
        // //虚拟墙右
        // else if(currentSensor.rightVir)
        // {
        //     WallFallowPara.BumpRecord = MagVirtualRight;
        //     WallFallowPara.BumpState = MagVirtualRight;
        // }
        // //虚拟墙左
        // else if(currentSensor.leftVir)
        // {
        //     WallFallowPara.BumpRecord = MagVirtualLeft;
        //     WallFallowPara.BumpState = MagVirtualLeft;
        // }
        // //虚拟墙中
        // else if(currentSensor.leftFrontVir || currentSensor.rightFrontVir)
        // {
        //     WallFallowPara.BumpRecord = MagVirtualInter;
        //     WallFallowPara.BumpState = MagVirtualInter;
        // }
        // //全方位左
        // else if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir != LEFTAW && currentSensor.obs/*全方位转向*/
        //     &&  getObsLeft() > getObsFront() && getObsLeft() > getObsRight())
        // {
        //     WallFallowPara.BumpRecord = OBSLeft;
        //     WallFallowPara.BumpState = OBSLeft;
        // }
        // //全方位右
        // else if(WallFallowPara.State == WallFollowAlong && WallFallowPara.Dir != RIGHTAW && currentSensor.obs
        //     &&  getObsRight() > getObsFront() && getObsRight() > getObsLeft())
        // {
        //     WallFallowPara.BumpRecord = OBSRight;
        //     WallFallowPara.BumpState = OBSRight;
        // }
        
        else if (((WallFallowPara.BumpDealState < BumpSignalBack) || (WallFallowPara.BumpDealState > Virtual5253Turn)) &&
                 (WallFallowPara.Model != DockWallFollow) &&
                 !WallFallowPara.VTurnCnt)
                //  sqrtf((((current_pos.realx * 15 / 100) - 0) * ((current_pos.realx * 15 / 100) - 0)) + (((current_pos.realy * 15 / 100) - 0) * ((current_pos.realy * 15 / 100) - 0))) > 2)
        {
            // FRIZY_LOG(LOG_DEBUG, "check 41 singal");
            if(currentSensor.leftInfrared)//左侧检测出回充虚拟墙信号
            {
                FRIZY_LOG(LOG_DEBUG, "leftInfrared signal");
                if(WallFallowPara.Dir != RIGHTAW)   //右沿墙不处理左侧回充座虚拟墙
                {
                    // WallFallowPara.Dir = LEFTAW;                      //包含无墙可能性
                    WallFallowPara.LastAddAngle = getAddAngle() / 10;
                    WallFallowPara.BumpRecord = InsideVirtual;
                    WallFallowPara.BumpState = InsideVirtual;
                    FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord = InsideVirtual");
                }
            }
            else if(currentSensor.rightInfrared)
            {
                FRIZY_LOG(LOG_DEBUG, "rightInfrared signal");
                if(WallFallowPara.Dir != LEFTAW)    //左侧沿墙不处理右侧回充座虚拟墙  
                {
                    WallFallowPara.BumpRecord = InsideVirtual;
                    WallFallowPara.BumpState = InsideVirtual;
                    FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord = InsideVirtual");
                }
            }
            else if(currentSensor.leftFrontInfrared || currentSensor.rightFrontInfrared)    //前方检测出回充虚拟墙信号
            {
                FRIZY_LOG(LOG_DEBUG, "frontInfrared signal");
                WallFallowPara.BumpRecord = FrontVirtual;
                WallFallowPara.BumpState = FrontVirtual;
            }
            else 
            {
                WallFallowPara.BumpState = BumpNo;
            }
        }
        else 
        {
            WallFallowPara.BumpState = BumpNo;
        }
        if(WallFallowPara.BumpState == ForbidFront || WallFallowPara.BumpState == ForbidLeft || WallFallowPara.BumpState == ForbidRight)
        {
            chassis.chassisSpeed(0, 0, 1);
            if(WallFallowPara.BumpState == ForbidFront)
            {
                WallFallowPara.State = WallFollowForbid;
            }
            
            WallFallowPara.BumpDealState = BumpSignalBack;
            WallFallowPara.BumpFlag = 1;
        }
        else if(WallFallowPara.BumpState == OBSFront)
        {
            // if((WallFallowPara.BumpDealState > WaitBack)||(WallFallowPara.BumpDealState < BumpSignalBack))
            // {               
                if(currentSensor.magnVirWall || currentSensor.cliff)
                    chassis.chassisSpeed(0, 0, 1);
                else 
                {
                    // FRIZY_LOG(LOG_DEBUG, "wheel stop");
                    chassis.wheelCtrlStop();
                }
                WallFallowPara.Dis = DEFALUT_WALL_VAL;
                WallFallowPara.BumpDealState = Turn;
                WallFallowPara.BumpFlag = 1;
                if(WallFallowPara.State == WallFollowVIRTUAL)
                {
                    WallFallowPara.VTurnCnt = 0;
                }
            // }
            WallFallowPara.FanEscapeFlag = 0;
            WallFallowPara.AloneTime = 0; 
            WallFallowPara.BumpCnt = 0;
            WallFallowPara.NoWallCnt = 0; 
        }
        else if(currentSensor.bump /*|| currentSensor.magnVirWall*/ || currentSensor.cliff)
        {
            // if((WallFallowPara.BumpDealState > WaitBack)||(WallFallowPara.BumpDealState < BumpSignalBack))
            // {
                if(/*currentSensor.magnVirWall ||*/ currentSensor.bump || currentSensor.cliff)          
                {      
                    FRIZY_LOG(LOG_DEBUG, "bump or cliff signal brake");
                    chassis.chassisSpeed(0, 0 ,1);
                    WallFallowPara.BumpDealState = BumpSignalBack;
                    WallFallowPara.BumpFlag = 1;
                }
            // }
            WallFallowPara.FanEscapeFlag = 0;
            WallFallowPara.AloneTime = 0; 
            WallFallowPara.BumpCnt = 0;
            WallFallowPara.NoWallCnt = 0; 
            if(currentSensor.bump && WallFallowPara.State == WallFollowForbid)
            {
                if(WallFallowPara.Dir == LEFTAW)
                {
                    if(getBanDis(2) > 0.3 && getBanDis(0) > 0.3)
                    {
                        WallFallowPara.BumpDealState = BumpSignalBack;
                        WallFallowPara.BumpFlag = 1;
                        WallFallowPara.State = WallFollowAlong;
                    }
                    else
                    {
                        WallFallowPara.BumpDealState = Turn;
                        WallFallowPara.BumpFlag = 1;
                    }
                }
                else if(WallFallowPara.Dir == RIGHTAW)
                {
                    if(getBanDis(1) > 0.3 && getBanDis(0) > 0.3)
                    {
                        WallFallowPara.BumpDealState = BumpSignalBack;
                        WallFallowPara.BumpFlag = 1;
                        WallFallowPara.State = WallFollowAlong;
                    }
                    else
                    {
                        WallFallowPara.BumpDealState = Turn;
                        WallFallowPara.BumpFlag = 1;
                    }
                }
            }
            // if(currentSensor.magnVirWall)
            // {
            //     // FRIZY_LOG(LOG_DEBUG, "识别虚拟墙信号");
            //     WallFallowPara.State = WallFollowVIRTUAL;
            //     WallFallowPara.BumpDealState = TurnWithOutVirtual;
            //     WallFallowPara.BumpFlag = 1;
            //     FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpDealState = TurnWithOutVirtual");
            // }
            if(currentSensor.bump && (!currentSensor.magnVirWall) && WallFallowPara.State == WallFollowVIRTUAL)
            {
                chassis.chassisSpeed(0, 0 ,1);
                WallFallowPara.BumpDealState = BumpSignalBack;
                WallFallowPara.BumpFlag = 1;
                WallFallowPara.State = WallFollowAlong;
                FRIZY_LOG(LOG_DEBUG, "along virtual wall finished");
            }
            if(WallFallowPara.State == WallFollowVIRTUAL)
            {
                WallFallowPara.VTurnCnt = 0;
                // WallFallowPara.BumpDealState = BumpSignalBack;
            }
        }
        else if((WallFallowPara.BumpState == FrontVirtual || WallFallowPara.BumpState == InsideVirtual)
        /*&&((WallFallowPara.BumpDealState < BumpSignalBack)||(WallFallowPara.BumpDealState > WaitBack))*/)//后退不做处理
        {
            chassis.chassisSpeed(0, 0 ,1);
            WallFallowPara.BumpDealState = TurnWithOutVirtual;
            WallFallowPara.BumpFlag = 1;
            WallFallowPara.Dis = DEFALUT_WALL_VAL;
            WallFallowPara.FanEscapeFlag = 0;
            WallFallowPara.AloneTime = 0; 
            WallFallowPara.NoWallCnt = 0; 
            WallFallowPara.BumpCnt = 0;
        }
        else if(currentSensor.midOmnibearingSlow == 1 && currentSensor.midOmnibearingTurn == 0)     // 前全方位减速处理
        {
            chassis.chassisSpeed(currentSensor.leftw * WHEEL_SLOW_INDEX, currentSensor.rightw * WHEEL_SLOW_INDEX, 1);
        }
        FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord:%d",WallFallowPara.BumpRecord);
    }

    int AlongWall::bumpDeal()
    {
        static int turnCnt = 0; /*adjustTurnCnt = 0, lastGyroState = 0;*/
        static int lastBumpDealState;
        lastBumpDealState = WallFallowPara.BumpDealState;
        // chassis.GetSensor(&currentSensor);
        FRIZY_LOG(LOG_DEBUG, "bumpDeaState:%d", WallFallowPara.BumpDealState);
        switch(WallFallowPara.BumpDealState)
        {
            case BumpSignalBack:
                if(WallFallowPara.BumpRecord > BumpInter && WallFallowPara.BumpRecord < ForbidFront)
                {
                    if(chassis.getWheelState() == WHEELSTOP && WallFallowPara.WallFollowRunState == WallFollow_Run)
                    {
                        WallFallowPara.BumpCnt = 0;
                        chassis.chassisSpeed(-150, -150, 1);
                        FRIZY_LOG(LOG_DEBUG, "send back speed");
                        WallFallowPara.BumpDealState = BackWithoutSignal;
                        backTime = BACK_TIME;
                        return 0;
                    }
                    else
                    {
                        FRIZY_LOG(LOG_DEBUG, "wheel not stop");
                        chassis.chassisSpeed(0, 0, 1);
                        return 0;
                    }
                }
                else 
                {
                    if(!wallBack())
                        return 0;
                    FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpRecord < CliffRight");
                    backTime = BACK_TIME;
                    WallFallowPara.BumpDealState = Turn;
                    return 0;
                }
            break;

            case BackWithoutSignal:
                // if(chassis.getWheelState() == WHEELBEHIND && (!currentSensor.magnVirWall || WallFallowPara.BumpState == OBSFront || !currentSensor.cliff))
                if(!currentSensor.cliff)
                {
                    WallFallowPara.BumpCnt ++;
                    if(WallFallowPara.BumpCnt > 10)
                    {
                        FRIZY_LOG(LOG_DEBUG, "no cliff signal, next step turn");
                        WallFallowPara.BumpCnt = 0;
                        chassis.chassisSpeed(0, 0, 1);
                        WallFallowPara.BumpDealState = Turn;
                        return 0;
                    }
                    else
                        return 0;
                }
                else 
                {
                    FRIZY_LOG(LOG_DEBUG, "still has cliff signal, back continue");
                    WallFallowPara.BumpCnt = 0;
                    chassis.chassisSpeed(-200, -200, 1);
                    return 0;
                    
                }
            break;

            case WaitBack:

                // FRIZY_LOG(LOG_DEBUG, "WaitBack, bump:%d",currentSensor.bump);
                if(chassis.getWheelState() == WHEELSTOP)
                {
                    WallFallowPara.BumpCnt = 0;
                    if(!currentSensor.bump)
                    {
                        backTime = BACK_TIME;
                        turnCnt ++;
                        if(turnCnt >= 3)//后退到旋转中间要间隔60ms，为路径做准备
                        {
                            turnCnt = 0;
                            WallFallowPara.BumpDealState = Turn;
                            return 0;
                        }
                        else
                            return 0;
                    }
                    // else 
                    // {
                    //     if(!wallBack())
                    //     {
                    //         return 0;
                    //     }
                    //     backTime = BACK_TIME;
                    // }
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"WaitBack chassisSpeed(0, 0, 1)");
                    chassis.chassisSpeed(0, 0, 1);
                    return 0;
                }    
            break;

            case Turn:
            {
                if(WallFallowPara.BumpRecord == ForbidFront)
                {
                    if(!rotateFlag)
                    {
                        chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        rotateFlag = 1;
                    }
                    if(!rotaterobot(recordForward, 55))
                        return 0; 
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn ForbidFront");
                }
                else if(WallFallowPara.BumpRecord == ForbidLeft || WallFallowPara.BumpRecord == ForbidRight)
                {
                    if(!rotateFlag)
                    {
                        chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        rotateFlag = 1;
                    }
                    if(!rotaterobot(recordForward, 30))
                        return 0;
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn ForbidLeft ForbidRight");
                }
                else if(WallFallowPara.BumpRecord == OBSFront)
                {   
                    if(!rotateFlag)
                    {
                        chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        rotateFlag = 1;
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 30))
                            return 0;
                    }
                    else
                    {
                        if(!rotaterobot(recordForward, 90))
                            return 0;
                    }
                    // chassis.chassisSpeed(0, 0, 1);
                    // WallFallowPara.Dis = DEFALUT_WALL_VAL;
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn OBSFront");
                }
                else if(WallFallowPara.BumpRecord == OBSRight)
                {
                    if(WallFallowPara.Dir == LEFTAW)
                    {
                        if(!rotateFlag)
                        {
                            chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(smallPlaceFlag)
                        {
                            if(!rotaterobot(recordForward, 20))
                                return 0;
                        }
                        else
                        {
                            if(!rotaterobot(recordForward, 135))
                                return 0;
                        }
                        // chassis.chassisSpeed(0, 0, 1);
                        WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                    }
                }
                else if(WallFallowPara.BumpRecord == OBSLeft)
                {
                    if(WallFallowPara.Dir == RIGHTAW)
                    {
                        if(!rotateFlag)
                        {
                            chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(smallPlaceFlag)
                        {
                            if(!rotaterobot(recordForward, 20))
                                return 0;
                        }
                        else
                        {
                            if(!rotaterobot(recordForward, 135))
                                return 0;
                        }
                        // chassis.chassisSpeed(0, 0, 1);
                        WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                    }
                }
                // else if(WallFallowPara.BumpRecord == BumpInter || WallFallowPara.BumpRecord == MagVirtualRight 
                // ||      WallFallowPara.BumpRecord == MagVirtualInter || WallFallowPara.BumpRecord == MagVirtualLeft)
                else if(WallFallowPara.BumpRecord == BumpInter)
                {
                    if(!rotateFlag)
                    {
                        chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;     
                        recordForward = (360-gyo_angle_*180/_Pi);   //gyro
                        recordTime = getCurrentTime();
                        FRIZY_LOG(LOG_DEBUG, "rotate start recordTime:%lld\n", recordTime);
                        WallFallowPara.StartTrunAngle = currentSensor.addAngle / 10;
                        // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                        rotateFlag = 1;
                        lastValue = 0;
                        minValueCnt = 0;
                        maxValueCnt = 0; 
                        adjustTurnCnt = 0;
                        lastObs = 0;
                        maxValue = 0;
                        maxObs = 0;
                        FanBaseUpPit = escape.getPitRol(0);
                        FanBaseUpRol = escape.getPitRol(1);
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 20))
                            return 0;
                    }
                    else
                    {
                        if(!rotaterobot(recordForward, 45))
                            return 0;
                    }
                    // WallFallowPara.BumpDealState = WallALongTurn;
                    // chassis.chassisSpeed(0, 0, 1);
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    backTime = BACK_TIME;
                    // back_sign =false;
                    recognize = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn BumpInter");
                }
                else if(WallFallowPara.BumpRecord == CliffInter || WallFallowPara.BumpRecord == CliffRight || WallFallowPara.BumpRecord == CliffLeft
                    ||  WallFallowPara.BumpRecord == MagVirtualInter || WallFallowPara.BumpRecord == MagVirtualRight || WallFallowPara.BumpRecord == MagVirtualLeft)
                {
                    if(!rotateFlag)
                    {
                        chassis.GridPoint(&current_pos);
                        // recordForward = current_pos.forward;
                        recordForward = (360-gyo_angle_*180/_Pi);
                        rotateFlag = 1;
                    }
                    if(smallPlaceFlag)
                    {
                        if(!rotaterobot(recordForward, 20))
                            return 0;
                    }
                    else
                    {
                        if(!rotaterobot(recordForward, 45))
                            return 0;
                    }
                    // chassis.chassisSpeed(0, 0, 1);
                    WallFallowPara.BumpFlag = 0;
                    rotateFlag = 0;
                    FRIZY_LOG(LOG_DEBUG, "Turn cliff:%d", WallFallowPara.BumpRecord);
                }
                WallFallowPara.BumpCnt = 0;
                WallFallowPara.BumpRecord = BumpNo;
                WallFallowPara.BumpDealState = BumpNoAction;
                WallFallowPara.BumpFlag = 0;
                FRIZY_LOG(LOG_DEBUG, "turn state finished:%d", WallFallowPara.BumpRecord);
            }
            break;
            
            case WaitTurn:
                if(chassis.getWheelState() == WHEELSTOP)
                {
                    WallFallowPara.BumpCnt = 0;
                    WallFallowPara.BumpDealState = BumpNoAction;
                    WallFallowPara.BumpFlag = 0;
                    WallFallowPara.BumpRecord = BumpNo;
                }
                else 
                {
                    chassis.chassisSpeed(0, 0, 1);
                    return 0;
                }
                    
            break;

            case TurnWithOutVirtual:    // 41信号处理
            {   
                if(WallFallowPara.State != WallFollowVIRTUAL)
                {
                    WallFallowPara.State = WallFollowVIRTUAL;
                    if(WallFallowPara.BumpState == InsideVirtual)
                    // if(WallFallowPara.BumpRecord == MagVirtualLeft || WallFallowPara.BumpRecord == MagVirtualRight)
                    {
                        if(!rotateFlag)
                        {
                            chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 35))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        backTime = BACK_TIME;
                        // back_sign =false;
                        WallFallowPara.State = WallFollowVIRTUAL;
                    }
                    else 
                    {
                        if(!rotateFlag)
                        {
                            chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 55))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        backTime = BACK_TIME;
                        // back_sign =false;
                    }
                    WallFallowPara.BumpDealState = WaitTurn;
                }  
                else
                {
                    if(WallFallowPara.BumpRecord == InsideVirtual)
                    // if(WallFallowPara.BumpRecord == MagVirtualLeft || WallFallowPara.BumpRecord == MagVirtualRight)
                    {
                        if(!rotateFlag)
                        {
                            chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 35))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        backTime = BACK_TIME;
                        // back_sign =false;
                    }
                    else 
                    {
                        if(!rotateFlag)
                        {
                            chassis.GridPoint(&current_pos);
                            // recordForward = current_pos.forward;
                            // FRIZY_LOG(LOG_DEBUG, "current_pos.forward:%f, recordForward:%f", current_pos.forward, recordForward);
                            recordForward = (360-gyo_angle_*180/_Pi);
                            rotateFlag = 1;
                        }
                        if(!rotaterobot(recordForward, 45))
                            return 0;
                        // chassis.chassisSpeed(0, 0, 1);
                        // WallFallowPara.BumpFlag = 0;
                        rotateFlag = 0;
                        backTime = BACK_TIME;
                        // back_sign =false;
                    }
                }
                // WallFallowPara.BumpDealState = WaitTurnOrEscape;
                WallFallowPara.VTurnCnt = 50;          
                WallFallowPara.BumpDealState = BumpNoAction;
                WallFallowPara.BumpFlag = 0;
                WallFallowPara.BumpCnt = 0;
                WallFallowPara.BumpRecord = BumpNo;
            }
            break;
        }
        // }
        FRIZY_LOG(LOG_DEBUG,"BumpDealState:%d finished", WallFallowPara.BumpDealState);
        return 1;
    }

    void AlongWall::wallFollowDeal(void)
    {
        
        int16_t CinDSpeed = 0;
        if(firstAlongWall)
        {
            if(first_flag)
            {
                first_flag = false;
                rotateFlag = 0;
                deal = 0;
                recognize = 0;
                escapeFlag = 0;
                recordForward = 360-gyo_angle_*180/_Pi;
                smallPlaceStartAngle = getAddAngle() / 10;
                FanBaseInitPit = escape.getPitRol(0);
            }
            if(!rotaterobot(recordForward, 45))
                return;
            firstAlongWall = false;
        }
        // FRIZY_LOG(LOG_INFO,"WallFallowPara.State: %d",WallFallowPara.State);
        // else                                 //无碰撞，处理沿墙动作
        // {
            if(WallFallowPara.PauseFlag)
            {
                WallFallowPara.BumpDealState = BumpNoAction;
                chassis.chassisSpeed(0, 0, 1);
                WallFallowPara.BumpFlag = 0;
                WallFallowPara.PauseState = WallFallowPara.State; //记录暂停前机子的状态
                WallFallowPara.State = WallFollowPause;
            }
            lastWallState = WallFallowPara.State;
            switch (WallFallowPara.State)
            {
                case WallFollowStart:
                    if(!WallFallowPara.Model)
                        break;
                    // WallFallowPara.WallFollowRunState = WallFollow_Run;
                    // WallFallowPara.Time = 0;
                    // WallFallowPara.State = WallFollowAlong;  //开始找墙
                break;

                // case WallFollowFind:
                    //直线找墙

                    //在绕柱沿墙找墙时，不通过沿墙值来判定沿墙方向
                    // if(!WallFallowPara.aroundFindWallFlag)
                    // findWallByValue();                      //通过沿墙值赋予方向
                // break;  

                case WallFollowAlong:
                    
                    // chassis.GetSensor(&currentSensor);
                    FRIZY_LOG(LOG_DEBUG,"WALLALONG LEFT SPEED:%d, RIGHT SPEED:%d", currentSensor.leftw, currentSensor.rightw);
                    if(WallFallowPara.Dir == LEFTAW)            //通过沿墙方向读取对应的沿墙值
                    {
                        WallFallowPara.Value = currentSensor.leftAlongWall;
                        // FRIZY_LOG(LOG_DEBUG,"LEFT_WallFallowPara.Value:%d", WallFallowPara.Value);
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)
                    {
                        WallFallowPara.Value = currentSensor.rightAlongWall;
                        // FRIZY_LOG(LOG_DEBUG,"RIGHT_WallFallowPara.Value:%d", WallFallowPara.Value);
                    }
                    if(smallPlaceFlag)
                    {
                        WallFallowPara.Dis = 3900;
                        FRIZY_LOG(LOG_DEBUG, "smallPlaceFlag = 1, WallFallowPara.Dis = 3900;");
                    }
                    // FRIZY_LOG(LOG_DEBUG,"pid wall");
                    CinDSpeed = wallPid(WallFallowPara.Value,WallFallowPara.Dis);//进行pid调速
                    //下发轮速
                    if(WallFallowPara.Dir == LEFTAW)//左沿墙
                    {
                        chassis.chassisSpeed(WALL_SPEED - (CinDSpeed / 2), WALL_SPEED + (CinDSpeed / 2), 1);
                        FRIZY_LOG(LOG_DEBUG,"LEFTAW send pid wall speed finished:%d, %d", WALL_SPEED - (CinDSpeed / 2),  WALL_SPEED + (CinDSpeed / 2));
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)//右沿墙
                    {
                        chassis.chassisSpeed(WALL_SPEED + (CinDSpeed / 2), WALL_SPEED - (CinDSpeed / 2), 1);
                        FRIZY_LOG(LOG_DEBUG,"RIGHTAW send pid wall speed finished:%d, %d", WALL_SPEED + (CinDSpeed / 2), WALL_SPEED - (CinDSpeed / 2));
                    }
                    recognizeSmallPlace();             
                break;

                case WallFollowEnd:      
                    WallFallowPara.WallFollowRunState = WallFollow_Stop;
                    chassis.chassisSpeed(0, 0, 1);
                    memset(&WallFallowPara, 0, sizeof(WallFallowPara_t));//复位沿墙参数
                    FRIZY_LOG(LOG_INFO, "Stop AlongWall");
                break;

                case WallFollowPause:
                    if(WallFallowPara.PauseFlag)
                    {
                        FRIZY_LOG(LOG_INFO, "WallFollow Pause");
                    }
                    WallFallowPara.PauseFlag = 0;
                    WallFallowPara.WallFollowRunState = WallFollow_Pause;
                break;  

                case WallFollowVIRTUAL:  
                    FRIZY_LOG(LOG_DEBUG, "WallFollowVIRTUAL");
                    if(WallFallowPara.VTurnCnt)
                    {
                        WallFallowPara.VTurnCnt --;
                    }
                    if(WallFallowPara.Dir == LEFTAW)
                    {
                        chassis.chassisSpeed(180 * 0.6, 180, 1);
                        break;
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)
                    {
                        chassis.chassisSpeed(180, 180 * 0.6, 1);
                        break;
                    }
                    // FRIZY_LOG(LOG_DEBUG, "along the virtual wall");
                break;

                case WallFollowForbid:
                    if(WallFallowPara.Dir == LEFTAW)
                    {
                        chassis.chassisSpeed(180 * 0.4, 180, 1);
                        break;
                    }
                    else if(WallFallowPara.Dir == RIGHTAW)
                    {
                        chassis.chassisSpeed(180, 180 * 0.4, 1);
                        break;
                    }
                break;
            }
            backTime = BACK_TIME;
        // }

    }

    void AlongWall::recognizeSmallPlace()
    {
        //获取陀螺仪累加角度
        int addAngle = getAddAngle() / 10;
        FRIZY_LOG(LOG_DEBUG, "addAngle:%d, smallPlaceStartAngle:%d, 1-2:%d", addAngle, smallPlaceStartAngle, addAngle - smallPlaceStartAngle);
        //小范围检测
        if(((addAngle - smallPlaceStartAngle >= 450 && WallFallowPara.Dir == LEFTAW) ||
           (addAngle - smallPlaceStartAngle <= -450 && WallFallowPara.Dir == RIGHTAW)) &&
           !enterSmallPlaceFlag)
        {
            FRIZY_LOG(LOG_DEBUG, "check addAngle:%d, smallPlaceStartAngle:%d", addAngle, smallPlaceStartAngle);
            smallPlaceCnt = getCurrentTime();
            enterSmallPlaceFlag = 1;
            smallPlaceStartAngle = addAngle;
        }
        //小范围判断
        if(((addAngle - smallPlaceStartAngle > 720 && WallFallowPara.Dir == LEFTAW) ||
           (addAngle - smallPlaceStartAngle < -720 && WallFallowPara.Dir == RIGHTAW)) && enterSmallPlaceFlag == 1)
        {
            FRIZY_LOG(LOG_DEBUG, "judge addAngle:%d, smallPlaceStartAngle:%d", addAngle, smallPlaceStartAngle);
            smallPlaceTurnCnt = 0;
            smallPlaceCnt = getCurrentTime();
            enterSmallPlaceFlag = 2;    
            FRIZY_LOG(LOG_DEBUG, "start smallplace mode");
            smallPlaceFlag = 1;
            smallPlaceStartAngle = addAngle;
        }
        else if(getCurrentTime() - smallPlaceCnt > 60000 && enterSmallPlaceFlag == 1)
        {
            FRIZY_LOG(LOG_DEBUG, "smallPlaceCnt > 60000");
            smallPlaceCnt = getCurrentTime();
            if((addAngle - smallPlaceStartAngle < 450 && WallFallowPara.Dir == LEFTAW) ||
               (addAngle - smallPlaceStartAngle > -450 && WallFallowPara.Dir == RIGHTAW))
            {
                FRIZY_LOG(LOG_DEBUG, "quit smallplace mode");
                smallPlaceFlag = 0;
                enterSmallPlaceFlag = 0;
            }
            else if((addAngle - smallPlaceStartAngle > 450 && WallFallowPara.Dir == LEFTAW) ||
                    (addAngle - smallPlaceStartAngle < -450 && WallFallowPara.Dir == RIGHTAW))
            {
                FRIZY_LOG(LOG_DEBUG, "start smallplace mode");
                smallPlaceTurnCnt = 0;
                enterSmallPlaceFlag = 2;
                smallPlaceFlag = 1;
            }
            smallPlaceStartAngle = addAngle;
        }
        if(((addAngle - smallPlaceStartAngle > 1080 && WallFallowPara.Dir == LEFTAW) ||
           (addAngle - smallPlaceStartAngle < -1080 && WallFallowPara.Dir == RIGHTAW)) && enterSmallPlaceFlag == 2)
        {
            FRIZY_LOG(LOG_DEBUG, "judge addAngle:%d, smallPlaceStartAngle:%d", addAngle, smallPlaceStartAngle);
            smallPlaceCnt = getCurrentTime();
            smallPlaceTurnCnt = 0;
            FRIZY_LOG(LOG_DEBUG, "quit smallplace mode");
            smallPlaceFlag = 0;
            enterSmallPlaceFlag = 0;
            smallPlaceStartAngle = addAngle;
        }
        else if(getCurrentTime() - smallPlaceCnt > 90000 && enterSmallPlaceFlag == 2)   //退出小范围
        {
            FRIZY_LOG(LOG_DEBUG, "smallPlaceCnt > 90000");
            smallPlaceCnt = getCurrentTime();
            if((addAngle - smallPlaceStartAngle <= 450 && WallFallowPara.Dir == LEFTAW) ||
               (addAngle - smallPlaceStartAngle >= -450 && WallFallowPara.Dir == RIGHTAW))
            {
                FRIZY_LOG(LOG_DEBUG, "quit smallplace mode");
                smallPlaceFlag = 0;
                enterSmallPlaceFlag = 0;
                smallPlaceTurnCnt = 0;
            }
            smallPlaceStartAngle = addAngle;
        }   
    }

    void AlongWall::findWallByValue()
    {
        // chassis.GetSensor(&currentSensor);
        if(currentSensor.leftAlongWall > IR_BEGIN_ALONG && !WallFallowPara.Dir)
        {
            if(WallFallowPara.Model == DockWallFollow)  //暂时不管
            {
                // FUN_WheelCtrlStop();
                // if(NoDir == WallFallowPara.Dir)
                //     WallFallowPara.Dir = RIGHT;
                // WallFallowPara.BumpFlag = 1;
                // WallFallowPara.BumpDealState = WallValueTurn;
            }
            else
                WallFallowPara.Dir = LEFTAW;
        }
        else if(currentSensor.rightAlongWall > IR_BEGIN_ALONG && !WallFallowPara.Dir)
        {
            if(WallFallowPara.Model == UncleanWallFollow)   //暂时不管
            {
                // FUN_WheelCtrlStop();
                // WallFallowPara.Dir = LEFT;
                // WallFallowPara.BumpFlag = 1;
                // WallFallowPara.BumpDealState = WallValueTurn;
            }
            else
                WallFallowPara.Dir = RIGHTAW;
        }
    }

    void AlongWall::straightFindWall()
    {
        chassis.chassisSpeed(200, 200, 1);
        // chassis.GetSensor(&currentSensor);
        if(WallFallowPara.Dir == LEFTAW)            //通过沿墙方向读取对应的沿墙值
        {
            WallFallowPara.Value = currentSensor.leftAlongWall;
        }
        else if(WallFallowPara.Dir == RIGHTAW)
        {
            WallFallowPara.Value = currentSensor.rightAlongWall;
        }
        if(WallFallowPara.Value > 3000)
            chassis.chassisSpeed(0, 0, 1);
        
    }

    void AlongWall::chassisAlongWall()
    {
        FRIZY_LOG(LOG_INFO, "start to chassisWallAlong");
        bool tmp_index;
        // StartWallFollow(RandomWallFollow, LEFTAW, SLOW);
        while(1)
        {
            
            if(GLOBAL_CONTROL != WHEEL_RUN)
            {
                // FRIZY_LOG(LOG_DEBUG, "GLOBAL_CONTROL:%d", GLOBAL_CONTROL);
                usleep(200 * 1000);
                continue;
            }
            else 
            {
                // FRIZY_LOG(LOG_DEBUG, "alongwall task running");
                usleep(20 * 1000);
                if(alongwalk_run_index == true)
                {   
                    FRIZY_LOG(LOG_DEBUG, "smallPlaceFlag:%d, WallFallowPara.BumpFlag:%d", smallPlaceFlag, WallFallowPara.BumpFlag);
                    // FRIZY_LOG(LOG_DEBUG, "alongwalk_run_index == true");
                    chassis.GetSensor(&currentSensor);          //获取底盘数据
                    FRIZY_LOG(LOG_INFO, "Sensor.bump:%d.obs:%d.cliff:%d",currentSensor.bump,currentSensor.obs,currentSensor.cliff);
                    // if(escapeFlag)
                    // {
                    //     if(!escapeWallBack())
                    //         continue;
                    //     escapeFlag = 0;
                    //     backTime = BACK_TIME;
                    // }
                    if(FanBaseEscapeFlag)
                    {
                        fanBaseEscapeFlagClear();
                        FRIZY_LOG(LOG_DEBUG, "AlongWall Start Escape");
                        chassis.chassisSpeed(0, 0, 1);
                        escape.stuckType == ESCAPE_EVENT_TYPE_LEFT_OFF;
                        if(escape.escDeal(1))
                        {
                            FRIZY_LOG(LOG_DEBUG, "AlongWall Escape SUCCEED");
                        }
                        else
                        {
                            FRIZY_LOG(LOG_DEBUG, "AlongWall Escape FAILED");
                        }
                    }
                    if(!WallFallowPara.BumpFlag)                            //处理机器类碰撞，不做扫描碰撞信息
                        bumpScan();
                    if(WallFallowPara.BumpFlag == 1)          //处理类碰撞动作，不处理沿墙动作
                    {
                        // FRIZY_LOG(LOG_DEBUG, "WallFallowPara.BumpFlag:1");
                        initPid();
                        // if(WallFallowPara.BumpRecord == BumpInter)
                        // {
                        //     if(!wallBack())
                        //     continue;
                        // }
                        bumpDeal();//机器 类碰撞  自旋处理
                    }
                    else
                    {
                        // FRIZY_LOG(LOG_DEBUG, "wallFollowDeal");
                        wallFollowDeal();    
                    }
                }
            // if(WallFallowPara.alongwalk_run_index == false && tmp_index ==true)
            // {
            //     chassis.chassisSpeed(0,0,1);
            //     wallstate_t = EXIT_WALL;
            // }
            }
            tmp_index = alongwalk_run_index;
        }
    }
    void AlongWall::initPid(void)
    {
        memset(&WallFallowPara.PID, 0 ,sizeof(PID_t));//复位沿墙参数
    }

    //NowValue 当前沿墙值， AimValue 目标沿墙值
    int16_t AlongWall::wallPid(uint16_t NowValue,uint16_t AimValue)
    {
        int16_t out;
        if(AimValue == BLACK_WALL)
        {
            if(NowValue < 500)
            {
                WallFallowPara.PID.P = (AimValue - NowValue) / 5;
            }
            else
            {
                WallFallowPara.PID.P = (AimValue - NowValue) / 16;
            }
            WallFallowPara.PID.PI += WallFallowPara.PID.P;
            WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P) * 8;
            // WallFallowPara.PID.P = (AimValue - NowValue)/11;
            // WallFallowPara.PID.PI += WallFallowPara.PID.P;
            // WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P)*4;
        }
        else
        {
            // if(NowValue < 200)
            // {
            //     WallFallowPara.PID.P = (AimValue - NowValue) / 12;
            // }
            // else
            {
                WallFallowPara.PID.P = (AimValue - NowValue) / 16;
            }
            WallFallowPara.PID.PI += WallFallowPara.PID.P / 4;
            WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P) * 8;
            // if(NowValue > 3100)
            //     WallFallowPara.PID.P = (AimValue - NowValue);
            // else
            //     WallFallowPara.PID.P = (AimValue - NowValue)/8;
            // WallFallowPara.PID.PI += WallFallowPara.PID.P / 4;
            // WallFallowPara.PID.PD = (WallFallowPara.PID.P - WallFallowPara.PID.Last_P)*8;
        }
        if(WallFallowPara.PID.PI > 30)
        {
            WallFallowPara.PID.PI = 30;
        }
        else if(WallFallowPara.PID.PI < -30)
        {
            WallFallowPara.PID.PI = -30;
        }
        WallFallowPara.PID.Last_P = WallFallowPara.PID.P;
        out = WallFallowPara.PID.P + WallFallowPara.PID.PD;
        FRIZY_LOG(LOG_DEBUG, "out:%d", out);
        if(AimValue == BLACK_WALL)
        {
            if(out > 180)
            {
                out = 180;
            }
            else if(out < -180)
            {
                out = -180;
            }
        }
        else
        {
            if(out > 240)
            {
                out = 240;
            }
            else if(out < -240)
            {
                out = -240;
            }
        }
        FRIZY_LOG(LOG_DEBUG, "P:%d I:%d D:%d NowValue:%d AimValue:%d out:%d", WallFallowPara.PID.P,WallFallowPara.PID.PI,WallFallowPara.PID.PD,NowValue,AimValue,out);
        return out;
    }

    bool AlongWall::rotaterobot(float recordforward, int angle)
    {
            float cur_foward,aimforward;
            chassis.GridPoint(&current_pos);
            // cur_foward = current_pos.forward;
            if(WallFallowPara.Dir == RIGHTAW)
            {   
                // if(WallFallowPara.BumpRecord == BumpInter)
                // {
                //     FRIZY_LOG(LOG_DEBUG, "getCurrentTime() - recordTime:%lld", getCurrentTime() - recordTime);
                //     if((getCurrentTime() - recordTime >= 10*1000))
                //     {
                //         chassis.chassisSpeed(0, 0, 1);
                //         escapeFlag = 1;
                //         return 1;
                //     }
                // }
                aimforward = recordforward - angle;
                if(aimforward <= 0)
                {
                    aimforward = 360 + aimforward;
                }
                if(WallFallowPara.BumpRecord == BumpInter && !recognize)
                {
                    if(recognizeBlackWall())
                        recognize = 1;
                }
                // FRIZY_LOG(LOG_DEBUG, "aimforward: %f, recordforward: %f",aimforward, recordforward);
                chassis.chassisSpeed(-120,120,1);   
                // chassis.GridPoint(&current_pos);
                FRIZY_LOG(LOG_DEBUG, "recordforward = %f, aimforward = %f, current_pos.forward = %f", recordforward, aimforward, fabs(360-gyo_angle_*180/_Pi));
                // if(fabs(current_pos.forward - aimforward) < error_index || fabs(current_pos.forward - aimforward) > (360 - error_index))
                if(fabs((360-gyo_angle_*180/_Pi) - aimforward) < error_index || fabs((360-gyo_angle_*180/_Pi) - aimforward) > (360 - error_index))
                {   
                    chassis.chassisSpeed(0, 0, 1);
                    FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
                    return 1;
                }
                // return 0;
            }
            else if(WallFallowPara.Dir == LEFTAW)
            {   
                // if(WallFallowPara.BumpRecord == BumpInter)
                // {
                //     FRIZY_LOG(LOG_DEBUG, "getCurrentTime() - recordTime:%lld", getCurrentTime() - recordTime);
                //     if((getCurrentTime() - recordTime >= 10 * 1000))
                //     {
                //         chassis.chassisSpeed(0, 0, 1);
                //         escapeFlag = 1;
                //         return 1;
                //     }
                // }
                aimforward = recordforward + angle;
                if(aimforward >= 360)
                {
                    aimforward = aimforward - 360;
                }
                if(WallFallowPara.BumpRecord == BumpInter && !recognize)
                {
                    if(recognizeBlackWall())
                        recognize = 1;
                }
                // FRIZY_LOG(LOG_DEBUG, "aimforward: %f, recordforward: %f",aimforward, recordforward);
                chassis.chassisSpeed(120, -120, 1);
                // chassis.GridPoint(&current_pos);
                // FRIZY_LOG(LOG_DEBUG, "recordforward = %f, aimforward = %f, current_pos.forward = %f", recordforward, aimforward, current_pos.forward);  
                FRIZY_LOG(LOG_DEBUG, "recordforward = %f, aimforward = %f, current_pos.forward = %f", recordforward, aimforward, fabs(360-gyo_angle_*180/_Pi));              
                // if(fabs(current_pos.forward - aimforward) < error_index || fabs(current_pos.forward - aimforward) > (360 - error_index))
                if(fabs((360-gyo_angle_*180/_Pi) - aimforward) < error_index || fabs((360-gyo_angle_*180/_Pi) - aimforward) > (360 - error_index))
                {
                    chassis.chassisSpeed(0, 0, 1);
                    FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");     
                    return 1;
                }
                return 0;
            }
    }
    void AlongWall::alongwallStart()
    {
        FRIZY_LOG(LOG_INFO, "alongwall thread start"); 
        alongwalk_run_index = false;
        alongwall_thread_ = std::make_shared<std::thread>(&AlongWall::chassisAlongWall, this);
    }
    void AlongWall::alongwallStop()
    {
        FRIZY_LOG(LOG_INFO, "alongwall thread stop"); 
        {
            alongwalk_run_index = false;
            WallFallowPara.State = WallFollowEnd;
            // log_info("exit thread id 0x%08x\n", thread_->get_id());
            alongwall_thread_->join();
            alongwall_thread_ = nullptr;
        }
    }

    bool AlongWall::isNear(float x, float y)
    {
        float dis;
        // FRIZY_LOG(LOG_DEBUG, "x:%f, y:%f", x, y);
        for(auto p : _maze.forbidenPoint)
        {
            dis = sqrtf((p.first - x) * (p.first - x) + (p.second - y) * (p.second - y));
            if(dis < 0.05)
                return true;
        }
        return false;
    }

    float AlongWall::getBanDis(int dir)
    {
        float tmpx, tmpy, forward, tmpcan, tmp;
        chassis.GridPoint(&current_pos);
        if(dir == 0)        //front
            forward = current_pos.forward;
        else if(dir == 1)   //right
        {
            forward = current_pos.forward + 90;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 2)   //left
        {
            forward = current_pos.forward - 90;
            if(forward < 0) 
                forward = forward + 360;
        }
        else if(dir == 3)
        {
            forward = current_pos.forward + 45;
            if(forward >= 360) 
                forward = forward - 360;
        }
        else if(dir == 4)
        {
            forward = current_pos.forward - 45;
            if(forward < 0) 
                forward = forward + 360;
        }
        tmpcan = (forward / 180) * 3.14;
        for(float i = 0.05f; i <= 0.5f; i += 0.05f)
        {
            tmpx = (current_pos.realx * 15 / 100) + (i * cos(tmpcan));
            tmpy = (current_pos.realy * 15 / 100) - (i * sin(tmpcan));
            // tmpx = current_pos.realx + (i * cos(tmpcan));
            // tmpy = current_pos.realy - (i * sin(tmpcan));
            if(isNear(tmpx, tmpy))
            {
                tmp = sqrtf((((current_pos.realx * 15 / 100) - tmpx) * ((current_pos.realx * 15 / 100) - tmpx)) + (((current_pos.realy * 15 / 100) - tmpy) * ((current_pos.realy * 15 / 100) - tmpy)));
                FRIZY_LOG(LOG_DEBUG, "catch forbid signal, return tmp:%f", tmp);
                return tmp;
            }
        }
        return INT_MAX;
    }

}    