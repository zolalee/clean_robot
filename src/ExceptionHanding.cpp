/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-11 15:53:55
 * @Project      : UM_path_planning
 */

#include <iostream>
#include "common_function/ExceptionHanding.h"
#include "navigation_algorithm/AlongWall.h"

#define _Pi 3.1415926

using namespace useerobot;
// extern float gyo_angle_;
namespace useerobot
{
    /**** 架空脱困识别 end ********/
    // static int16_t angVelOutCnt = 0,abnrCurrOutCntL = 0,abnrCurrOutCntR = 0;
    //脱困检测变量
    stuckCheckPara_t escCkeckPara;
    // 脱困参数定义
    escapeActionPara_t escActPara;
    //记录脱困前的沿墙方向
    WallFollowDir_t alongWallDir;

    //能转动的角度范围定义
    #define ENABLE_MOVE_ANGLE_H (40)
    #define ENABLE_MOVE_ANGLE_L (10)
    #define WHEEL_LEFT_CURRENT_LIMIT (150)  //左轮电流限制
    #define WHEEL_RIGHT_CURRENT_LIMIT (150) //右轮电流限制
    #define WHEEL_CURRENT_ABNORMAL (10) //电流异常差值倍数 放大了10倍
    //标识宏定义
    #define CUR_ABNORMAL (0x01) //电流异常标识
    #define RATE_ABNORMAL (0x02) //角速度异常标识
    #define PITROL_ABNORMAL (0x04) //俯仰横滚异常标识
    #define ACC_ABNORMAL (0x08) //加速度异常标识
    //数组内存宏定义
    #define STUCK_ACC_ARRY_NUM (5)//卡死判定加速度滑动平均值数组内存
    #define PITROL_AVG_NUM (5)//俯仰和横滚滑动平均值数组内存
    #define PITROL_LAST_AVG_NUM (10)//1.6s前俯仰和横滚滑动平均值数组内存
    #define AVG_WHEEL_CURARRY_NUM (5)//电流滑动平均值数组内存
    #define AVG_LAST_WHEEL_CURARRY_NUM (10)//1.6s前电流滑动平均值数组内存
    /*滑动数据存储数组定义*/
    static int escLastLeftSideBrushCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int escLastRightSideBrushCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int escLastLeftCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};
    static int escLastRightCurAvg[AVG_LAST_WHEEL_CURARRY_NUM] = {0};    
    static int escLeftCurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int escRightCurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int RightSideBrushCurArry[AVG_WHEEL_CURARRY_NUM] = {0};
    static int LeftSideBrushCurArry[AVG_WHEEL_CURARRY_NUM] = {0};

    static int gyroAccBuf[STUCK_ACC_ARRY_NUM] = {0};
    static float pitRolAvgArry[PITROL_AVG_NUM] = {0};
    static float pitRolLastAvg[PITROL_LAST_AVG_NUM] = {0};
    static int escPreActionArry[3] = {0};//记录脱困前的前三个动作


    static int16_t angVelOutCnt = 0,abnrCurrOutCntL = 0,abnrCurrOutCntR = 0;



    
    SmallArea smallArea;

    EscapePlan::EscapePlan(/* args */)
    {
    }
 
    EscapePlan::~EscapePlan()
    {

    }
    
    void EscapePlan::Init()
    {
        trouble.aimPoint.x = 1000,trouble.aimPoint.y = 1000;
        smallArea.smallWall = 0;
        trouble.type = nothing;
        wallPoint.clear();
        wallAgg.clear();     
        rollsign = 0; 
    }


    Trouble EscapePlan::EscapeRecognition(Sensor sensor,Grid cur)
    {
      printf("wwwccc1.%d.%d.%d\n",sensor.leftw,sensor.rightw,IsWall());
      if (IsWall() != 0)
      {
        printf("step1\n");
        stuckTime3 ++;
        if (stuckTime3 == STUCK_WALL_TIME)
        {
          if (abs(cur.x - stuck.x) + abs(cur.y - stuck.y) < 3
              && (abs(cur.forward - stuck.forward) < 45 || abs(cur.forward - stuck.forward) > 315))
          {
            printf("kasi3\n");
            stuckTime3 = 0;
            trouble.type = stucks;
            return trouble;
          }
        
          stuck.x = cur.x,stuck.y = cur.y;
          stuckTime3 = 0;
        }     
        printf("step2\n");
      }
      else
      {
        
        stuckTime3 = 0;
        if (sensor.leftw > 0 && sensor.rightw > 0)
        {
          stuckTime2 = 0;

          stuckTime1 ++;
          if (stuckTime1 == STUCK_GO_TIME)
          {
            if (abs(cur.x - stuck.x) + abs(cur.y - stuck.y) < 3)
            {
              printf("kasi1\n");
              stuckTime1 = 0;
              trouble.type = stucks;
              return trouble;
            }
            stuck.x = cur.x,stuck.y = cur.y;
            stuckTime1 = 0;
          }
        }
        else if (sensor.leftw * sensor.rightw < 0)
        {
          stuckTime1 = 0;

          stuckTime2 ++;
          if (stuckTime2 == STUCK_TURN_TIME)
          {
            printf("kasi2\n");
            stuckTime2 = 0;
            trouble.type = stucks;
            return trouble;
          }
          
        }
        else
        {
          printf("stay\n");
          stuckTime1 = 0;
          stuckTime2 = 0;
        }
      }

      //
      trouble.type = nothing;
      if (IsWall() == 2 && wallPoint.size() < 1000 && wallAgg.size() < 1000)
      {
                                  
                        

        if (smallArea.smallWall 
            && abs(smallArea.smallPoint.x - cur.x) + abs(smallArea.smallPoint.y - cur.y) > 8){
            FRIZY_LOG(LOG_DEBUG,"huifu");
            smallArea.smallWall = 0;
        }

        FRIZY_LOG(LOG_DEBUG,"step3.%d.%d",wallPoint.size(),wallAgg.size());
        if (wallAgg.size() != 0)
        {
          
          auto it = wallAgg.end()-1;
          for(;;it--)
          {
            if (wallAgg.back().addAngle - it->addAngle > 330)
            {
              FRIZY_LOG(LOG_DEBUG,"roll1.%d.%d",wallAgg.back().addAngle,it->addAngle);
              rollsign = 1;
              break;
            }
            
            if (wallAgg.back().addAngle - it->addAngle < -450){

                if (process == BOUND && IsWall() == 2 
                    && wallAgg.back().addAngle - it->addAngle < -450){

                    
                    int8_t tempro = 1;
                    for(;it < wallAgg.end();it++)
                    {
                        if (abs(wallAgg.back().x - it->x) + abs(wallAgg.back().y - it->y > 5))
                        {
                            tempro = 0;
                            break;
                        }                    
                    }
                    
                    if (tempro == 1)
                    {
                        Init();
                        smallArea.smallPoint = cur;
                        //smallArea.smallWall = 1;
                        FRIZY_LOG(LOG_DEBUG,"xiaofanwei2");
                        trouble.type = smallarea;
                        return trouble;
                    }
                }

               FRIZY_LOG(LOG_DEBUG,"roll2.%d.%d",wallAgg.back().addAngle,it->addAngle);
               rollsign = 2;
               break;   

            }
            if (it == wallAgg.begin())
              break;
          }
        }
        
        if (wallPoint.size() == 0 || cur.x != wallPoint.back().x || cur.y != wallPoint.back().y)
        {
          FRIZY_LOG(LOG_DEBUG,"step4.%d.%d.%d.%d",wallPoint.size(),wallAgg.size(),trouble.aimPoint.x,trouble.aimPoint.y);  
          if (abs(trouble.aimPoint.x - cur.x) + abs(trouble.aimPoint.y - cur.y) < sensor.size)
          {
            FRIZY_LOG(LOG_DEBUG,"escape.%d",rollsign);
            if (rollsign == 1)
            {
              FRIZY_LOG(LOG_DEBUG,"raozhu1");
              trouble.type = windcolumn;
              return trouble;

            }
            else if (rollsign == 2 && process != BOUND)
            {
              Init();
              smallArea.smallPoint = cur;
              //smallArea.smallWall = 1;
              FRIZY_LOG(LOG_DEBUG,"xiaofanwei1");
              trouble.type = smallarea;
              return trouble;
            }
          }
          else
          {
            int tempN = wallPoint.size() - sensor.size;
            for(int i = 0;i < tempN;i++)
            {
              //
              if (abs(wallPoint[i].x - cur.x) + abs(wallPoint[i].y - cur.y) < 1)
              {
                trouble._maprange.xmax = -1000;
                trouble._maprange.xmin = 1000;
                trouble._maprange.ymax = -1000;
                trouble._maprange.ymin = 1000;                
                auto it = wallPoint.end()-1;
                for(;;it--)
                {
                  if (it->x < trouble._maprange.xmin) trouble._maprange.xmin = it->x;
                  if (it->x > trouble._maprange.xmax) trouble._maprange.xmax = it->x;
                  if (it->y < trouble._maprange.ymin) trouble._maprange.ymin = it->y;
                  if (it->y > trouble._maprange.ymax) trouble._maprange.ymax = it->y;                 
                  FRIZY_LOG(LOG_DEBUG,"yiquan.%d.%d",it->x,it->y);
                  trouble.rollArray.push_back(*it);
                  if (cur.x == it->x && cur.y == it->y)
                    break;
                }

                FRIZY_LOG(LOG_DEBUG,"aimPoint.%d.%d",wallPoint[i+sensor.size].x,wallPoint[i+sensor.size].y);
                trouble.aimPoint = wallPoint[i+sensor.size];
                break;
              }
              else
              {
                
              }
            }
          }

          FRIZY_LOG(LOG_DEBUG,"input1.%d.%d",cur.x,cur.y);
          wallPoint.push_back(cur);
        }

        if (wallAgg.size() == 0 || abs(cur.addAngle - wallAgg.back().addAngle) > 30)
        {
            FRIZY_LOG(LOG_DEBUG,"input2.%d",cur.addAngle);

            wallAgg.push_back(cur);        

        }
      }
      else
      {
        if (wallPoint.size() || wallAgg.size())
        {
          FRIZY_LOG(LOG_DEBUG,"clear all");
          smallArea.smallWall = 0;
          trouble.aimPoint.x = 1000,trouble.aimPoint.y = 1000;

           wallPoint.clear();
           wallAgg.clear(); 
        }
        FRIZY_LOG(LOG_DEBUG,"nothing");
        trouble.type = nothing;
        rollsign = 0;
      }
      
      return trouble;
    }




    /**************************************分割线************************************************/
    //获取当前时间  单位ms
    long long getTime()
    {
        struct timeval t;
        gettimeofday(&t, NULL);
        return (long long)t.tv_sec*1000 + (long long)t.tv_usec/1000;
    }

    //获取两个轮子状态  
    wheelState_t EscapePlan::getWheelControlState()    
    {   
        //0:停止 1:前进 2:后退
        wheelState_t tmp_wheelsta;
        chassisEscape.GetSensor(&escapSensor);
        if(escapSensor.leftw < 0)
            tmp_wheelsta.wheelLeftState = 2;
        else if(escapSensor.leftw == 0)
            tmp_wheelsta.wheelLeftState = 0;
        else 
            tmp_wheelsta.wheelLeftState = 1;
        if(escapSensor.rightw < 0)
            tmp_wheelsta.wheelRightState = 2;
        else if(escapSensor.rightw == 0)
            tmp_wheelsta.wheelRightState = 0;
        else 
            tmp_wheelsta.wheelRightState = 1;
        tmp_wheelsta.wheelState = chassisEscape.getWheelState();
        return tmp_wheelsta;
    }

    //获取俯仰横滚值 num 0:俯仰    1:横滚
    float EscapePlan::getPitRol(int num)
    {
        chassisEscape.GetSensor(&escapSensor);
        if(num == 0)
            return escapSensor.XAngle;
        else if(num == 1)
            return escapSensor.YAngle;
        else 
            return 0;

    }

    //获取加速度
    int EscapePlan::getAcc(int num)
    {
        chassisEscape.GetSensor(&escapSensor);
        if(num == 0)
            return escapSensor.XAcc;
        else if(num == 1)
            return escapSensor.YAcc;
        else if(num == 2)
            return escapSensor.ZAcc;
        else 
            return 0;
    }

    //获取角速度
    int EscapePlan::getAngleRate()
    {
        chassisEscape.GetSensor(&escapSensor);
        return escapSensor.zGyroOriginal;
    }
    
    //获取累加角度
    int EscapePlan::getAddAngle()
    {
        chassisEscape.GetSensor(&escapSensor);
        return escapSensor.addAngle;
    }

    /*
    *   数组入队 (滑动存入一定的数据)
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    void SlidingArrayAddF(float value_buf[],int n,float ADNum)
    {
        for(int i = 0; i < n - 1; i ++)
        {
            memcpy(&value_buf[i],&value_buf[i+1],sizeof(value_buf[i]));
        }
        value_buf[n-1]=ADNum;
    }

    /*
    *   数组入队 (滑动存入一定的数据)
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    void SlidingArrayAddI(int value_buf[],int n,int ADNum)
    {
        for(int i = 0; i < n - 1; i ++)
        {
            memcpy(&value_buf[i],&value_buf[i+1],sizeof(value_buf[i]));
        }
        value_buf[n-1]=ADNum;
    }

    //滑动平均滤波算法（递推平均滤波法） 

    /*
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    int GlideFilterAD(int value_buf[],int n,int ADNum)
    {
        int sum=0;
        SlidingArrayAddI(value_buf,n,ADNum);

        for(int count=0;count<n;count++)
            sum+=value_buf[count];
        return (int)(sum/n);
    
    }

    /*
        ADNum为获得的AD数
        n为数组value_buf[]的元素个数。该函数主要被调用，利用参数的数组传值
    */
    float GlideFilterADf(float value_buf[],int n,float ADNum)
    {
        float sum=0.0f;
        SlidingArrayAddF(value_buf,n,ADNum);
        for(int count=0;count<n;count++)
            sum+=value_buf[count];
        return (float)(sum/n);
    }

    //加速度滑动平均值
    int EscapePlan::gyroAccSlidingAvg()
    {
        int accY = getAcc(1);
        return  GlideFilterAD(gyroAccBuf,STUCK_ACC_ARRY_NUM,accY)*10;//除以1024
    }

    /*
    * 俯仰和横滚滑动平均值
    * 机器   pit 前面抬起 负数 后面抬起 正数
    *        rol 右边抬起 负数 左边抬起 正数
    */
    float EscapePlan::gyroPitRolSlidingAvg()
    {
        chassisEscape.GetSensor(&escapSensor);
        float anglePitEsc =  escapSensor.XAngle;//俯仰
        float angleRolEsc =  escapSensor.YAngle;//横滚
        float pitRolSqrtf = 0;
        pitRolSqrtf = sqrtf(anglePitEsc*anglePitEsc+angleRolEsc*angleRolEsc);
        FRIZY_LOG(LOG_INFO,"gyroPitRolSlidingAvg Pit:%f,Rol:%f, PitRol:%f",anglePitEsc, angleRolEsc, pitRolSqrtf);
        return GlideFilterADf(pitRolAvgArry,PITROL_AVG_NUM,pitRolSqrtf);
    }


    //机器卡死判定      return true 卡死  false 正常
    bool EscapePlan::ESCAPE_StuckTrigCheck_My_Test()
    {
        // FRIZY_LOG(LOG_INFO, "机器卡死判定");
        uint8_t stuckRet = 0;
        static int8_t lastescWheelLSta = 0;
        static int8_t lastescWheelRSta = 0;
        static int8_t PitRolCheckCnt = 0;
        
        wheelState_t wheelSta_tmp;
        wheelSta_tmp = getWheelControlState();
        // FRIZY_LOG(LOG_DEBUG,"10SPit:%f,10SRol:%f,10SPitRol:%f",escActPara.escPre10PrePit, escActPara.escPre10PreRol, escActPara.escPre10PrePitRol);
        // FRIZY_LOG(LOG_DEBUG,"wheelstate:%d, leftwheelstate:%d, rightwheelstate:%d",wheelSta_tmp.wheelState,wheelSta_tmp.wheelLeftState,wheelSta_tmp.wheelRightState);
        if(lastescWheelLSta != wheelSta_tmp.wheelLeftState || 
           lastescWheelRSta != wheelSta_tmp.wheelRightState)
        {
            PitRolCheckCnt = 0;
            // stuckRet &= ~CUR_ABNORMAL;  //电流异常标识
            stuckRet &= ~PITROL_ABNORMAL;   //俯仰异常标识
        }   
        lastescWheelLSta = wheelSta_tmp.wheelLeftState;
        lastescWheelRSta = wheelSta_tmp.wheelRightState;
        
        //电流检测
        if(abnormalElectricCheck())   
        {
            FRIZY_LOG(LOG_INFO,"cur check:abnormal");
            stuckRet |= CUR_ABNORMAL;
        }
        else 
        {
            // FRIZY_LOG(LOG_DEBUG,"cur check:normal");
            stuckRet &= ~CUR_ABNORMAL;
        
        }
        //俯仰和横滚检测
        if(abnormalPitRolCheck())
        {
            FRIZY_LOG(LOG_INFO,"pitrol check:abnormal");
            stuckRet |= PITROL_ABNORMAL;
            PitRolCheckCnt = 500/20;
        }
        else
        {
            // FRIZY_LOG(LOG_DEBUG,"pitrol check:normal");
            if(PitRolCheckCnt>0)
            {
                PitRolCheckCnt--;
                stuckRet &= ~PITROL_ABNORMAL;
            }
        }
        // if(SideBrushElectricCheck())
        // {
        //     FRIZY_LOG(LOG_DEBUG,"SideBrushElectricCheck :abnormal");
        //     stuckType = ESCAPE_EVENT_TYPE_STUCK;
        //     return true;
        // }
        // if( ((stuckRet&CUR_ABNORMAL) && (stuckRet & PITROL_ABNORMAL))//俯仰横滚和电流同时异常判定为困住
        //     || (stuckRet&RATE_ABNORMAL)//角速度异常判定为异常
        //   )
        if((stuckRet & CUR_ABNORMAL) && (stuckRet & PITROL_ABNORMAL))
        {

            angVelOutCnt = 0;
            abnrCurrOutCntL = 0;
            abnrCurrOutCntR = 0;
            // printf("esc xxx myTest stuck[%d]\n",stuckRet);
            stuckType = ESCAPE_EVENT_TYPE_STUCK;
            return true;
        }
        return false;
    }

    static int addAngleCnt = 0;
    //架空离地判定初始化
    void EscapePlan::ESCAPE_LeftOffCheckInit()
    {
        chassisEscape.GridPoint(&escapGrid);
        LeftOffDistance = 0;
        LeftOffStartAngle = getAddAngle() / 10;
        LeftOffStartX = escapGrid.realx * 15 / 100;
        LeftOffStartY = escapGrid.realy * 15 / 100;    
        addAngleCnt = 0;
    }
    
    //架空离地判定
    bool EscapePlan::ESCAPE_LeftFloorCheck()
    {
        int checkFlag = 0;
        chassisEscape.GridPoint(&escapGrid);
        if(!recordPosFlag)
        {
            recordPosFlag = 1;
            LeftOffStartX = escapGrid.realx * 15 / 100;
            LeftOffStartY = escapGrid.realy * 15 / 100; 
            LeftOffStartAngle = getAddAngle() / 10;
        }
        //检测陀螺仪角度是否有变化
        if(abs(getAddAngle() / 10 - LeftOffStartAngle) > 5)
        {
            FRIZY_LOG(LOG_DEBUG, "refresh data");
            addAngleCnt = 0;
            LeftOffDistance = 0;
            LeftOffStartAngle = getAddAngle() / 10;
            LeftOffStartX = escapGrid.realx * 15 / 100;
            LeftOffStartY = escapGrid.realy * 15 / 100;
        }
        else
        {
            ++addAngleCnt;
            // FRIZY_LOG(LOG_DEBUG, "addAngleCnt:%d", addAngleCnt);
            //8s内陀螺仪角度无变化
            if(addAngleCnt > 100)
            {   
                //检测位姿是否有变化
                float nowx = escapGrid.realx * 15 / 100;
                float nowy = escapGrid.realy * 15 / 100;
                LeftOffDistance = sqrtf(((LeftOffStartX - escapGrid.realx * 15 / 100) * (LeftOffStartX - escapGrid.realx * 15 / 100)) + 
                                        ((LeftOffStartY - escapGrid.realy * 15 / 100) * (LeftOffStartY - escapGrid.realy * 15 / 100)));
                FRIZY_LOG(LOG_DEBUG, "LeftOffDistance:%f", LeftOffDistance);
                if(LeftOffDistance < 0.1)
                    checkFlag = 1;
                else
                {
                    checkFlag = 0;
                    recordPosFlag = 0;
                    addAngleCnt = 0;
                }
            }
        }
        
        if(checkFlag == 1)
        {
            FRIZY_LOG(LOG_DEBUG, "Check Left Floor");
            stuckType = ESCAPE_EVENT_TYPE_LEFT_OFF;
            return true;
        }
        else
            return false;
    }

    //监测陀螺仪数据用于判定是否脱困成功           未完成
    int EscapePlan::monitorIsEscapeSuccess()
    {
        int16_t behindAccTmpThres = 0;
        static int8_t lastActOffWheel = 0;
        static int16_t curInvalidCnt = 0;
        static int16_t curActOffCnt = 0;
        static int16_t stopTimeCnt = 0;
        static int16_t posAccCnt = 0;
        static int16_t maxAcc = 0,minAcc = 0;
        static int16_t aglRatLargCnt = 0;
        static int32_t stopAccAvg = 0;
        static int32_t sumEscAcc = 0;
        int behindAccAvg = 0;
        int lCurTmp = 0, rCurTmp = 0;    
        int lSpeed = 0, rSpeed = 0;
        wheelState_t wheelState;
        wheelState = getWheelControlState();
        chassisEscape.GetSensor(&escapSensor);
        //两轮电流
        lCurTmp = escapSensor.leftWheelElec;
        rCurTmp = escapSensor.rightWheelElec;
        
        lSpeed = escapSensor.leftw;
        rSpeed = escapSensor.rightw;
        //动作切换
        if(lastActOffWheel != chassisEscape.getWheelState())
        {
            curActOffCnt = 0;
        }
        lastActOffWheel = chassisEscape.getWheelState();
        // ???

        if(chassisEscape.getWheelState() == WHEELSTOP)    //轮子停止
        {
            if(++stopTimeCnt>=2)
            {
                stopTimeCnt = 0;
                stopAccAvg = gyroAccSlidingAvg();
            }
            maxAcc = 0;
            minAcc = 0;
            sumEscAcc = 0;
            posAccCnt = 0;
            aglRatLargCnt = 0;
        }
        else if(chassisEscape.getWheelState() == WHEELBEHIND)
        {
            stopTimeCnt = 0;
            behindAccAvg = gyroAccSlidingAvg()-stopAccAvg;
            if(behindAccAvg > 0)
                posAccCnt++;    

            sumEscAcc +=behindAccAvg;
            if(maxAcc<behindAccAvg)
            {
                maxAcc = behindAccAvg;
            }
            if(behindAccAvg<minAcc)
            {
                minAcc = behindAccAvg;
            }

            if(lSpeed >= 60 || rSpeed >= 60)
            {
                if(lCurTmp > 400 || rCurTmp > 400)
                {
                    curInvalidCnt++;
                    if(curInvalidCnt > 400/20)
                    {
                        curInvalidCnt = 0;
                        return -1;//动作无效
                    }
                }
                else 
                {
                    curInvalidCnt = 0;
                    if(0 == stopAccAvg)
                        stopAccAvg = 1;
                    //不同目标速度阈值不一样
                }
            }
        }
        else 
        {
            posAccCnt = 0;
            stopTimeCnt = 0;
            curInvalidCnt = 0;
        }
    }

    //脱困动作电流和角速度监控              未完成
    void EscapePlan::escActCanMoveCheck()
    {
            static int8_t lastWheelSta = WHEELSTOP;
            static int8_t effectRateCnt = 0;
            static int8_t effectCurCnt = 0; 
            static int8_t lastWheelAct = 0;
            static int16_t checkDelayCnt = 0;//开始动作后400ms，开始记录
            //从停止到运动 lastWheelSta == WHEEL_STOP 
            if(WHEELSTOP != chassisEscape.getWheelState() && lastWheelSta!= chassisEscape.getWheelState())
            {
                checkDelayCnt = 20;//400/20
            }
            lastWheelSta = chassisEscape.getWheelState();
            if(checkDelayCnt > 0)
            {
                checkDelayCnt--;
                return;
            }
            if(WHEELSTOP == chassisEscape.getWheelState())
                return;
            
    }

    //角速度异常判定    未完成
    bool EscapePlan::abnormalAngVelocCheck(int mode_t)
    {
        bool angVelRet = false;
        static uint8_t lastEscWheelSta = WHEELSTOP;
        static int16_t actionChangCnt = 0;
        int32_t spdDiff = 0;
        int32_t spdDiffThresh = 0;
        int32_t rateAngle = 0;
        wheelState_t wheel_sta;

        rateAngle;

        wheel_sta = getWheelControlState();
        //动作变更500ms之后再开始判定
        if(lastEscWheelSta != wheel_sta.wheelState)
        {
            actionChangCnt = 25;//500/20
        }
        lastEscWheelSta = wheel_sta.wheelState;
        //累加左右自旋和左后右后的角速度
        if(wheel_sta.wheelLeftState == WHEELBEHIND)
        {
            if(wheel_sta.wheelRightState == WHEELFRONT || wheel_sta.wheelRightState == WHEELSTOP)
            {
                escActPara.leftAngVelAdd += abs(rateAngle);
            }
        }
        else if(wheel_sta.wheelRightState == WHEELBEHIND)
        {
            if(wheel_sta.wheelLeftState == WHEELFRONT || wheel_sta.wheelRightState == WHEELSTOP)
            {
                escActPara.rightAngVelAdd += abs(rateAngle);
            }
        }
        //动作变更500ms之后再开始判定
        // if(actionChangCnt > 0 && )
    }
    //俯仰和横滚异常判定  return true 异常 false 正常   
    bool EscapePlan::abnormalPitRolCheck()  
    {
        // FRIZY_LOG(LOG_DEBUG,"进入俯仰和横滚异常判定");
        static bool lastOutAvgFlag = false;
        static float lastOutAvg = 0.0f;
        bool pitRolFailRet = false;
        float pitRolEscTmp = 0.0f;  //俯仰和横滚滑动平均值
        pitRolEscTmp = gyroPitRolSlidingAvg();
        // FRIZY_LOG(LOG_DEBUG,"pitRolEscTmp:%f",pitRolEscTmp);
        // FRIZY_LOG(LOG_DEBUG,"lastOutAvgFlag:%d, pitRolLastAvg[0]:%f, fabsf(pitRolEscTmp - pitRolLastAvg[0]:%f",
        // lastOutAvgFlag, pitRolLastAvg[0], fabsf(pitRolEscTmp - pitRolLastAvg[0]));
        // if(false == lastOutAvgFlag && fabsf(pitRolEscTmp - pitRolLastAvg[0]) >= 2.0f)
        if(false == lastOutAvgFlag && fabsf(pitRolEscTmp - pitRolLastAvg[0]) >= 6.0f)
        {
            if(!lastOutAvgFlag)
            {
                angVelOutCnt = 0;
                // FRIZY_LOG(LOG_INFO,"lastOutAvgFlag is false");
            }
            lastOutAvgFlag = true;
            // FRIZY_LOG(LOG_INFO,"lastOutAvgFlag is true");
            lastOutAvg = pitRolLastAvg[0];
        }
        // FRIZY_LOG(LOG_DEBUG,"lastOutAvgFlag:%d, lastOutAvg:%f", lastOutAvgFlag, lastOutAvg);
        // FRIZY_LOG(LOG_DEBUG,"fabsf(pitRolEscTmp - lastOutAvg):%f",fabsf(pitRolEscTmp - lastOutAvg));
        // if(lastOutAvgFlag && fabsf(pitRolEscTmp - lastOutAvg) >= 2.0f)
        if(lastOutAvgFlag && fabsf(pitRolEscTmp - lastOutAvg) >= 6.0f)
        {   
            chassisEscape.GetSensor(&escapSensor);
            // FRIZY_LOG(LOG_DEBUG, "check Pit:%d", escapSensor.XAngle);
            // FRIZY_LOG(LOG_DEBUG, "angVelOutCnt:%d", angVelOutCnt);
            // if(abs(escapSensor.XAngle) > 2.5f)
            if(escapSensor.XAngle < -70.0f || escapSensor.XAngle > 40.0f)
            {
                // FRIZY_LOG(LOG_DEBUG,"PIT ABNORMAL");
                if(angVelOutCnt > 2000/50)
                // if(angVelOutCnt > 10)
                {
                    //这里面不清零
                    FRIZY_LOG(LOG_DEBUG, "esc pitRolFail[%0.1f %0.1f %d %0.1f]",pitRolEscTmp,pitRolLastAvg[0],escapSensor.XAngle,lastOutAvg);
                    // FRIZY_LOG(LOG_DEBUG,"pitRolFailRet = true");
                    pitRolFailRet = true;
                }
                else
                {
                    angVelOutCnt++;
                }
            }
            else 
            {
                // FRIZY_LOG(LOG_DEBUG,"< 2.5f");
                angVelOutCnt = 0;
                lastOutAvgFlag = false;
                pitRolFailRet = false;
            }
        }
        else 
        {
            // FRIZY_LOG(LOG_DEBUG,"< 2.0f");
            angVelOutCnt = 0;
            lastOutAvgFlag = false;
            pitRolFailRet = false;
        }
        SlidingArrayAddF(pitRolLastAvg,PITROL_LAST_AVG_NUM,pitRolEscTmp);
        return pitRolFailRet;
    }
    // 边耍缠绕检测
    void EscapePlan::SideBrushElectricCheck()
    {
        int32_t LeftSideBrushCurAvg = 0;
        int32_t RightSideBrushCurAvg = 0;
        //边耍电流采集与判定
        // chassisEscape.GetSensor(&escapSensor);

        if(escapSensor.rightSideBrushElectricity && escapSensor.leftSideBrushElectricity)
        {
            //边刷检测
            if(escapSensor.rightSideBrushElectricity >SIDEBRUSHELECTRICITYINDEX && escapSensor.leftSideBrushElectricity >SIDEBRUSHELECTRICITYINDEX)
            {
                RightSideBrushCurAvg = GlideFilterAD(RightSideBrushCurArry,AVG_WHEEL_CURARRY_NUM,escapSensor.rightSideBrushElectricity);
                LeftSideBrushCurAvg = GlideFilterAD(LeftSideBrushCurArry,AVG_WHEEL_CURARRY_NUM,escapSensor.leftSideBrushElectricity);
                sideCheck.leftSideCurrentNow = RightSideBrushCurAvg;
                sideCheck.rightSideCurrentNow = RightSideBrushCurAvg;
                if(sideCheck.leftSideCurrentNow > SIDEBRUSHELECTRICTYMAX || sideCheck.rightSideCurrentNow >SIDEBRUSHELECTRICTYMAX)
                {
                    sideCheck.sideCurrentCountOne ++;           //边刷一级阈值计数 ++
                    sideCheck.sideCurrentCountTwo = 0;          //边刷二级阈值计数清零
                    if(sideCheck.sideCurrentCountOne >= 20)
                    {
                        sideCheck.sideCurrentCountThree = 0;        //边刷三级阈值计数清零
                        sideCheck.sideCurrentCountOne = 0;          //边刷一级阈值计数清零
                        sideCheck.sideCurrentErr ++;                //边刷错误阈值 ++；
                        side_abnomal_index = true;
                        // UMAPI_CtrlSideBrush(0);
                    }
                }
                else if(sideCheck.leftSideCurrentNow <= SIDEBRUSHELECTRICTYMAX && sideCheck.rightSideCurrentNow <=SIDEBRUSHELECTRICTYMAX)
                {
                    sideCheck.sideCurrentCountTwo ++;           //边刷二级阈值计数 ++
                    if(sideCheck.sideCurrentCountTwo > 40)
                    {
                        sideCheck.sideCurrentCountOne = 0;          //边刷一级阈值计数清零
                        sideCheck.sideCurrentCountTwo = 0;          //边刷二级阈值计数清零
                        sideCheck.sideCurrentCountThree = 0;        //边刷三级阈值计数清零
                        sideCheck.sideCurrentErr = 0;               //边刷错误阈值清零
                        side_abnomal_index = false;
                    }
                }
                else if(sideCheck.leftSideCurrentNow > SIDEBRUSHELECTRICTYMAX*1.5 || sideCheck.rightSideCurrentNow >SIDEBRUSHELECTRICTYMAX*1.5)     
                {
                    sideCheck.sideCurrentCountThree ++;             //边刷三级阈值计数++
                    if(sideCheck.sideCurrentCountThree >= 5)
                    {
                        sideCheck.sideCurrentCountThree = 0;                //边刷三级阈值计数清零
                        sideCheck.sideCurrentErr = 5;         //边刷错误阈值赋值
                        side_brush_alert = true ;
                    }
                }                
            }

        
        }
    }
    //电流异常检测              
    bool EscapePlan::abnormalElectricCheck()
    {
        static bool curOutFlagL = false;
        static bool curOutFlagR = false; 
        bool judRetL = false;
        bool judRetR = false;
        bool curSumRetL = false;
        bool curSumRetR = false;
        int32_t escLeftCurAvg = 0;
        int32_t escRightCurAvg = 0;
        
        wheelState_t wheelSta_tmp;
        wheelSta_tmp = getWheelControlState();
        //电流采集与判定
        chassisEscape.GetSensor(&escapSensor);
        // FRIZY_LOG(LOG_DEBUG,"left cur:%f,right cur:%f", escapSensor.leftWheelElec, escapSensor.rightWheelElec);
        if(escapSensor.leftWheelElec && escapSensor.rightWheelElec)
        {
            //左轮
            if(WHEELSTOP != wheelSta_tmp.wheelLeftState && escapSensor.leftWheelElec > 50)
            {
                // FRIZY_LOG(LOG_DEBUG,"left cur:%f", escapSensor.leftWheelElec);
                //求滑动平均值
                escLeftCurAvg = GlideFilterAD(escLeftCurArry,AVG_WHEEL_CURARRY_NUM,escapSensor.leftWheelElec);
                // FRIZY_LOG(LOG_DEBUG,"leftCurSlidAvg:%d", escLeftCurAvg);
                // 数组中填满数据之后才开始判定
                // FRIZY_LOG(LOG_DEBUG,"leftCurSlidAvg num:%d", escCkeckPara.arryAvgCurCountL);
                if(escCkeckPara.arryAvgCurCountL > AVG_WHEEL_CURARRY_NUM*AVG_LAST_WHEEL_CURARRY_NUM)
                {
                    // FRIZY_LOG(LOG_DEBUG,"数组中填满数据之后才开始判定");
                    // FRIZY_LOG(LOG_DEBUG,"10*escLeftCurAvg:%d,escLastLeftCurAvg[0]:%d", 10*escLeftCurAvg, escLastLeftCurAvg[0]);
                    // FRIZY_LOG(LOG_DEBUG, "10*escLeftCurAvg)/escLastLeftCurAvg[0]:%d", 10*escLeftCurAvg / escLastLeftCurAvg[0]);
                    //当前的值和1.6s 前的滑动平均值变化大于1.5倍
                    if((10*escLeftCurAvg)/escLastLeftCurAvg[0] >= WHEEL_CURRENT_ABNORMAL)
                    {
                        if(!curOutFlagL)
                        {
                            abnrCurrOutCntL = 0;
                        }
                        curOutFlagL = true;
                        // printf("out leftCur %d %d\n",escLeftCurAvg,escCkeckPara.arryAvgCurCountL);
                    }
                    // FRIZY_LOG(LOG_DEBUG, "abnrCurrOutCntL:%d",abnrCurrOutCntL);
                    if(curOutFlagL)
                    {   FRIZY_LOG(LOG_DEBUG,"left cur:%f", escapSensor.leftWheelElec);
                        if(escapSensor.leftWheelElec > WHEEL_LEFT_CURRENT_LIMIT)
                        {
                            FRIZY_LOG(LOG_DEBUG, "abnrCurrOutCntL:%d", abnrCurrOutCntL);
                            if(abnrCurrOutCntL > 800/20)
                            {
                                //这里面不清零
                                judRetL = true;
                                curOutFlagL = false;
                            }
                            else
                            {
                                abnrCurrOutCntL++;
                            }
                        }
                        else
                        {
                            judRetL = false;
                            abnrCurrOutCntL = 0;
                        }
                    }
                    else
                    {
                        abnrCurrOutCntL = 0;
                        judRetL = false;
                    }
                }
                else
                {
                    // FRIZY_LOG(LOG_DEBUG,"数组元素+1");
                    escCkeckPara.arryAvgCurCountL++;
                }
                SlidingArrayAddI(escLastLeftCurAvg,AVG_LAST_WHEEL_CURARRY_NUM,escLeftCurAvg);
            }
            else 
            {
                abnrCurrOutCntL = 0;
                curOutFlagL = false;
                judRetL = false;
            }
            //右轮
            if(WHEELSTOP != wheelSta_tmp.wheelRightState && escapSensor.rightWheelElec)
            {
                // FRIZY_LOG(LOG_DEBUG,"right cur:%f", escapSensor.rightWheelElec);
                //求滑动平均值
                escRightCurAvg = GlideFilterAD(escRightCurArry,AVG_WHEEL_CURARRY_NUM,escapSensor.rightWheelElec);
                // FRIZY_LOG(LOG_DEBUG,"rightCurSlidAvg:%d", escRightCurAvg);
                // FRIZY_LOG(LOG_DEBUG,"rightCurSlidAvg num:%d", escCkeckPara.arryAvgCurCountR);
                if(escCkeckPara.arryAvgCurCountR > AVG_WHEEL_CURARRY_NUM*AVG_LAST_WHEEL_CURARRY_NUM)
                {
                    // FRIZY_LOG(LOG_DEBUG,"数组中填满数据之后才开始判定");
                    //当前的值 1.6s 前的滑动平均值变化大于1.5倍
                    // FRIZY_LOG(LOG_DEBUG,"10*escRightCurAvg:%d,escLastRightCurAvg[0]:%d", 10*escRightCurAvg, escLastRightCurAvg[0]);
                    // FRIZY_LOG(LOG_DEBUG, "10*escRightCurAvg/escLastRightCurAvg[0]:%d", 10*escRightCurAvg / escLastRightCurAvg[0]);
                    if(10*escRightCurAvg/escLastRightCurAvg[0] >= WHEEL_CURRENT_ABNORMAL)
                    {
                        if(!curOutFlagR)
                        {
                            abnrCurrOutCntR = 0;
                        }
                        curOutFlagR = true;
                        // printf("out rightCur %d %d\n",escRightCurAvg,escCkeckPara.arryAvgCurCountR);
            
                    }
                    if(curOutFlagR)
                    {   FRIZY_LOG(LOG_DEBUG,"right cur:%f", escapSensor.rightWheelElec);
                        if(escapSensor.rightWheelElec > WHEEL_RIGHT_CURRENT_LIMIT)
                        {
                            FRIZY_LOG(LOG_DEBUG, "abnrCurrOutCntR:%d", abnrCurrOutCntR);
                            if(abnrCurrOutCntR > 800/20)
                            {
                                //这里面不清零
                                judRetR = true;
                                curOutFlagR = false;
                            }
                            else
                            {
                                abnrCurrOutCntR++;
                            }
                        }
                        else
                        {
                            abnrCurrOutCntR = 0;
                            judRetR = false;
                        }
                    }
                    else
                    {
                        abnrCurrOutCntR = 0;
                        judRetR = false;
                    }
                }
                else
                {
                    escCkeckPara.arryAvgCurCountR++;
                }
                SlidingArrayAddI(escLastRightCurAvg,AVG_LAST_WHEEL_CURARRY_NUM,escRightCurAvg);
            }
            else 
            {
                abnrCurrOutCntR = 0;
                curOutFlagR = false;
                judRetR = false;
            }
        }        
        else
        {
            curOutFlagL = false;
            curOutFlagR = false;
            abnrCurrOutCntR = 0;
            abnrCurrOutCntL = 0;
        }
        if(judRetR || judRetL || curSumRetR ||curSumRetL)
        {
            abnrCurrOutCntR = 0;
            abnrCurrOutCntL = 0;
            // escCkeckPara.arryAvgCurCountR = 0;
            // escCkeckPara.arryAvgCurCountL = 0;
            return true;
        }
        else
        {
            return false;
        }
    }


    
    //按给定距离后退
    void EscapePlan::wheelBackDist(int speed, int dis)//速度 距离mm
    {
        int tmp_time;
        tmp_time = dis * 10 / abs(speed);
        while(tmp_time)
        {
            chassisEscape.chassisSpeed(speed, speed, 1);
            usleep(100 * 1000);
            tmp_time --;
        }
        chassisEscape.chassisSpeed(0, 0, 1);
        return;
    }
    
    //按给定时间后退
    void EscapePlan::wheelCtrlStraight(int speed,int walkTime)//速度  时间ms
    {
            chassisEscape.chassisSpeed(speed, speed, 1);
            usleep(walkTime * 1000);
            chassisEscape.chassisSpeed(0, 0, 1);
    }

    //自旋                                                  //0:相对角度 1:绝对角度
    void EscapePlan::escSpin(int speed, int dir, float angle, int relatAbs)
    {
        float aimforward;
        long long rotateStartTime = getTime();
        // chassisEscape.GridPoint(&escapGrid);
        chassisEscape.GetSensor(&escapSensor);
        if(!dir)
        {
            if(!relatAbs)
            {
                aimforward = (360-gyo_angle_*180/_Pi) - angle;
                if(aimforward <= 0)
                {
                    aimforward = 360 + aimforward;
                }
            }
            else
            {
                aimforward = angle;
            }
            while(GLOBAL_CONTROL == WHEEL_RUN)
            {
                if(getTime() - rotateStartTime >= 8000)
                {
                    FRIZY_LOG(LOG_INFO, "spin timeout");
                    return;
                }
                FRIZY_LOG(LOG_INFO,"aimforward:%f, gyo_angle_:%f", aimforward, (360-gyo_angle_*180/_Pi));
                chassisEscape.chassisSpeed(-speed,speed,1);
                chassisEscape.GetSensor(&escapSensor);
                if(fabs(aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(aimforward - (360-gyo_angle_*180/_Pi)) > 350)
                {
                    FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");
                    chassisEscape.chassisSpeed(0, 0, 1);
                    return;
                }
                usleep(20 * 1000);
            }
        }
        else
        {
            if(!relatAbs)
            {
                aimforward = (360-gyo_angle_*180/_Pi) + angle;
                if(aimforward >= 360)
                {
                    aimforward = aimforward - 360;
                }
            }
            else
            {
                aimforward = angle;
            }
            while(GLOBAL_CONTROL == WHEEL_RUN)
            {
                if(getTime() - rotateStartTime >= 8000)
                {
                    FRIZY_LOG(LOG_INFO, "spin timeout");
                    return;
                }
                FRIZY_LOG(LOG_INFO,"aimforward:%f, gyo_angle_:%f", aimforward, (360-gyo_angle_*180/_Pi));
                chassisEscape.chassisSpeed(speed, -speed,1);
                chassisEscape.GetSensor(&escapSensor);
                if(fabs(aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(aimforward - (360-gyo_angle_*180/_Pi)) > 350)
                {
                    FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");   
                    chassisEscape.chassisSpeed(0, 0, 1);  
                    return;
                }
                usleep(20 * 1000);
            }
        }

    }

//单边旋                                                          //0:相对角度 1:绝对角度
    void EscapePlan::singleRotate(int speedL, int speedR, float angle, int relatAbs)
    {
        float aimforward;
        long long rotateStartTime = getTime();
        // chassisEscape.GridPoint(&escapGrid);
        chassisEscape.GetSensor(&escapSensor);
        if(speedL < speedR)
        {
            if(!relatAbs)
            {
                aimforward = (360-gyo_angle_*180/_Pi) - angle;
                if(aimforward <= 0)
                {
                    aimforward = 360 + aimforward;
                }
            }
            else
            {
                aimforward = angle;
            }
            FRIZY_LOG(LOG_DEBUG,"start gyo_angle_:%f,aimforward:%f",360-gyo_angle_*180/_Pi, aimforward);
            while(GLOBAL_CONTROL == WHEEL_RUN)
            {
                if(getTime() - rotateStartTime >= 8000)
                {
                    FRIZY_LOG(LOG_INFO, "single rotate timeout");
                    return;
                }
                FRIZY_LOG(LOG_INFO,"aimforward:%f, gyo_angle_:%f", aimforward, (360-gyo_angle_*180/_Pi));
                chassisEscape.chassisSpeed(speedL, speedR, 1);
                chassisEscape.GetSensor(&escapSensor);
                if(fabs(aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(aimforward - (360-gyo_angle_*180/_Pi)) > 350)
                {
                    FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
                    chassisEscape.chassisSpeed(0, 0, 1);
                    return;
                }
                usleep(20 * 1000);
            }
        }
        else
        {
            if(!relatAbs)
            {
                aimforward = (360-gyo_angle_*180/_Pi) + angle;
                if(aimforward >= 360)
                {
                    aimforward = aimforward - 360;
                }
            }
            else
            {
                aimforward = angle;
            }
            FRIZY_LOG(LOG_DEBUG,"start gyo_angle_:%f,aimforward:%f",(360-gyo_angle_*180/_Pi), aimforward);
            while(GLOBAL_CONTROL == WHEEL_RUN)
            {
                if(getTime() - rotateStartTime >= 8000)
                {
                    FRIZY_LOG(LOG_INFO, "single rotate timeout");
                    return;
                }
                FRIZY_LOG(LOG_INFO,"aimforward:%f, gyo_angle_:%f", aimforward, (360-gyo_angle_*180/_Pi));
                chassisEscape.chassisSpeed(speedL, speedR, 1);
                chassisEscape.GetSensor(&escapSensor);
                if(fabs(aimforward - (360-gyo_angle_*180/_Pi)) < 10 || fabs(aimforward - (360-gyo_angle_*180/_Pi)) > 350)
                {
                    FRIZY_LOG(LOG_DEBUG, "ROTATE SUCCESSFUL");
                    chassisEscape.chassisSpeed(0, 0, 1);
                    return;
                }
                usleep(20 * 1000);
            }
        }
        
    }
    
    //用于记录记录10s前的机器姿态
    int EscapePlan::recordPitRol_10s()
    {
        float tmp;
        if(record_time == 0)
        {
            record_time = getTime();
            escActPara.escPre10PrePit = getPitRol(0);
            escActPara.escPre10PreRol = getPitRol(1);
            chassisEscape.GetSensor(&escapSensor);
            escActPara.escPre10PrePitRol = gyroPitRolSlidingAvg();
            // escActPara.escPre10PrePitRol = sqrtf(escapSensor.XAngle * escapSensor.XAngle + escapSensor.YAngle * escapSensor.YAngle);
            FRIZY_LOG(LOG_DEBUG,"0s pre pit:%f,rol:%f,pitrol:%f",escActPara.escPre10PrePit, escActPara.escPre10PreRol, escActPara.escPre10PrePitRol);
        }
        if(getTime() - record_time >= 10 * 1000) 
        {   
            // FRIZY_LOG(LOG_DEBUG, "getTime() - record_time >= 500");
            if(recordPit.size() < 10)
                recordPit.push_back(getPitRol(0));
            else
            {
                vector<float>::iterator it_1 = recordPit.begin();
                recordPit.erase(it_1);
                recordPit.push_back(getPitRol(0));
                sort(recordPit.begin(), recordPit.end(), [](const float& a, 
                const float  &b){
                    return a < b;
                    });
                // for(auto i : recordPit)
                // {
                //     cout << i << " ";
                // }
                // cout << endl;
                escActPara.escPre10PrePit = (recordPit.at(3) + recordPit.at(4) + recordPit.at(5)) / 3;
                // float a = 0.0f;
                // FRIZY_LOG(LOG_DEBUG,"escActPara.escPre10PrePit:%f",escActPara.escPre10PrePit);
                // for(auto i : recordPit)
                // {
                //     a += i;
                // }
                // escActPara.escPre10PrePit = a/recordPit.size();
                // FRIZY_LOG(LOG_DEBUG,"sum:%f",a);
                // FRIZY_LOG(LOG_DEBUG,"recordPit.size:%d",recordPit.size());
            }

            if(recordRol.size() < 10)
                recordRol.push_back(getPitRol(1));
            else
            {
                vector<float>::iterator it_2 = recordRol.begin();
                recordRol.erase(it_2);
                recordRol.push_back(getPitRol(1));
                sort(recordRol.begin(), recordRol.end(), [](const float& a, 
                const float  &b){
                    return a < b;
                    });
                // for(auto j : recordPit)
                // {
                //     cout << j << " ";
                // }
                // cout << endl;
                escActPara.escPre10PreRol = (recordRol.at(3) + recordRol.at(4) + recordRol.at(5)) / 3;
                // float b = 0.0f;
                // FRIZY_LOG(LOG_DEBUG,"escActPara.escPre10PreRol:%f",escActPara.escPre10PreRol);
                // for(auto j : recordRol)
                // {
                //     b += j;
                // }
                // escActPara.escPre10PreRol = b/recordRol.size();
                // FRIZY_LOG(LOG_DEBUG,"sum:%f",b);
                // FRIZY_LOG(LOG_DEBUG,"recordRol.size:%d",recordRol.size());
            }
            if(recordPitRol.size() < 10)
            {
                chassisEscape.GetSensor(&escapSensor);
                // tmp = sqrtf(escapSensor.XAngle * escapSensor.XAngle + escapSensor.YAngle * escapSensor.YAngle);
                tmp = gyroPitRolSlidingAvg();
                recordPitRol.push_back(tmp);
            }
            else 
            {
                vector<float>::iterator it_3 = recordPitRol.begin();
                recordPitRol.erase(it_3);
                chassisEscape.GetSensor(&escapSensor);
                // tmp = sqrtf(escapSensor.XAngle * escapSensor.XAngle + escapSensor.YAngle * escapSensor.YAngle);
                tmp = gyroPitRolSlidingAvg();
                recordPitRol.push_back(tmp);
                sort(recordPitRol.begin(), recordPitRol.end(), [](const float& a, 
                const float  &b){
                    return a < b;
                    });
                // for(auto k : recordPitRol)
                // {
                //     cout << k << " ";
                // }
                // cout << endl;
                escActPara.escPre10PrePitRol = (recordPitRol.at(3) + recordPitRol.at(4) + recordPitRol.at(5)) / 3;
                // float c = 0.0f;
                // FRIZY_LOG(LOG_DEBUG,"escActPara.escPre10PrePitRol:%f",escActPara.escPre10PrePitRol);
                // for(auto k : recordPitRol)
                // {
                //     c += k;
                // }
                // escActPara.escPre10PrePitRol = c/recordPitRol.size();
                // FRIZY_LOG(LOG_DEBUG,"sum:%f",c);
                // FRIZY_LOG(LOG_DEBUG,"recordPitRol.size:%d",recordPitRol.size());
            }
            FRIZY_LOG(LOG_DEBUG,"current pit:%d,rol:%d,pitrol:%f",getPitRol(0),getPitRol(1),tmp);
            FRIZY_LOG(LOG_DEBUG,"each 10s pre pit:%f,rol:%f,pitrol:%f",escActPara.escPre10PrePit, escActPara.escPre10PreRol, escActPara.escPre10PrePitRol);
            //更新时间
            record_time = getTime();
        }
    }
    
    // 脱困获取相对变化俯仰横滚角度         
    float EscapePlan::getMEMSRelatAngle(int index)
    {
        float temp_pit,temp_rol;
        if(index == 0)
        {
            temp_pit = getPitRol(0);
            FRIZY_LOG(LOG_DEBUG,"current pit:%f,10s pre pit:%f", temp_pit, escActPara.escPre10PrePit);
            return (temp_pit- escActPara.escPre10PrePit);
        }
        else if(index == 1)
        {
            temp_rol = getPitRol(1);
            FRIZY_LOG(LOG_DEBUG,"current rol:%f,10s pre rol:%f", temp_rol, escActPara.escPre10PreRol);
            return (temp_rol - escActPara.escPre10PreRol);
        }
        else 
            return 0.0f;
    }

    //进入脱困前和执行脱困动作后的差值
    float EscapePlan::getMEMSActChangAngle(int index)
    {
        if(index == 0)
        {
            FRIZY_LOG(LOG_DEBUG,"current pit:%f, before escape pit:%f", getPitRol(0), escActPara.escActionPrePit);
            return getPitRol(0) - escActPara.escActionPrePit;
        }
        else if(index == 1)
        {
            FRIZY_LOG(LOG_DEBUG,"current rol:%f, before escape rol:%f", getPitRol(1), escActPara.escActionPreRol);
            return getPitRol(1) - escActPara.escActionPreRol;
        }
        else 
            return 0.0f;

    }

    // 针对架空脱困和尾部翘起卡死脱困，如果数据正常直接切入脱困   
    bool EscapePlan::escActStepBehind()
    {
        static int8_t pitRolBCnt = 0;
        if(escActPara.escPreTypeRec & 0x40)//机器前面卡住，后面翘起(欧式家具卡)
        {
            if(fabsf(getMEMSRelatAngle(0)) < 5.0f && fabsf(getMEMSRelatAngle(1)) < 5.0f)
            // if(fabsf(getMEMSRelatAngle(0)) < 2.5f && fabsf(getMEMSRelatAngle(1)) < 2.5f)
            {
                if(++pitRolBCnt > 100/20)
                {
                    pitRolBCnt = 0;
                    FRIZY_LOG(LOG_DEBUG, "escPreTypeRec 0x40 ok");
                    return true;
                }
                else 
                    pitRolBCnt = 0;
            }
        }
        else if(escActPara.escPreTypeRec & 0x20)//机器前面翘起(风扇座架起)
        {
            if(fabsf(getMEMSRelatAngle(0)) < 5.0f && fabsf(getMEMSRelatAngle(1)) < 5.0f)
            // if(fabsf(getMEMSRelatAngle(0))<1.5f && fabsf(getMEMSRelatAngle(1))<1.5f)
            {
                if(++pitRolBCnt > 100/20)
                {
                    FRIZY_LOG(LOG_DEBUG, "escPreTypeRec 0x20 ok");
                    pitRolBCnt = 0;
                    return true;
                }
            }
            else
            {
                pitRolBCnt = 0;
            }
        }
        //监控进入脱困后机器移动的角度
        int16_t angleChangTme = 0;
        angleChangTme = abs(enterAddAngle - getAddAngle());
        FRIZY_LOG(LOG_DEBUG, "angleChangTme:%d, escAngleMaxChange:%d", angleChangTme, escActPara.escAngleMaxChange);
        if(angleChangTme > escActPara.escAngleMaxChange)
        {
            escActPara.escAngleMaxChange = angleChangTme;
        }
        return false;
    }

    //执行脱困动作前对陀螺仪数据分析
    //分析是哪种卡住，进入不同的脱困动作     
    void EscapePlan::escStepPreGyroAnal()
    {
        FRIZY_LOG(LOG_DEBUG,"into escStepPreGyroAnal");
        static int8_t escPreStopTime = 0;
        float tmpEscPrePR = 0.0f;
        if(WHEELSTOP == chassisEscape.getWheelState())
        {
            if(++escPreStopTime>10)
            {
                escPreStopTime = 0;
                if(getTime() - escStartTime > 8000)     //进入脱困的时间大于8s
                {
                    escEnterIntervalCnt = 0;
                    escActPara.escActStepRecord = ESC_STEP_0;
                    escActPara.enterStep0PreStep = ESCACT_START;        
                }
                else       //进入脱困的时间小于8s   短时进入脱困
                {
                    escEnterIntervalCnt++;
                    escActPara.escActStepRecord = ESC_STEP_1;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    FRIZY_LOG(LOG_DEBUG,"esc < 8s");
                }
                if(escEnterIntervalCnt > 8)//短时进入脱困超过8次，直接判定为脱困失败
                {
                    escActPara.escActStepRecord = ESC_STEP_FAILD;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    FRIZY_LOG(LOG_INFO,"escEnterIntervalCnt:%d",escEnterIntervalCnt);
                }
                escActPara.escStopPrePitRol = gyroPitRolSlidingAvg();//记录机器陀螺仪姿态
                escActPara.escActionPrePit = getPitRol(0);          //俯仰 
                escActPara.escActionPreRol = getPitRol(1);          //横滚
                FRIZY_LOG(LOG_DEBUG,"before act pit:%f,rol:%f,pitrol:%f", escActPara.escActionPrePit, escActPara.escActionPreRol, escActPara.escStopPrePitRol);
                //是靠墙动作进入的脱困，并且姿态相差较大，这里判定为上了风扇座或者U型椅子的脚
                tmpEscPrePR = fabsf(escActPara.escStopPrePitRol - escActPara.escPre10PrePitRol);        //当前姿态与卡住前10s的机器姿态
                FRIZY_LOG(LOG_DEBUG,"tmpEscPrePR:%f = escStopPrePitRol:%f - escPre10PrePitRol:%f", tmpEscPrePR, escActPara.escStopPrePitRol, escActPara.escPre10PrePitRol);
                FRIZY_LOG(LOG_DEBUG,"10s esc pit:%f,rol:%f,pitrol:%f", escActPara.escPre10PrePit, escActPara.escPre10PreRol, escActPara.escPre10PrePitRol);
                if(tmpEscPrePR > 4.0f || fabsf(getMEMSRelatAngle(0)) > 5.0f || fabsf(getMEMSRelatAngle(1)) > 5.0f)
                {
                    if(getMEMSRelatAngle(0) <-4.0f)//俯仰，前面翘起
                    {
                        escActPara.escPreTypeRec = 0x20;
                    }
                    else if(getMEMSRelatAngle(0) > 4.0f)//俯仰，前面压下
                    {
                        if(getMEMSRelatAngle(1) >= 4.0f)//右前被压下
                        {
                            escActPara.escPreTypeRec = 0x41;
                        }
                        else if(getMEMSRelatAngle(1) <= -4.0f)//左前被压下
                        {
                            escActPara.escPreTypeRec = 0x42;
                        }
                        else//前被压下
                        {
                            escActPara.escPreTypeRec = 0x40;
                        }
                    }
                    // saveEnterEscapMode =  TR_BASIS_PICTCH;   
                }

                if(escActPara.escPreTypeRec & 0x20
                 || stuckType == ESCAPE_EVENT_TYPE_LEFT_OFF)//架空脱困
                {
                    FRIZY_LOG(LOG_DEBUG,"ESCAPE TYEP : ESCAPE_EVENT_TYPE_LEFT_OFF");
                    escActPara.escActStepRecord = ESC_STEP_OVERHEAD;//先尝试架空脱困
                    escActPara.enterStep0PreStep = ESCACT_START;
                    
                }
                else if(escActPara.escPreTypeRec & 0x40
                     || stuckType == ESCAPE_EVENT_TYPE_STUCK)//底部卡住
                {
                    escActPara.escActStepRecord = ESC_STEP_STUCK_HEAD;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    FRIZY_LOG(LOG_DEBUG,"ESCAPE TYEP : ESC_STEP_STUCK_HEAD");
                }
            }
            else if(escPreStopTime == 5)
            {
                FRIZY_LOG(LOG_INFO,"sliding average:%f,before stick 10s:%f",gyroPitRolSlidingAvg(),escActPara.escPre10PrePitRol);
            }
        }
        else 
        {
            if(getTime() - escActPara.escActExecuTime >= 2000)
            {
                chassisEscape.chassisSpeed(0, 0, 1);
            }
            escPreStopTime = 0;
        }
    }
    
    //记录脱困前的动作
    void EscapePlan::recordActBeforeEsc()
    {
        int wheelSta = WHEELSTOP;
        wheelSta = chassisEscape.getWheelState();
        if(wheelSta != WHEELSTOP)
            SlidingArrayAddI(escPreActionArry, 3, wheelSta);
    }

    /*
    * 脱困步骤0 后退
    */
    void EscapePlan::escActStep_0()
    {
        static int8_t escBeStep3Cnt = 0;
        switch(escActPara.escActState)
        {
            case 0:     //1后退
            {   
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    //如果是动作有效，进入后退动作
                    if(1 == escActPara.enterStep0Sta)
                    {
                        wheelBackDist(-200, 80);//正常的速度后退
                        escActPara.escUnmovableRateCnt = 0;
                    }
                    else 
                    {
                        if(escActPara.enterStep0PreStep == ESC_STEP_ABERRANT)
                        {
                            wheelBackDist(-200, 80);//正常的速度后退
                        }
                        else if(escActPara.enterStep0PreStep == ESC_STEP_3)
                        {
                            wheelCtrlStraight(-300, 800);//后退
                            escBeStep3Cnt++;
                        }
                        else if(escActPara.enterStep0PreStep == ESC_STEP_1 || escActPara.enterStep0PreStep == ESC_STEP_2)//由第一二步到这里的
                        {
                            
                            wheelCtrlStraight(-240, 800);//后退
                        }
                        else 
                        {
                            escBeStep3Cnt = 0;
                            wheelBackDist(-200, 80);//正常的速度后退
                        }
                    }
                    escActPara.enterStep0Sta = 0;
                    escActPara.escActState = ESCACT_BEHIND_WAIT;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 1500)
                    {
                        chassisEscape.chassisSpeed(0, 0, 1);
                    }
                }
            }
            break;

            case ESCACT_BEHIND_WAIT:     //2 后退等待
            {
                if(-1 == escActPara.actionRetDelay || (WHEELSTOP == chassisEscape.getWheelState()))//后退动作无效
                {
                    if(escActPara.enterStep0PreStep == ESC_STEP_3)//由第三步到这里的
                    {
                        escActPara.escActStepRecord = ESC_STEP_4; 
                    }
                    else if(escActPara.enterStep0PreStep == ESC_STEP_2)//由第二步到这里的
                    {
                        escActPara.escActStepRecord = ESC_STEP_3;
                    }
                    else if(escActPara.enterStep0PreStep == ESC_STEP_1)//由第一步到这里的
                    {
                        escActPara.escActStepRecord = ESC_STEP_2;
                    }
                    else
                    {
                        escActPara.escActStepRecord = ESC_STEP_1;
                    }
                    if(escActPara.escActValidCnt>=4)//动作3次都成功了，后退失败，也判定为脱困成功
                    {
                        escActPara.escActStepRecord = ESC_STEP_SUCCESS;
                        escActPara.enterSuccPreStep = ESC_STEP_1;
                        
                        FRIZY_LOG(LOG_DEBUG,"ValidCnt>4 success");
                    }
                    escActPara.escActState = ESCACT_START;
                }    
                else if(1 == escActPara.actionRetDelay)//后退动作有效
                {
                    escActPara.enterSuccPreStep = ESC_STEP_0;
                    escActPara.escActStepRecord = ESC_STEP_SUCCESS;
                    escActPara.escActState = ESCACT_START;
                }
                if(escBeStep3Cnt >= 3)
                {
                escBeStep3Cnt = 0;
                FRIZY_LOG(LOG_DEBUG,"escBeStep3Cnt>3 faild\n");
                }
            }
            break;

            default:
                escActPara.escActState = ESCACT_START;
            break;            
        }
    }
    
    /*
    * 动作1之后的判定中间操作
    * cnt 动作次数
    * lastAct 上一个动作 
    */
    void EscapePlan::escActStep1WaitTmp(int cnt, int lastAct)
    {
        if(escActPara.actionRetDelay == -1)//自旋无效
        {
            if(getTime() - escActPara.escActExecuTime >= 1500)
            {
                if(cnt <= 1)//尝试自旋两次
                {
                    escActPara.escActState = (escapeAction_t)lastAct;
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
                else //自旋两次都没有成功
                {
                    escActPara.escActStepRecord = ESC_STEP_2;//进入脱困第二步骤
                    escActPara.escActState = ESCACT_START;
                }
            }
        }
        else if(escActPara.actionRetDelay == 1 || chassisEscape.getWheelState() == WHEELSTOP)
        {
            escActPara.escActStepRecord = ESC_STEP_0;//切入后退
            escActPara.enterStep0PreStep = ESC_STEP_1;
            escActPara.escActState = ESCACT_START;
            escActPara.enterStep0Sta = 1;
            escActPara.escActValidCnt++;
        }
    }

    /*          
    * 脱困步骤1 根据脱困前的动作进行尝试动作
    * actResu：动作的结果 -1表示无效动作 1表示有效动作
    */
    void EscapePlan::escActStep_1()
    {
        switch(escActPara.escActState)
        {
            case 0:
            {
                //根据脱困前的动作来决定之后的动作
                if(escPreActionArry[2] == WHEELLEFTSPIN)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else if(escPreActionArry[2] == WHEELRIGHTSPIN)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else if(escPreActionArry[2] == WHEELFRONT)
                {
                    if(RIGHTAW == getAlongWallDir())//之前是右沿墙
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else if(LEFTAW == getAlongWallDir())
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_BEHIND;
                    }
                    
                }
                else //尝试左后 右后
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
            }
            break;

            case ESCACT_SPIN_RIGHT://5右自旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escSpin(300, 1, 30, 0);
                    escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
                    escActPara.rightSpinCnt++;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 1500)
                        chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;

            case ESCACT_SPIN_RIGHT_WAIT://6右自旋等待
            {
                escActStep1WaitTmp(escActPara.rightSpinCnt,ESCACT_SPIN_RIGHT);
            }
            break;

            case ESCACT_SPIN_LEFT:   //3左自旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escSpin(300, 0, 30, 0);
                    escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
                    escActPara.leftSpinCnt++;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 1500)
                        chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;

            case ESCACT_SPIN_LEFT_WAIT://4左自旋等待
            {
                escActStep1WaitTmp(escActPara.leftSpinCnt,ESCACT_SPIN_LEFT);
            }
            break;

            case ESCACT_NUILROT_LEFT://7 左单边旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.leftBehindCnt++;
                    singleRotate(-300, -60, 30, 0);
                    escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 1500)
                        chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;

            case ESCACT_NUILROT_LEFT_WAIT:// 8左单边旋等待
            {
                escActStep1WaitTmp(escActPara.leftBehindCnt,ESCACT_NUILROT_LEFT);
            }
            break;

            case ESCACT_NUILROT_RIGHT://右单边旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.leftBehindCnt++;
                    singleRotate(-60, -300, 30, 0);
                    escActPara.escActState = ESCACT_NUILROT_RIGHT_WAIT;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 1500)
                        chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;

            case ESCACT_NUILROT_RIGHT_WAIT://右单边旋等待
            {
                escActStep1WaitTmp(escActPara.rightBehindCnt,ESCACT_NUILROT_RIGHT);
            }
            break;

            default:
                escActPara.escActState = ESCACT_START;
            break;
        }
    }
    
    /*
    * 动作2 之后的判定中间操作
    * faildAct 失败之后要切换的动作
    */
    void EscapePlan::escActStep2WaitTmp(int failedAct)
    {
        int timeDiffStep2 = 0;
        int angleActDiffStep2 = 0;
        timeDiffStep2 = getTime() - escActPara.escActExecuTime;
        angleActDiffStep2 = abs(escActPara.escActPreAngle - getAddAngle())/10;
        if(-1 == escActPara.actionRetDelay || (timeDiffStep2 >= 2000 && angleActDiffStep2<5))//动作无效
        {
            if(timeDiffStep2 >= 1500)
            {
                if(ESCACT_SPIN_LEFT == failedAct)//下一步发出的是左自旋指令
                {
                    if(escActPara.leftSpinCnt >= 1)//左自旋 动作次数计数
                    {
                        failedAct = ESCACT_NUILROT_LEFT;
                    }   
                }
                else if(ESCACT_SPIN_RIGHT == failedAct)//下一步发出的是右自旋指令
                {
                    if(escActPara.rightSpinCnt >= 1)//右自旋 动作次数计数
                    {
                        failedAct = ESCACT_NUILROT_RIGHT;
                    }
                }
                else if(ESCACT_NUILROT_LEFT == failedAct)//下一步发出的是左后旋指令
                {
                    if(escActPara.leftBehindCnt >= 1)//左后 动作次数计数
                    {
                        failedAct = ESCACT_SPIN_LEFT;
                    }
                }
                else if(ESCACT_NUILROT_RIGHT == failedAct)//下一步发出的是右后旋指令
                {
                    if(escActPara.rightBehindCnt >= 1)//右后 动作次数计数
                    {
                        failedAct = ESCACT_SPIN_RIGHT;
                    }
                }
                //所有动作都尝试了一次
                if(escActPara.leftSpinCnt && escActPara.rightSpinCnt && escActPara.leftBehindCnt && escActPara.rightBehindCnt)
                {
                    escActPara.escActStepRecord = ESC_STEP_0;
                    escActPara.escActState = ESCACT_START;
                    escActPara.enterStep0PreStep = ESC_STEP_2;
                    escActPara.enterStep0Sta = -1;//标记是动作失败切回的后退
                    chassisEscape.chassisSpeed(0, 0, 1);
                    FRIZY_LOG(LOG_DEBUG, "escStep2 4 dir %d %d",escActPara.escUnmovableRateCnt,escActPara.escUnmovableCurCnt);
                }
                else
                {
                    escActPara.escActState = (escapeAction_t)failedAct;
                }
                escActPara.escActPreAngle =getAddAngle();
                FRIZY_LOG(LOG_DEBUG, "escActStep2 outTime->%d",escActPara.escActState);
            }
            escActPara.escActValidCnt = 0;
        }
        else if(escActPara.actionRetDelay == 1|| chassisEscape.getWheelState() == WHEELSTOP)//动作有效
        {
            FRIZY_LOG(LOG_DEBUG, "step2 enter succ");
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;
            escActPara.enterSuccPreStep = ESC_STEP_2;
            escActPara.escActState = ESCACT_START;
            escActPara.escActValidCnt++;
            if(escActPara.stepContinuNum>=4)
            {
                escActPara.escActStepRecord = ESC_STEP_0;
                escActPara.escActState = ESCACT_START;
                escActPara.enterStep0PreStep = ESC_STEP_2;
                escActPara.enterStep0Sta = -1;//标记是动作失败切回的后退
                escActPara.stepContinuNum = 0;
            }
            escActPara.stepContinuNum++;
        }
    }

    /*
    * 脱困步骤2 第一步动失败后，根据第一步的动作，换一个方向转动
    * actResu：动作的结果 -1表示无效动作 1表示有效动作
    */
    void EscapePlan::escActStep_2()
    {
        static int lastStep2Sta = -1;
        if(lastStep2Sta != escActPara.escActState)
        FRIZY_LOG(LOG_DEBUG, "escStep2[%d->%d]", lastStep2Sta,escActPara.escActState);
        lastStep2Sta = escActPara.escActState;
        switch(escActPara.escActState)
        {
            case 0:
            {
                if(0 != escActPara.rightSpinCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else if(0 != escActPara.leftSpinCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else if(0 != escActPara.rightBehindCnt)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else if(0 != escActPara.leftBehindCnt)
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                
                escActPara.leftSpinCnt = 0;//左自旋 动作次数计数
                escActPara.rightSpinCnt = 0;//右自旋 动作次数计数
                escActPara.leftBehindCnt = 0;//左后 动作次数计数
                escActPara.rightBehindCnt = 0;//右后 动作次数计数
            }
            break;

            case ESCACT_SPIN_LEFT:       //3 左自旋
            {
                escActPara.leftSpinCnt++;
                escSpin(300, 0, 60, 0);
                escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
            }
            break;
            case ESCACT_SPIN_LEFT_WAIT:  //4 左自旋等待
            {
                escActStep2WaitTmp(ESCACT_NUILROT_RIGHT); 
            }
            break;

            case ESCACT_SPIN_RIGHT:      //5 右自旋
            {
                escActPara.rightSpinCnt++;
                escSpin(300, 1, 60, 0);
                escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
            }
            break;
            case ESCACT_SPIN_RIGHT_WAIT: //6 右自旋等待
            {
                escActStep2WaitTmp(ESCACT_NUILROT_LEFT);
            }
            break;

            case ESCACT_NUILROT_LEFT:    //7 左单边旋
            {
                escActPara.leftBehindCnt++;
                singleRotate(-400, -80, 35, 0);
                escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
            }
            break;
            case ESCACT_NUILROT_LEFT_WAIT: //8 左单边旋等待
            {
                escActStep2WaitTmp(ESCACT_SPIN_RIGHT);
            }
            break;
            case ESCACT_NUILROT_RIGHT:     //9 右单边旋
            {
                escActPara.rightBehindCnt++;
                singleRotate(-80, -400, 35, 0);
                escActPara.escActState = ESCACT_NUILROT_RIGHT_WAIT;
            }
            break;
            case ESCACT_NUILROT_RIGHT_WAIT://10 右单边旋等待
            {
                escActStep2WaitTmp(ESCACT_SPIN_LEFT);
            }
            break;
            
            default:
                escActPara.escActState = ESCACT_START;
                break;
        }
    }

    /*
    * 动作3之后的判定中间操作
    * cnt 动作次数
    * lastAct 下一个动作 
    */
    void EscapePlan::escActStep3WaitTmp(int cnt,int lastAct)
    {   
        if(-1 == escActPara.actionRetDelay)//自旋无效
        {   
            //持续2s,切换动作
            if(getTime() - escActPara.escActExecuTime >= 2000)
            {
                if(cnt <= 1)//自旋尝试两次
                {
                    escActPara.escActState = (escapeAction_t)lastAct;
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
                else  //自旋两次都没有成功
                {
                    if(escActPara.rightSpinCnt<=1 || escActPara.leftSpinCnt<=1)
                    {
                        if(lastAct == ESCACT_SPIN_LEFT)
                        {
                            if(escActPara.leftSpinCnt>=2)
                            {
                                escActPara.escActState = ESCACT_SPIN_RIGHT;
                            }
                        }
                        else
                        {
                            if(escActPara.rightSpinCnt>=2)
                            {
                                escActPara.escActState = ESCACT_SPIN_LEFT;
                            }
                        }
                    }
                    else
                    {
                        //进入第四步
                        escActPara.escActStepRecord = ESC_STEP_4;
                        escActPara.escActState = ESCACT_START;
                    }
                }
                FRIZY_LOG(LOG_DEBUG,"cntTmpStep3 %d %d\n",cnt,escActPara.escActState);
            }
        }
        else if(escActPara.actionRetDelay == 1 || chassisEscape.getWheelState() == WHEELSTOP)
        {
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;//切入成功
            escActPara.enterSuccPreStep = ESC_STEP_3;
            escActPara.escActState = ESCACT_START;
            if(escActPara.stepContinuNum>=3)
            {
                //进入第四步
                escActPara.escActStepRecord = ESC_STEP_4;
                escActPara.escActState = ESCACT_START;
                escActPara.stepContinuNum = 0;
            }
            escActPara.stepContinuNum++;
            FRIZY_LOG(LOG_DEBUG, "Step3 enter back:%d %d",cnt,escActPara.escActState);
        }
    }   

    /*
    * 脱困步骤3 左右大力自旋各两次
    */
    void EscapePlan::escActStep_3()
    {
        switch(escActPara.escActState)
        {
            case 0:
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    //左边角速度累加大于右边,这里就认为左边比右边可移动空间更大
                    if(escActPara.leftAngVelAdd > escActPara.rightAngVelAdd)
                    {
                        escActPara.escActState  = ESCACT_SPIN_LEFT;
                    }
                    else
                    {
                        escActPara.escActState  = ESCACT_SPIN_RIGHT;
                    }
                    escActPara.leftSpinCnt = 0;//左自旋 动作次数计数
                    escActPara.rightSpinCnt = 0;//右自旋 动作次数计数
                    escActPara.leftBehindCnt = 0;//左后 动作次数计数
                    escActPara.rightBehindCnt = 0;//右后 动作次数计数
                    escActPara.frontCnt = 0;
                    escActPara.behindCnt = 0;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 2000)
                        chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;

            case ESCACT_SPIN_LEFT:       //3 左自旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.leftSpinCnt++;
                    escSpin(500, 0, 30, 0);
                    escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 2000)
                        chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            case ESCACT_SPIN_LEFT_WAIT:  //4 左自旋等待
            {
                escActStep3WaitTmp(escActPara.leftSpinCnt,ESCACT_SPIN_LEFT); 
            }

            case ESCACT_SPIN_RIGHT:      //5 右自旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.rightSpinCnt++;
                    escSpin(500, 1, 30, 0);
                    escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
                }
                else 
                {
                    if(getTime() - escActPara.escActExecuTime >= 2000)
                        chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;
            case ESCACT_SPIN_RIGHT_WAIT: //6 右自旋等待
            {
                escActStep3WaitTmp(escActPara.rightSpinCnt,ESCACT_SPIN_RIGHT);
            }
            break;

            default:
            escActPara.escActState = ESCACT_START;
            break;
        }
    }

    /*
    * 动作4之后的判定中间操作 
    */
    void EscapePlan::escActStep4WaitTmp()
    {
        static int step4LastAct = 0;
        static int angleRateCnt = 0; 
        static int step4ActWaitTime = 0;
        int timeExecuDiff = 0;
        int angleDiff = 0;
        int angleRate = 0;
        timeExecuDiff = getTime() - escActPara.escActExecuTime;
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle())/10;
        angleRate = getAngleRate();
        if(angleRate > 1500)
        {
            angleRateCnt++;
        }
        else
        {
            angleRateCnt = 0;
        }
        if(step4LastAct != ESCACT_BEHIND && step4LastAct != ESCACT_FRONT)
        {
            if(timeExecuDiff <= 1400 && timeExecuDiff>=1000 \
                &&chassisEscape.getWheelState() ==  WHEELSTOP)
            {
                angleRateCnt = 100;
            }
        }
        if(angleRateCnt > 8||(escActPara.actionRetDelay == 1))
        {
            angleRateCnt = 0;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;
            escActPara.enterSuccPreStep = ESC_STEP_4;
            escActPara.escActState = ESCACT_START;
            FRIZY_LOG(LOG_DEBUG, "step4 enter succ %d %d",escActPara.escActionTryCnt,timeExecuDiff);
            return;
        }
        if(chassisEscape.getWheelState() == WHEELSTOP)
        {
            if(0 == escActPara.escActionTryCnt)
            {
                angleRateCnt = 0;
                if(escActPara.escAngleMaxChange >600)
                    escActPara.escActState = ESCACT_FRONT;
                else
                    escActPara.escActState = ESCACT_BEHIND;        
            }
            else if(1 == escActPara.escActionTryCnt)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else if(2 == escActPara.escActionTryCnt)
            {
                if(angleDiff > 5 && angleDiff < 70)
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                else if(angleDiff >= 70)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }//不需要考虑其他情况
    
            }
            else if(3 == escActPara.escActionTryCnt)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if(4 == escActPara.escActionTryCnt)
            {
                if(angleDiff > 5 && angleDiff < 70)
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                else if(angleDiff >= 70)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }//不需要考虑其他情况
    
            }
            else if(5 <=escActPara.escActionTryCnt && escActPara.escActionTryCnt<=7)//左右摇摆
            {
                if((escActPara.escActionTryCnt % 2) == 0)
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                
                step4ActWaitTime = 500;
            }
            else if(8 == escActPara.escActionTryCnt)//后退
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(9 == escActPara.escActionTryCnt)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if(10 <=escActPara.escActionTryCnt && escActPara.escActionTryCnt<=12)//左右摇摆
            {
                if((escActPara.escActionTryCnt % 2) == 0)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
                
                step4ActWaitTime = 500;
            }
            else if(13 == escActPara.escActionTryCnt)//后退
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(14 == escActPara.escActionTryCnt)//尝试向右旋转
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
                step4ActWaitTime = 4000;
            }
            else if(15 == escActPara.escActionTryCnt)
            {
                if(angleDiff>5 && angleDiff <=50)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                    step4ActWaitTime = 3000;
                }
                else if(angleDiff>50 && angleDiff <=80)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else if(angleDiff >80)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
            }
            else if(16 == escActPara.escActionTryCnt)//尝试向左旋转
            {
                if(ESCACT_SPIN_RIGHT == step4LastAct)//第二次右转了
                {
                    if(angleDiff>5 && angleDiff <=50)
                    {
                        escActPara.escActState = ESCACT_SPIN_RIGHT;
                        step4ActWaitTime = 3000;
                    }
                    else if(angleDiff>50 && angleDiff <=80)
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else if(angleDiff >80)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT;//尝试向左旋转
                        step4ActWaitTime = 4000;
                    }
                }
                else if(ESCACT_FRONT == step4LastAct || ESCACT_FRONT == ESCACT_NUILROT_LEFT)
                {
                    escActPara.actionRetDelay = 1;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT; //尝试向左旋转
                }
            }
            else if(17 == escActPara.escActionTryCnt)
            {
                if(ESCACT_SPIN_LEFT == step4LastAct)
                {
                if(angleDiff>5 && angleDiff <=50)
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT;
                        step4ActWaitTime = 3000;
                    }
                    else if(angleDiff>50 && angleDiff <=80)
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    else if(angleDiff >80)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                }
                else if(ESCACT_FRONT == step4LastAct || ESCACT_FRONT == ESCACT_NUILROT_RIGHT)
                {
                    escActPara.actionRetDelay = 1;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
            }
            else if(18 == escActPara.escActionTryCnt)
            {
                if(ESCACT_SPIN_LEFT == step4LastAct)
                {
                if(angleDiff>5 && angleDiff <=50)
                    {
                        escActPara.escActState = ESCACT_SPIN_LEFT;
                        step4ActWaitTime = 3000;
                    }
                    else if(angleDiff>50 && angleDiff <=80)
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    else if(angleDiff >80)
                    {
                        escActPara.escActState = ESCACT_FRONT;
                    }
                }
                else if(ESCACT_FRONT == step4LastAct || ESCACT_FRONT == ESCACT_NUILROT_RIGHT)
                {
                    escActPara.actionRetDelay = 1;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
            }
            else
            {
                escActPara.escActionTryCnt = 0;
                
                if(escActPara.escUnmovableRateCnt > 1100 || escActPara.escUnmovableCurCnt>900 \
                            || escActPara.escAngleMaxChange<300)
                {
                    FRIZY_LOG(LOG_DEBUG, "MachUnVable %d %d %d",escActPara.escUnmovableRateCnt
                    ,escActPara.escUnmovableCurCnt,escActPara.escAngleMaxChange);
                    escActPara.escActStepRecord = ESC_STEP_FAILD;
                }
                else if(escActPara.stepContinuNum>=3)
                {
                    escActPara.escActStepRecord = ESC_STEP_1;
                    escActPara.enterStep0PreStep = ESCACT_START;
                    escActPara.stepContinuNum = 0;
                    FRIZY_LOG(LOG_DEBUG, "step4->step1");
                }
                escActPara.stepContinuNum++;
            }
            step4LastAct = (escapeAction_t)escActPara.escActState;
            escActPara.escActionTryCnt++;
            FRIZY_LOG(LOG_DEBUG, "step4 try=%d %d",escActPara.escActionTryCnt,escActPara.escActState);
            escActPara.escActPreAngle = getAddAngle();
        }
        else
        {
            if(step4ActWaitTime < 450)
                step4ActWaitTime = 3000;
    
            if(timeExecuDiff >= step4ActWaitTime)
            {
                chassisEscape.chassisSpeed(0, 0, 1);
            }
        }
    }

    /*
    * 脱困步骤4
    * 前进，然后快速摆动
    */
    void EscapePlan::escActStep_4()
    {
        switch(escActPara.escActState)
        {
            case ESCACT_START:
            {
                escActPara.frontCnt = 0;//前进 动作次数计数
                escActPara.behindCnt = 0;//后退 动作次数计数
                escActPara.leftSpinCnt = 0;//左自旋 动作次数计数
                escActPara.rightSpinCnt = 0;//右自旋 动作次数计数
                escActPara.leftBehindCnt = 0;//左后 动作次数计数
                escActPara.rightBehindCnt = 0;//右后 动作次数计数
                escActPara.escActionTryCnt = 0;//动作尝试次数计数
                escActStep4WaitTmp();
            }
            break;

            case ESCACT_NUILROT_LEFT://左单边旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.leftBehindCnt++;
                    singleRotate(-400, -100, 50, 0);
                    escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 2500)
                    {
                        chassisEscape.chassisSpeed(0, 0, 1);
                    }
                }
            }
            break;
            case ESCACT_NUILROT_LEFT_WAIT://左单边旋等待
            {
                escActStep4WaitTmp();
            }
            break;

            case ESCACT_NUILROT_RIGHT://右单边旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.rightBehindCnt++;
                    singleRotate(-100, -400, 50, 0);
                    escActPara.escActState = ESCACT_NUILROT_LEFT_WAIT;
                }
                else
                {
                    if(getTime() - escActPara.escActExecuTime >= 2500)
                    {
                        chassisEscape.chassisSpeed(0, 0, 1);
                    }
                }
            }
            break;
            case ESCACT_NUILROT_RIGHT_WAIT://右单边旋等待
            {
                escActStep4WaitTmp();
            }
            break;

            case ESCACT_SPIN_RIGHT:      //5 右自旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.rightSpinCnt++;
                    escSpin(500, 1, 90, 0);
                    escActPara.escActState = ESCACT_SPIN_RIGHT_WAIT;
                }
                if(getTime() - escActPara.escActExecuTime >= 2500)
                {
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break;
            case ESCACT_SPIN_RIGHT_WAIT:    //6 右自旋等待
            {
                escActStep4WaitTmp();
            }
            break;

            case ESCACT_SPIN_LEFT:       //3 左自旋
            {
                if(chassisEscape.getWheelState() == WHEELSTOP)
                {
                    escActPara.leftSpinCnt++;
                    escSpin(500, 0, 90, 0);
                    escActPara.escActState = ESCACT_SPIN_LEFT_WAIT;
                }
                if(getTime() - escActPara.escActExecuTime >= 2500)
                {
                    chassisEscape.chassisSpeed(0, 0, 1);
                }
            }
            break; 
            case ESCACT_SPIN_LEFT_WAIT:  //4 左自旋等待
            {
                escActStep4WaitTmp();
            }
            break;

            case ESCACT_FRONT:             //14前进
            {
                escActPara.frontCnt++;
                wheelCtrlStraight(160, 600);
                escActPara.escActState = ESCACT_FRONT_WAIT;
            }
            break;
            case ESCACT_FRONT_WAIT:        //15前进等待
            {
                escActStep4WaitTmp();
            }
            break;

             case ESCACT_BEHIND:          //1 后退
            {
                escActPara.behindCnt++;
                wheelBackDist(-400,80);//正常的速度后退
                escActPara.escActState = ESCACT_BEHIND_WAIT;
            }
            break;
            case ESCACT_BEHIND_WAIT:     //2 后退等待
            {
                escActStep4WaitTmp();
            }
            break;
            default:
                escActPara.escActState = ESCACT_START;
                break;
        }
    }    

    /*      
    *在架空脱困 卡死中监控机器的俯仰横滚数据     
    * 10成功下来 2x前 3x后 4x左 5x右 6x 中
    */          
    int8_t EscapePlan::overheadCheck(float tmpDiff)
    {
        bool actChangGraFlag = false;
        int8_t retTmp = 0;
        float currPitTmp = getPitRol(0);
        float currRolTmp = getPitRol(1);
        float overPit = getMEMSRelatAngle(0);
        float overRol = getMEMSRelatAngle(1);
        float actChangPitTmp = fabsf(getMEMSActChangAngle(0));
        float actChangRolTmp = fabsf(getMEMSActChangAngle(1));
        //俯仰和衡滚变化角度大
        if(actChangPitTmp>=5.0f || actChangRolTmp>=5.0f)
        {
            actChangGraFlag = true;
        }
        //判定为架空脱成功了
        // if((tmpDiff<2.5f && fabsf(overPit) <3.5f && fabsf(overRol) <3.5f &&actChangGraFlag)
        //     ||(tmpDiff<1.5f && fabsf(overPit) <1.5f && fabsf(overRol) <1.5f)
        //     ||(actChangGraFlag &&(fabsf(currPitTmp)<3.2f&&fabsf(currRolTmp)<3.2f))
        //     ||(1 == escActPara.groundPeneDiff&&actChangGraFlag))
        // if((tmpDiff < 3.5f && fabsf(overPit) < 4.5f && fabsf(overRol) < 4.5f && actChangGraFlag)    
        //     ||(tmpDiff < 2.5f && fabsf(overPit) < 2.5f && fabsf(overRol) < 2.5f)
        //     ||(actChangGraFlag && (fabsf(currPitTmp) < 10.0f && fabsf(currRolTmp) < 10.0f))
        //     ||(1 == escActPara.groundPeneDiff && actChangGraFlag))
        if((tmpDiff < 6.0f && fabsf(overPit) < 6.0f && fabsf(overRol) < 6.0f && actChangGraFlag)    
            ||(tmpDiff < 5.0f && fabsf(overPit) < 5.0f && fabsf(overRol) < 5.0f)
            ||(actChangGraFlag && (fabsf(currPitTmp) < 10.0f && fabsf(currRolTmp) < 10.0f))
            ||(1 == escActPara.groundPeneDiff && actChangGraFlag)
            ||(abs(currPitTmp - escActPara.escPre10PrePit) < 5.0f) && abs(currRolTmp - escActPara.escPre10PreRol) < 5.0f)
        {
            retTmp = 10;
            FRIZY_LOG(LOG_DEBUG,"overhead escape sucess!");
        }
        else 
        {
            if(overPit < -5.0f)//前面翘起
            {
                retTmp = 21;
            }
            else if(overPit > 5.0f)//后面翘起
            {
                retTmp = 31;
            }
            else if(overRol>5.0f)//左边翘起
            {
                retTmp = 41;
            }
            else if(overRol<-5.0f)//右边翘起
            {
                retTmp = 51;
            }
            else 
            {
                if(fabsf(overRol)-fabsf(overPit)>1.5f)//偏左右
                {
                    if(overRol)//偏左
                    {
                        retTmp = 42;
                    }
                    else//偏右
                    {
                        retTmp = 52;
                    }
                }
                else if(fabsf(overPit)-fabsf(overRol)>1.5f)//偏前后
                {
                    if(overPit>0)//偏后
                    {
                        retTmp = 32;
                    }
                    else//偏前
                    {
                        retTmp = 22;
                    }
                }
                
                else
                {
                    retTmp = 60;
                }
            }
        }
        FRIZY_LOG(LOG_DEBUG, "retTmp:%d", retTmp);
        return retTmp;
    }
    static int8_t leftTryOverhCnt = 0,rightTryOverhCnt = 0;
    static int8_t frontTryOverhCnt = 0,behindTryOverhCnt = 0,interTryOverhCnt = 0;
    
    //架空做完动作等待判断,确定下一步动作
    int EscapePlan::judgeActionOverHead(float tmpDiff)
    {
        static int8_t lastOverActSta = 0; 
        int8_t actPitRol = 0;
        int8_t actPitRolTmp = 0;
        int32_t angleDiff = 0;
        actPitRol = overheadCheck(tmpDiff);
        actPitRolTmp = (int8_t)(actPitRol/10);
        if(actPitRolTmp == 1)
        {
            FRIZY_LOG(LOG_INFO,"success head");
            chassisEscape.chassisSpeed(0, 0, 1);
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;//切入成功
            return 0;
        }
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle())/10;
        if(actPitRolTmp == 2)
        {
            if(0 == frontTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(1 == frontTryOverhCnt)
            {
                alongWallDir = getAlongWallDir();
                if(alongWallDir == RIGHTAW)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else if(alongWallDir == LEFTAW)
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
                lastOverActSta = escActPara.escActState;
            }
            frontTryOverhCnt++;
        }
        else if(actPitRolTmp == 3)//后
        {
            if(0 == behindTryOverhCnt)
            {
                escActPara.escActState = ESCACT_FRONT;
            }
            else if(1 == behindTryOverhCnt)
            {
                if(getPitRol(1) > 0)//左边翘起多
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
                else 
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                lastOverActSta = escActPara.escActState;
            }
            else 
            {
                if(5 == behindTryOverhCnt||10 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
            }
            behindTryOverhCnt++;
        }
        else if(actPitRolTmp == 4)//左
        {
            if(0 == leftTryOverhCnt)
            {
                escActPara.escActState = ESCACT_NUILROT_LEFT;
            }
            else if(1 == leftTryOverhCnt)
            {
                if(getPitRol(1) > 0)    //左边翘起多
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
                else 
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                lastOverActSta = escActPara.escActState;
            }
            leftTryOverhCnt++;
        }
        else if(actPitRolTmp == 5)//右
        {
            if(0 == rightTryOverhCnt)
            {
                escActPara.escActState = ESCACT_NUILROT_RIGHT;
            }
            if(1 == rightTryOverhCnt)
            {
                if(angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
                {
                    escActPara.escActState = (escapeAction_t)lastOverActSta;
                }
                else 
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                lastOverActSta = escActPara.escActState;
            }
            rightTryOverhCnt++;
        }
        else if(actPitRolTmp == 6)//中
        {
            if(0 == interTryOverhCnt)
            {
                alongWallDir = getAlongWallDir();
                if(alongWallDir == RIGHTAW)
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else if(alongWallDir == LEFTAW)
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
            }
            else if(1 == interTryOverhCnt)
            {
                if(angleDiff > ENABLE_MOVE_ANGLE_L && angleDiff < ENABLE_MOVE_ANGLE_H)
                {
                    escActPara.escActState = (escapeAction_t)lastOverActSta;
                }
                else
                {
                    if(getPitRol(1)>0)//左边翘起多
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                    lastOverActSta = escActPara.escActState;
                }
                
            }
            behindTryOverhCnt = 0;
            interTryOverhCnt++;
        }
        escActPara.escActPreAngle = getAddAngle();
        return 0;
    }
    //卡死做完动作等待判断,确定下一步动作
    int EscapePlan::judgeActionStuckHead(float tmpDiff)
    {
        static int8_t lastHeadSta = 0;
        int8_t actPitRol = 0;
        int8_t actPitRolTmp = 0;
        int32_t angleDiff = 0;
        //2x前 3x后 4x左 5x右 6x 中
        actPitRol = overheadCheck(tmpDiff);
        actPitRolTmp = (int8_t)(actPitRol/10);
        if(actPitRolTmp == 1)
        {
            FRIZY_LOG(LOG_INFO,"success stuck");
            chassisEscape.chassisSpeed(0, 0, 1);
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;//切入成功
            return 0;
        }
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle()/10);
        FRIZY_LOG(LOG_DEBUG,"angleDiff:%d = escActPara.escActPreAngle:%d - getAddAngle():%d/10");
        if(actPitRolTmp != 2)//前面翘起//2 == actPirRolTmp
        {   
            if(0 == frontTryOverhCnt)
            {
                if(getPitRol(1)>0)//左边翘起多
                {
                    escActPara.escActState = ESCACT_NUILROT_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_NUILROT_RIGHT;
                }
            }
            else if(1<=frontTryOverhCnt && frontTryOverhCnt<=4)
            {
                if(angleDiff>5 && angleDiff<20)
                {
                    escActPara.escActState = (escapeAction_t)lastHeadSta;
                }
                else
                {
                    if(ESCACT_NUILROT_LEFT == lastHeadSta)
                    {
                        lastHeadSta = ESCACT_NUILROT_RIGHT;
                    }
                    else
                    {
                        lastHeadSta = ESCACT_NUILROT_LEFT;
                    }
                }
            }
            else if(5 == frontTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(6<=frontTryOverhCnt && frontTryOverhCnt<=9)
            {
                if(angleDiff>5)
                {
                    escActPara.escActState = (escapeAction_t)lastHeadSta;
                }
                else
                {
                    if(ESCACT_NUILROT_LEFT == lastHeadSta)
                    {
                        lastHeadSta = ESCACT_SPIN_RIGHT;
                    }
                    else
                    {
                        lastHeadSta = ESCACT_SPIN_LEFT;
                    }
                }
            }
            else if(10 == frontTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(11<=frontTryOverhCnt && frontTryOverhCnt<=14)
            {
                if(angleDiff>5 && angleDiff<20)
                {
                    escActPara.escActState = (escapeAction_t)lastHeadSta;
                }
                else
                {
                    if(ESCACT_NUILROT_LEFT == lastHeadSta)
                    {
                        lastHeadSta = ESCACT_SPIN_RIGHT;
                    }
                    else
                    {
                        lastHeadSta = ESCACT_SPIN_LEFT;
                    }
                }
            }
            else if(15 == frontTryOverhCnt)
            {
                escActPara.escActState = ESCACT_FRONT;
            }
            else if(16 == frontTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(17 == frontTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }

            frontTryOverhCnt++;
        }
        else if(actPitRolTmp == 2)
        {
            if(0 == interTryOverhCnt)
            {
                escActPara.escActState = ESCACT_FRONT;
                frontTryOverhCnt = 0;
            }
            else if(1<=interTryOverhCnt && interTryOverhCnt<=4)
            {
                if(angleDiff>5 && angleDiff<20)
                {
                    escActPara.escActState = (escapeAction_t)lastHeadSta;
                }
                else
                {
                    if(ESCACT_NUILROT_LEFT == lastHeadSta)
                    {
                        lastHeadSta = ESCACT_SPIN_RIGHT;
                    }
                    else
                    {
                        lastHeadSta = ESCACT_SPIN_LEFT;
                    }
                }
            }
            interTryOverhCnt++;
        }
        else if(actPitRolTmp == 3 || actPitRolTmp == 6)
        {
            if(0 == behindTryOverhCnt)
            {
                escActPara.escActState = ESCACT_FRONT;
            }
            else if(1 == behindTryOverhCnt)
            {
                if(getPitRol(1)>0)//左边翘起多
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
            }
            else if(1 == behindTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(2 == behindTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(3 == behindTryOverhCnt)
            {
                if(escActPara.leftSpinCnt>escActPara.rightSpinCnt)
                {
                    escActPara.escActState = ESCACT_SPIN_RIGHT;
                }
                else
                {
                    escActPara.escActState = ESCACT_SPIN_LEFT;
                }
            }
            else if(4 == behindTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(5 == behindTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(6 == behindTryOverhCnt)
            {
                escActPara.escActState = ESCACT_FRONT;
            }
            else 
            {
                if(12 == behindTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_FRONT;
                }
            }
            behindTryOverhCnt++;
        }
        else if(actPitRolTmp == 4)
        {
            if(0 == leftTryOverhCnt)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if(1 == leftTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(2 == leftTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(3 == leftTryOverhCnt)
            {
                escActPara.escActState = ESCACT_SPIN_LEFT;
            }
            else if(4 == leftTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(5 == leftTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            leftTryOverhCnt++; 

        }
        else if(actPitRolTmp == 5)
        {
            if(0 == rightTryOverhCnt)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else if(1 == rightTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(2 == rightTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(3 == rightTryOverhCnt)
            {
                escActPara.escActState = ESCACT_SPIN_RIGHT;
            }
            else if(4 == rightTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            else if(5 == rightTryOverhCnt)
            {
                escActPara.escActState = ESCACT_BEHIND;
            }
            rightTryOverhCnt++;
        }
        escActPara.escActPreAngle = getAddAngle();
        return 0;
    }

    //架空脱困  
    void EscapePlan::escStepOverHead()
    {
        float currPitRol = 0.0f,diffPitRol = 0.0f;
        currPitRol = gyroPitRolSlidingAvg();//记录机器陀螺仪姿态
        diffPitRol = fabsf(escActPara.escPre10PrePitRol - currPitRol);//和10s前的做对比
        switch(escActPara.escActState)
        {
            case ESCACT_START://判断下一步执行什么动作
                escActPara.frontCnt = 0;//前进 动作次数计数
                escActPara.behindCnt = 0;//后退 动作次数计数
                escActPara.leftSpinCnt = 0;//左自旋 动作次数计数
                escActPara.rightSpinCnt = 0;//右自旋 动作次数计数
                escActPara.leftBehindCnt = 0;//左后 动作次数计数
                escActPara.rightBehindCnt = 0;//右后 动作次数计数
                leftTryOverhCnt = 0;
                rightTryOverhCnt = 0;
                frontTryOverhCnt = 0;
                behindTryOverhCnt = 0;
                interTryOverhCnt = 0;
                judgeActionOverHead(diffPitRol);
            break;

            case ESCACT_NUILROT_LEFT:
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_RIGHT");
                singleRotate(-300, 10, 90, 0);
                judgeActionOverHead(diffPitRol);
            break;

            case ESCACT_NUILROT_RIGHT:
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_RIGHT");
                singleRotate(10, -300, 90, 0);
                judgeActionOverHead(diffPitRol);
            break;

            case ESCACT_SPIN_LEFT:
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_LEFT");
                escSpin(300, 0, 90, 0);
                judgeActionOverHead(diffPitRol);
            break;

            case ESCACT_SPIN_RIGHT:
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_RIGHT");
                escSpin(300, 1, 90, 0);
                judgeActionOverHead(diffPitRol);
            break;

            case ESCACT_BEHIND:
                FRIZY_LOG(LOG_DEBUG, "ESCACT_BEHIND");
                wheelBackDist(-300, 150);
                judgeActionOverHead(diffPitRol);
            break;

            case ESCACT_FRONT:
                FRIZY_LOG(LOG_DEBUG, "ESCACT_FRONT");
                wheelCtrlStraight(160, 1000);
                judgeActionOverHead(diffPitRol);
            break;

            default:
                escActPara.escActState = ESCACT_START;
            break;
        }
    }
    
    //头部卡死      
    void EscapePlan::escActStepStuckTheHead()
    {
        float currPitRol = 0.0f,diffPitRol = 0.0f;
        currPitRol = gyroPitRolSlidingAvg();//记录机器陀螺仪姿态
        // FRIZY_LOG(LOG_INFO, "compare with 10s pitrol:%f, %f", escActPara.escPre10PrePit, escActPara.escPre10PreRol);
        diffPitRol = fabsf(escActPara.escPre10PrePitRol - currPitRol);//和10s前的做对比
        switch(escActPara.escActState)
        {
            case ESCACT_START:
                escActPara.frontCnt = 0;//前进 动作次数计数
                escActPara.behindCnt = 0;//后退 动作次数计数
                escActPara.leftSpinCnt = 0;//左自旋 动作次数计数
                escActPara.rightSpinCnt = 0;//右自旋 动作次数计数
                escActPara.leftBehindCnt = 0;//左后 动作次数计数
                escActPara.rightBehindCnt = 0;//右后 动作次数计数
                leftTryOverhCnt = 0;
                rightTryOverhCnt = 0;
                frontTryOverhCnt = 0;
                behindTryOverhCnt = 0;
                interTryOverhCnt = 0;
                escActPara.escActState = ESCACT_BEHIND;
            break;

            case ESCACT_NUILROT_LEFT://左单边旋
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_LEFT");
                singleRotate(-400, 10, 60, 0);
                judgeActionStuckHead(diffPitRol);
            break;

            case ESCACT_NUILROT_RIGHT://右单边旋
                FRIZY_LOG(LOG_DEBUG, "ESCACT_NUILROT_RIGHT");
                singleRotate(10, -400, 60, 0);
                judgeActionStuckHead(diffPitRol);
            break;
            
            case ESCACT_SPIN_RIGHT:      //5 右自旋
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_RIGHT");
                escSpin(300, 1, 60, 0);
                judgeActionStuckHead(diffPitRol);
            break;

            case ESCACT_SPIN_LEFT:       //3 左自旋
                FRIZY_LOG(LOG_DEBUG, "ESCACT_SPIN_LEFT");
                escSpin(300, 0, 60, 0);
                judgeActionStuckHead(diffPitRol);
            break;

            case ESCACT_BEHIND:          //1 后退
                FRIZY_LOG(LOG_DEBUG, "ESCACT_BEHIND");
                wheelCtrlStraight(-300, 600);
                judgeActionStuckHead(diffPitRol);
            break;

            case ESCACT_FRONT:             //14前进
                FRIZY_LOG(LOG_DEBUG, "ESCACT_FRONT");
                wheelCtrlStraight(160, 600);
                judgeActionStuckHead(diffPitRol);
            break;

            default:
                escActPara.escActState = ESCACT_START;
            break;
        }
    }   
    
    //头部卡住动作执行等待操作      不用
    void EscapePlan::stuckActHeadWait(float tmpDiff)
    {
        static int8_t lastHeadSta = 0;
        static int32_t actExeTime = 0;
        int8_t actPirRol = 0;
        int8_t actPirRolTmp = 0;
        int32_t angleDiff = 0;
        //2x前 3x后 4x左 5x右 6x中
        actPirRol = overheadCheck(tmpDiff);
         actPirRolTmp = (int8_t)(actPirRol/10);
    
        if(1 == actPirRolTmp)
        {
            FRIZY_LOG(LOG_INFO,"success head");
            chassisEscape.chassisSpeed(0, 0, 1);
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;//切入成功
            return;
        }
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle());
        if(chassisEscape.getWheelState() == WHEELSTOP)
        {
            if(actPirRolTmp != 2)
            {
                if(0 == frontTryOverhCnt)
                {
                    if(getPitRol(1)>0)//左边翘起多
                    {
                        escActPara.escActState = ESCACT_NUILROT_LEFT;
                    }
                    else
                    {
                        escActPara.escActState = ESCACT_NUILROT_RIGHT;
                    }
                }
                else if(1<=frontTryOverhCnt && frontTryOverhCnt<=4)
                {
                    if(angleDiff>5 && angleDiff<20)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if(ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_NUILROT_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_NUILROT_LEFT;
                        }
                    }
                }
                else if(5 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                else if(6<=frontTryOverhCnt && frontTryOverhCnt<=9)
                {
                    if(angleDiff>5)
                    {
                        escActPara.escActState = (escapeAction_t)lastHeadSta;
                    }
                    else
                    {
                        if(ESCACT_NUILROT_LEFT == lastHeadSta)
                        {
                            lastHeadSta = ESCACT_SPIN_RIGHT;
                        }
                        else
                        {
                            lastHeadSta = ESCACT_SPIN_LEFT;
                        }
                    }
                }
                else if(10 == frontTryOverhCnt)
                {
                    escActPara.escActState = ESCACT_BEHIND;
                }
                

            }
        }
    }
    
    // 架空动作 之后的判定中间操作  不用
    int EscapePlan::escStepOverheadWait(float tmpDiff)
    {
        static int8_t lastOverActSta = 0; 
        static int8_t errEacOVerActCnt = 0;
        static int32_t nextActWaitTime = 0;//下一个动作等待的时间
        int8_t actPirRol = 0;
        int8_t actPirRolTmp = 0;
        int16_t angleRet = 0;
        int32_t angleDiff = 0;
        //2x前 3x后 4x左 5x右 6x 中
        actPirRol = overheadCheck(tmpDiff);
        actPirRolTmp = (int8_t)(actPirRol/10);
        if(1 == actPirRolTmp)
        {
            chassisEscape.chassisSpeed(0, 0, 0);
            escActPara.escActState = ESCACT_START;
            escActPara.escActStepRecord = ESC_STEP_SUCCESS;//切入成功
            FRIZY_LOG(LOG_INFO,"enter step success, actPirRol:%d, actPirRolTmp:%d", actPirRol, actPirRolTmp);
        }
        if(actPirRolTmp == 3 && ESCACT_FRONT_WAIT != escActPara.escActState)
        {
            nextActWaitTime = 1000;
        }
        angleDiff = abs(escActPara.escActPreAngle - getAddAngle())/10;
        if(chassisEscape.getWheelState() == WHEELSTOP)
        {
            
        }
        
        
    }
    
    //脱困失败处理
    void EscapePlan::escActStepFailed()
    {
        memset(escLastLeftCurAvg,0,sizeof(escLastLeftCurAvg));
        memset(escLastRightCurAvg,0,sizeof(escLastRightCurAvg));
        memset(escLeftCurArry,0,sizeof(escLeftCurArry));
        memset(escRightCurArry,0,sizeof(escRightCurArry));
        memset(pitRolLastAvg,0,sizeof(pitRolLastAvg));
        memset(pitRolAvgArry,0,sizeof(pitRolAvgArry));
        memset(gyroAccBuf,0,sizeof(gyroAccBuf));
        memset(escPreActionArry,0,sizeof(escPreActionArry));
        memset(&escActPara,0,sizeof(escActPara));
        ESCAPE_LeftOffCheckInit();
        stuckType = NOTHTING;
        escCkeckPara.arryAvgCurCountR = 0;
        escCkeckPara.arryAvgCurCountL = 0;
        record_time = 0;
        recordPit.clear();
        recordRol.clear();
        recordPitRol.clear();
        chassisEscape.chassisSpeed(0, 0, 1);
    }

    //脱困成功
    void EscapePlan::escActStepSuccess()
    {
        //脱困后,根据进入脱困前的状况旋转角度
        if(beforeEscState == 1) //之前处于沿墙状态下进入的脱困
        {
            if(getAlongWallDir() == LEFTAW)
            {
                float aimforward = enterGyroAngle + 90;
                if(aimforward >= 360)
                {
                    aimforward = aimforward - 360;
                }
                escSpin(120, 1, aimforward, 1);
                FRIZY_LOG(LOG_DEBUG, "escape end right spin angle: 90");
            }
            else
            {
                float aimforward = enterGyroAngle - 90;
                if(aimforward <= 0)
                {
                    aimforward = 360 + aimforward;
                }
                escSpin(120, 0, aimforward, 1);
                FRIZY_LOG(LOG_DEBUG, "escape end left spin angle: 90");
            }
        }
        else    //非沿墙状态下进入的脱困
        {
            float aimforward = enterGyroAngle + 180;
            if(aimforward >= 360)
            {
                aimforward = aimforward - 360;
            }
            escSpin(120, 1, aimforward, 1);
            FRIZY_LOG(LOG_DEBUG, "escape end spin angle: 180");
        }
        memset(escLastLeftCurAvg,0,sizeof(escLastLeftCurAvg));
        memset(escLastRightCurAvg,0,sizeof(escLastRightCurAvg));
        memset(escLeftCurArry,0,sizeof(escLeftCurArry));
        memset(escRightCurArry,0,sizeof(escRightCurArry));
        memset(pitRolLastAvg,0,sizeof(pitRolLastAvg));
        memset(pitRolAvgArry,0,sizeof(pitRolAvgArry));
        memset(gyroAccBuf,0,sizeof(gyroAccBuf));
        memset(escPreActionArry,0,sizeof(escPreActionArry));
        memset(&escActPara,0,sizeof(escActPara));
        ESCAPE_LeftOffCheckInit();
        stuckType = NOTHTING;
        escCkeckPara.arryAvgCurCountR = 0;
        escCkeckPara.arryAvgCurCountL = 0;
        record_time = 0;
        recordPit.clear();
        recordRol.clear();
        recordPitRol.clear();
        chassisEscape.chassisSpeed(0, 0, 1);
    }

    //脱困处理
    bool EscapePlan::escDeal(int state)
    {   
        FRIZY_LOG(LOG_INFO, "start escape!");
        int lastAct = -1;
        escStartTime = getTime();//记录脱困开始时间
        enterGyroAngle = 360-gyo_angle_*180/_Pi;    //记录脱困前的角度
        FRIZY_LOG(LOG_DEBUG, "enterGyroAngle:%f", enterGyroAngle);
        enterAddAngle = getAddAngle();              //记录脱困前的陀螺仪累加角度
        beforeEscState = state;                     //记录脱困前的扫地机状态
        while(GLOBAL_CONTROL == WHEEL_RUN)
        {
            //执行动作改变,更新动作执行时间
            if(lastAct != escActPara.escActState)
            {
                FRIZY_LOG(LOG_INFO,"ACTION CHANGE,UPDATE TIME");
                escActPara.escActExecuTime = getTime();
                escActPara.actionRetDelay = 0;
            }
            lastAct = escActPara.escActState;
            //一个状态停留超过5s，直接判定为动作无效
            if(getTime() - escActPara.escActExecuTime >= 8000)
            {
                FRIZY_LOG(LOG_INFO,"ACTION INVALID");
                escActPara.enterStep0PreStep = ESC_STEP_0;
                escActPara.escActState = ESCACT_START;
                FRIZY_LOG(LOG_DEBUG, "act out 8s escActState:%d escActStepRecord:%d",escActPara.escActState,escActPara.escActStepRecord);
            }
            else if(getTime() - escActPara.escActExecuTime >= 5000)
            {
                escActPara.actionRetDelay =-1;
                FRIZY_LOG(LOG_DEBUG, "act out 5s escActPara:%d escActStepRecord:%d",escActPara.escActState,escActPara.escActStepRecord);
            }
            if(getTime() - escStartTime >= 60 * 1000)
            {
                // escActPara.enterStep0PreStep = ESC_STEP_0;
                escActPara.escActStepRecord = ESC_STEP_FAILD;
                FRIZY_LOG(LOG_INFO,"TIME OUT, ESCAPE FAILED");
            }
            //针对架空脱困和尾部翘起卡死脱困，如果数据正常直接切入脱困
            if(escActPara.escActStepRecord != ESC_STEP_SUCCESS && 
                escActStepBehind())
            {
                chassisEscape.chassisSpeed(0, 0, 1);
                escActPara.enterSuccPreStep = escActPara.escActStepRecord;
                escActPara.escActStepRecord = ESC_STEP_SUCCESS;
                escActPara.escActState = ESCACT_START;
            }
            switch(escActPara.escActStepRecord)
            {
                case ESC_STEP_START:
                    //脱困前停止10次，采集停止的加速数据
                    escStepPreGyroAnal();
                break;
                case ESC_STEP_0://1
                    FRIZY_LOG(LOG_INFO,"ESC_STEP_0");
                    escActStep_0();
                break;
                case ESC_STEP_1://2
                    FRIZY_LOG(LOG_INFO,"ESC_STEP_1");
                    escActStep_1();
                    break;
                case ESC_STEP_2://3
                    FRIZY_LOG(LOG_INFO,"ESC_STEP_2");
                    escActStep_2();
                    break;
                case ESC_STEP_3://4
                    FRIZY_LOG(LOG_INFO,"ESC_STEP_3");
                    escActStep_3();
                    break;
                case ESC_STEP_4://5
                    FRIZY_LOG(LOG_INFO,"ESC_STEP_4");
                    escActStep_4();
                    break;
                case ESC_STEP_FAILD:
                    FRIZY_LOG(LOG_INFO,"ESCAPE FAILED");
                    escActStepFailed();
                    return 0;
                break;
                case ESC_STEP_SUCCESS:
                    FRIZY_LOG(LOG_INFO,"ESCAPE SUCCESSFUL");
                    escActStepSuccess();
                    return 1;
                break;
                case ESC_STEP_OVERHEAD://架空
                    escStepOverHead();
                break;

                case ESC_STEP_STUCK_HEAD://头部卡死(欧式家具)
                    escActStepStuckTheHead();
                break;

                default:
                    escActPara.escActStepRecord = ESC_STEP_START;
                break;
            }
            usleep(25 * 1000);
        }
    }
    void EscapePlan::EscapeSmallArea()
    {
        FRIZY_LOG(LOG_INFO," EscapeSmallArea ");
        for(int i =0 ; i< 10; i++)
        {
            chassisEscape.GetSensor(&escapSensor);
            if(escapSensor.bump !=0)
            {
                chassisEscape.chassisSpeed(100,-100,1);
                usleep(50*1000);
            }
            
            else
            {
                wheelCtrlStraight(-200, 50);
            }
        }
    }
    

}