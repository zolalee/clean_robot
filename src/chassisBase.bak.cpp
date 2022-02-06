/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-17 10:48:29
 * @LastEditTime : 2022-01-25 15:12:48
 * @Project      : UM_path_planning
 */
#include "um_chassis/chassisBase.h"
#include "navigation_algorithm/AlongWall.h"
#include "navigation_algorithm/RoadPlanning.h"

#include <random>

#ifndef __UMUSERMODE__
#define __UMUSERMODE__ 444
#endif


using namespace core;
namespace useerobot{
extern PRO process;
extern ROAD_STATE roadState;
extern allDirectionPointInfo_t* laserDis;
share_path_planning_t* current_path_planning; 
int calibration_time = 0; 
current_pose_t* current_pose;
current_pose_t share_pose;
current_pose_t raw_share_pose;
static int wall = 0;
static int lastAngle = 0;
static int tempadd = 0;
static bool speedOrder = 0;
bool getBlockInfoIdenx = false;
bool getPointInfoIndex = false;
bool getForbbidenInfoIndex =false;
CHASSIS_CONTROL GLOBAL_CONTROL;
chassisBase *chassisBase::m_Instance = nullptr;
SensorData_t sensordata;
// extern float angle_;
int round_doubel(double number)
{
return (number>0.0)?(number+0.5):(number-0.5);
}
chassisBase *chassisBase::getInstance()
{
        if (m_Instance == nullptr){
                m_Instance = new chassisBase();
        }
        return m_Instance;
}
chassisBase::chassisBase()
{
        
};
chassisBase::~chassisBase()
{
        
};

double chassisBase::Conlaser(){
    
    //std::normal_distribution<double> distribution(5.0, 2.0);
    FRIZY_LOG(LOG_DEBUG,"lasersize.%d",laserDis->size);
    int sum_laser_times = 0;
    for (int i = 0;i < laserDis->size; ++i){
            
        if (!laserDis->point[0].x && !laserDis->point[0].y)
            continue;
        if (laserDis->point[i].theta < 4*_Pi/9 || laserDis->point[i].theta > 14*_Pi/9)
        {
                double tmp = pow(pow(laserDis->point[i].x, 2) + pow(laserDis->point[i].y, 2), 0.5);
                if (tmp > 0.1 && tmp < 0.2)
                {

                FRIZY_LOG(LOG_DEBUG,"laser stop.%f,%f,%f",tmp,laserDis->point[i].x,laserDis->point[i].y);
                sum_laser_times ++;
                if(sum_laser_times>3)
                {
                        FRIZY_LOG(LOG_DEBUG,"there is obstacle in laser data");
                        return tmp;
                }                
                }
        }
    }
    return 0;
}

//获取传感器数据
void chassisBase::GetSensor(Sensor* pss)
{
// FRIZY_LOG(LOG_INFO, "enter GetSensor");
    

//     for (auto tmp: vec){
//         if ()
//     }
    pss->laserM = 0;
//     if (!IsWall() && roadState != roadTrue)
//         pss->laserM = Conlaser();

    pss->batvolume = sensordata.batVolume;

    pss -> bump = sensordata.Bump_Motor;
    
    pss -> leftw = sensordata.LeftWheel_Speed;
    pss -> rightw = sensordata.RightWheel_Speed;
    pss -> rightAlongWall = sensordata.rightAlongWallValue;
    pss -> leftAlongWall = sensordata.leftAlongWallValue;

    //全方位减速
    pss -> leftOmnibearingSlow = sensordata.rightOmnibearingSlow_index;
    pss -> rightOmnibearingSlow = sensordata.leftOmnibearingSlow_index;
    pss -> midOmnibearingSlow = sensordata.middleOmnibearingSlow_index;

    //全方位转向
    pss -> leftOmnibearingTurn = sensordata.rightOmnibearingTurn_index;   
    pss -> rightOmnibearingTurn = sensordata.leftOmnibearingTurn_index;   
    pss -> midOmnibearingTurn = sensordata.middleOmnibearingTurn_index;
    
    //全方位开关灯值
    pss -> leftOmnibearingOn = sensordata.leftOmnibearingOn_index;
    pss -> leftOmnibearingOff = sensordata.leftOmnibearingOff_index;
    pss -> midOmnibearingOn = sensordata.midOmnibearingOn_index;
    pss -> midOmnibearingOff = sensordata.midOmnibearingOff_index;
    pss -> rightOmnibearingOn = sensordata.rightOmnibearingOn_index;
    pss -> rightOmnibearingOff = sensordata.rightOmnibearingOff_index;
    pss -> leftCliff = sensordata.leftGeologicalDetect_index;
    pss -> rightCliff = sensordata.rightGeologicalDetect_index;
    pss -> midCliff = sensordata.middleGeologicalDetect_index;
    pss -> leftCliffValue = abs(sensordata.leftGeologicalDetect_index_on - sensordata.leftGeologicalDetect_index_off);
    pss -> midLeftCliffValue = abs(sensordata.middleLeftGeologicalDetect_index_on - sensordata.middleLeftGeologicalDetect_index_off);
    pss -> midRightCliffValue = abs(sensordata.middleRightGeologicalDetect_index_on - sensordata.middleRightGeologicalDetect_index_off);
    pss -> rightCliffValue = abs(sensordata.rightGeologicalDetect_index_on - sensordata.rightGeologicalDetect_index_off);

    pss -> mcuLeftCliff = sensordata.mcuLeftCliff;
    pss -> mcuRightCliff = sensordata.mcuRightCliff;
    pss -> mcuLeftMidCliff = sensordata.mcuLeftMidCliff;
    pss -> mcuRightMidCliff = sensordata.mcuRightMidCliff;

    pss -> rechargeSign = sensordata.rechargeShrapnel_index;
    //脱困所需数据
    pss -> XAngle = float(sensordata.X_AngleOriginal / 10.0);
    pss -> YAngle = float(sensordata.Y_AngleOriginal / 10.0);
    pss -> ZAngle = sensordata.Z_AngleOriginal;
    pss -> XAcc = sensordata.X_AccOriginal;
    pss -> YAcc = sensordata.Y_AccOriginal;
    pss -> ZAcc = sensordata.Z_AccOriginal;
    pss -> addAngle = sensordata.AddAngle;
    pss-> Initialized = sensordata.Initialized;
    pss -> leftWheelElec = sensordata.leftWheelElectricity_index * 10;
    pss -> rightWheelElec = sensordata.rightWheelElectricity_index * 10;
    pss -> zGyroOriginal = sensordata.Z_GyroOriginal;
    //红外数据
    pss -> leftInfrared = sensordata.leftInfrared_index;
    pss -> rightInfrared = sensordata.rightInfrared_index;
    pss -> leftFrontInfrared = sensordata.leftFrontInfrared_index;
    pss -> rightFrontInfrared = sensordata.rightFrontInfrared_index;
    //虚拟墙
    pss -> leftVir = sensordata.left_virtulwall;
    pss -> rightVir = sensordata.right_virtulwall;
    pss -> leftFrontVir = sensordata.frontLeft_virtulwall;
    pss -> rightFrontVir = sensordata.frontRight_virtulwall;
    pss -> leftBehindVir = sensordata.behindLeft_virtulwall;
    pss -> rightBehindVir = sensordata.behindRight_virtulwall;
    //边刷电流
    pss->leftSideBrushElectricity =  sensordata.leftSideBrushElectricity;
    pss->rightSideBrushElectricity = sensordata.rightSideBrushElectricity;
    if (sensordata.rightOmnibearingTurn_index
        || sensordata.leftOmnibearingTurn_index
        || sensordata.middleOmnibearingTurn_index
        || (pss->laserM > 0.1 && pss->laserM < 0.2))
        pss -> obs = 1;
    else
        pss -> obs = 0;

    if (sensordata.leftGeologicalDetect_index
        || sensordata.rightGeologicalDetect_index
        || sensordata.middleGeologicalDetect_index 
        || sensordata.mcuLeftCliff
        || sensordata.mcuRightCliff
        || sensordata.mcuLeftMidCliff
        || sensordata.mcuRightMidCliff)
        pss -> cliff = 1;
    else
        pss -> cliff = 0;
    
    //虚拟墙信号
    if(sensordata.left_virtulwall || sensordata.right_virtulwall
    || sensordata.frontLeft_virtulwall || sensordata.frontRight_virtulwall
    || sensordata.behindLeft_virtulwall || sensordata.behindRight_virtulwall)
        pss -> magnVirWall = 1;
    else 
        pss -> magnVirWall = 0;

    //FRIZY_LOG(LOG_INFO, "pss.%d.%d.%d",pss -> bump,pss -> obs,pss -> cliff);
        
    pss -> size = 1;
     
    
}

//重新规划栅格地图
void chassisBase::GridPoint(Grid* point)
{
        static float_t jumpX = 0;
        static float_t jumpY = 0;
        static float_t jumpAngle = 0;
        {
            // 锁定共享内存，share_mem_sync在作用域内自动锁定解锁
            //FRIZY_LOG(LOG_INFO, "current_pose is %f",current_pose->theta);   
            share_mem_sync sync(current_pose);

             raw_share_pose.x = current_pose->x;
             raw_share_pose.y = current_pose->y;
             raw_share_pose.theta = current_pose->theta;
             share_pose.is_correcting = current_pose->is_correcting; //矫正参数
             point->correct_index = share_pose.is_correcting;//矫正参数
        }
        FRIZY_LOG(LOG_DEBUG,"real1.%f,%f,%f,%f",raw_share_pose.x,raw_share_pose.y,raw_share_pose.theta,current_pose->timeStamp);

        share_pose.x = (raw_share_pose.x * 100)/15;
        share_pose.y = (raw_share_pose.y * 100)/15;
        share_pose.theta = raw_share_pose.theta *180/_Pi;
        
        FRIZY_LOG(LOG_DEBUG,"real2.%f,%f,%f,%f",share_pose.x,share_pose.y,share_pose.theta,jumpAngle);

        if (fabs(jumpX - share_pose.x) > 0.3 && fabs(jumpY - share_pose.y) <= 0.3)
        {
                //jumpingCount ++;
                printf("JumpingX.%f\n",fabs(jumpX - share_pose.x)*15);
        }
        else if (fabs(jumpX - share_pose.x) <= 0.3 && fabs(jumpY - share_pose.y) > 0.3)
        {
                //jumpingCount ++
                printf("JumpingY.%f\n",fabs(jumpY - share_pose.y)*15);
        }
        else if (fabs(jumpX - share_pose.x) > 0.3 && fabs(jumpY - share_pose.y) > 0.3)
        {
                //jumpingCount ++;
                printf("JumpingXY.%f.%f\n",fabs(jumpX - share_pose.x)*15,fabs(jumpY - share_pose.y)*15);
        }
	if (fabs(jumpAngle - share_pose.theta) > 10 && fabs(jumpAngle - share_pose.theta) < 90)
        {
                //jumpingCount ++;
                printf("JumpingAngle.%f\n",fabs(jumpAngle - share_pose.theta));
        }

        jumpX = share_pose.x;
        jumpY = share_pose.y;
        jumpAngle = share_pose.theta;

        point->realx = share_pose.x;
        point->realy = share_pose.y;

        //重塑XY整数坐标与子坐标
        if (share_pose.x - point->x >= 1) point->x ++;
                
        if (share_pose.x - point->x <= -1) point->x --;	

        if (share_pose.y - point->y >= 1) point->y ++;
                                        
        if (share_pose.y - point->y <= -1) point->y --;		 

        //
        point->dx = 1000*(share_pose.x - point->x);
        point->dy = 1000*(share_pose.y - point->y);


        //计算当前朝向
        //current_pose.theta = int(current_pose.theta);
        if (share_pose.theta <= 0 && share_pose.theta >= -180) 
                point->forward = fabs(share_pose.theta); 
        else
                point->forward = 360 - share_pose.theta;

        // point->forward = share_pose.theta;

        

        //计算累积角度
        int diff = 0;
        int iagg = point->forward;
        if (abs(iagg - lastAngle) > 300)
        {
                if (iagg < lastAngle)
                    diff = 360 - lastAngle + iagg;
                else
                    diff = iagg - 360 -  lastAngle;	
        }
        else
                diff = iagg - lastAngle;


        lastAngle = iagg;

        tempadd = tempadd + diff;
        
        point->addAngle = tempadd;

        return;
}
void chassisBase::getPlanningInfo(Planning_info *planning_info)
{
        
        share_mem_sync sync(current_path_planning);
        planning_info->charger_front_position.x = current_path_planning->frontOfChargingPile.x;
        planning_info->charger_front_position.y = current_path_planning->frontOfChargingPile.y;
        planning_info->charger_seat_position.x = current_path_planning->chargingPilePos.x;
        planning_info->charger_seat_position.y = current_path_planning->chargingPilePos.y;
        planning_info->charger_seat_position.isNew = current_path_planning->chargingPilePos.isNew;
        
        // need to add
        planning_info->cleanBlock->isNew = current_path_planning->selectedBlock->isNew;
        planning_info->cleanPointPos.isNew = current_path_planning->selectedPointPos.isNew;
        FRIZY_LOG(LOG_DEBUG,"charger_front_position.%f,%f , planning_info->cleanPointPos.isNew = %d",current_path_planning->frontOfChargingPile.x,current_path_planning->frontOfChargingPile.y,current_path_planning->selectedPointPos.isNew);
        if(getBlockInfoIdenx ==true && planning_info->cleanBlock->isNew == 1)
        {
                planning_info->cleanBlock->bottomLeftCorner = current_path_planning->selectedBlock->bottomLeftCorner; 
                planning_info->cleanBlock->bottomRightCorner = current_path_planning->selectedBlock->bottomRightCorner;
                planning_info->cleanBlock->topLeftCorner = current_path_planning->selectedBlock->topLeftCorner;
                planning_info->cleanBlock->topRightCorner = current_path_planning->selectedBlock->topRightCorner;
                planning_info->blockCenter.x = (current_path_planning->selectedBlock->bottomLeftCorner.x*100/15 + current_path_planning->selectedBlock->topRightCorner.x*100/15)/2;
                planning_info->blockCenter.y = (current_path_planning->selectedBlock->bottomLeftCorner.y*100/15 + current_path_planning->selectedBlock->topRightCorner.y*100/15)/2;
                current_path_planning->selectedBlock->isNew = 0;
                getBlockInfoIdenx = false;
        }
        // FRIZY_LOG(LOG_DEBUG, "getPointInfoIndex = %d",getPointInfoIndex);
        // FRIZY_LOG(LOG_DEBUG, "planning_info->cleanPointPos.isNew = %d",planning_info->cleanPointPos.isNew);
        if(getPointInfoIndex == true && planning_info->cleanPointPos.isNew == 1)
        {
                
                planning_info->cleanPointPos.x = round_doubel(current_path_planning->selectedPointPos.x*100/15);
                planning_info->cleanPointPos.y = round_doubel(current_path_planning->selectedPointPos.y*100/15);
                planning_info->cleanPointPos.isNew = IS_OLD;
                getPointInfoIndex = false;
                FRIZY_LOG(LOG_DEBUG, "cleanPointPos == %f ,%f",current_path_planning->selectedPointPos.x,current_path_planning->selectedPointPos.y);
        }
        if(getForbbidenInfoIndex == true)
        {
                for(int i =0 ; i< MAX_BLOCK_NBR;i++)
                {
                        if(current_path_planning->prohibitedBlock.Block[i].isNew == 1)
                        {
                                planning_info->forbidenArea.Block[i].bottomLeftCorner = current_path_planning->prohibitedBlock.Block[i].bottomLeftCorner;
                                planning_info->forbidenArea.Block[i].bottomRightCorner = current_path_planning->prohibitedBlock.Block[i].bottomRightCorner;
                                planning_info->forbidenArea.Block[i].topLeftCorner = current_path_planning->prohibitedBlock.Block[i].topLeftCorner;
                                planning_info->forbidenArea.Block[i].topRightCorner = current_path_planning->prohibitedBlock.Block[i].topRightCorner;
                                planning_info->forbidenArea.Block[i].isNew =1;
                                FRIZY_LOG(LOG_DEBUG, " set the  ForbbidenInfoIndex = %d ",i);                  
                        }
                        
                }
                getForbbidenInfoIndex = false;
                FRIZY_LOG(LOG_DEBUG, " update the  ForbbidenInfo ");

        }
        
        

}

WHEELSTATE chassisBase::getWheelState()
{
        GetSensor(&cur_sensor);
        if(IsWall() != EXIT_WALL && cur_sensor.leftw > 0 && cur_sensor.rightw >0)
            return WHEELFIXSPEED;
        else if((cur_sensor.leftw > 0 && cur_sensor.rightw > 0) && abs(cur_sensor.leftw - cur_sensor.rightw) <= 30)
            return WHEELFRONT;
        else if((cur_sensor.leftw < 0 && cur_sensor.rightw < 0) && abs(cur_sensor.leftw - cur_sensor.rightw) <= 30)
            return WHEELBEHIND;
        else if((cur_sensor.leftw < 0 && cur_sensor.leftw < cur_sensor.rightw && abs(cur_sensor.rightw + cur_sensor.leftw) >= 50) || 
            (cur_sensor.rightw > 0 && cur_sensor.rightw > cur_sensor.leftw && abs(cur_sensor.rightw + cur_sensor.leftw) >= 50))
            return WHEELNUILROTLEFT;
        else if((cur_sensor.rightw < 0 && cur_sensor.rightw < cur_sensor.leftw && abs(cur_sensor.rightw + cur_sensor.leftw) >= 50) ||
            (cur_sensor.leftw > 0 && cur_sensor.rightw < cur_sensor.leftw && abs(cur_sensor.rightw + cur_sensor.leftw) >= 50))
            return WHEELNUILROTRIGHT;
        else if(cur_sensor.leftw < cur_sensor.rightw && abs(cur_sensor.leftw + cur_sensor.rightw) <= 20)
            return WHEELLEFTSPIN;
        else if(cur_sensor.leftw > cur_sensor.rightw && abs(cur_sensor.leftw + cur_sensor.rightw) <= 20)
            return WHEELRIGHTSPIN;
        else if(cur_sensor.leftw == 0 && cur_sensor.rightw == 0)
            return WHEELSTOP; 
}

void chassisBase::wheelCtrlStop()
{
    FRIZY_LOG(LOG_DEBUG, "wheelCtrlStop");
    // while(getWheelState() != WHEELSTOP)
    // {
    //     GetSensor(&cur_sensor);
    //     if(abs(cur_sensor.midOmnibearingOn - cur_sensor.midOmnibearingOff) >= 3500)
    //     {
    //         chassisSpeed(0, 0, 1);
    //         return;
    //     }
    //     else 
    //         chassisSpeed(cur_sensor.leftw * 0.5, cur_sensor.rightw * 0.5, 1);
    //     usleep(20 * 1000);
    // }
    GetSensor(&cur_sensor);
    chassisSpeed(cur_sensor.leftw * 0.5, cur_sensor.rightw * 0.5, 1);
    for(int i = 0; i < 20; i++)
    {
        usleep(20 * 1000);
    }
    chassisSpeed(0, 0, 1);
}

int chassisBase::MakeChassisGoStraight(int16_t speed,GridPose targetPose)
{
        FRIZY_LOG(LOG_INFO, "start to chassis go straight ");
        //printf("Speed = %d; mode:goSTRAIGHT")
        robotPose = GetCurGridPose();
        while(fabs(sqrt((robotPose.i - targetPose.i)*(robotPose.i - targetPose.i)+(robotPose.j - targetPose.j)*(robotPose.j - targetPose.j)) )> variance)
        {
           chassisSpeed(speed,goSTRAIGHT);
           robotPose = GetCurGridPose();     
        }
}       

int chassisBase::MakeChassisTurnLeft(int16_t speed, int angle)
{
        robotPose = GetCurGridPose();
        targetAngle = robotPose.forward + angle*_Pi/180;
        //how to rotato angle??? to do
        while(fabs(robotPose.forward - targetAngle) > Diff_angle)
        {chassisSpeed(speed,turnLEFT);
        robotPose = GetCurGridPose();
        }
        FRIZY_LOG(LOG_INFO, "Turn %d left done ",angle);
}
int chassisBase::MakeChassisTurnright(int16_t speed, int angle)
{
        //how to rotato angle??? to do
        robotPose = GetCurGridPose();
        targetAngle = robotPose.forward + angle*_Pi/180;
        while(fabs(robotPose.forward - targetAngle) > Diff_angle)
        {
        chassisSpeed(speed,turnRIGHT);
        robotPose = GetCurGridPose();
        }
        FRIZY_LOG(LOG_INFO, "Turn %d right done ",angle);
}
int chassisBase::MakeChassisGoTurn(int16_t leftspeed,int16_t rightspeed,int16_t times)
{
        FRIZY_LOG(LOG_DEBUG, "start to excute go turn ");
        targetRobotPose = GetCurGridPose();
        for(int i =0 ;i <times;i++)
        {
              chassisSpeed(leftspeed,rightspeed,1);
              robotPose = GetCurGridPose();
              while(sqrt((robotPose.i - targetRobotPose.i)*(robotPose.i - targetRobotPose.i)+(robotPose.j - targetRobotPose.j)*(robotPose.j - targetRobotPose.j))>0.1)//判断条件太生硬
              {
                   chassisSpeed(leftspeed,rightspeed,1);   
              }
        }
}

int chassisBase::MakeBaseRecharge()
{
        FRIZY_LOG(LOG_INFO, "start to excute base Recharge ");
        chassisRecharge();
}
int chassisBase::MakeBaseEscapeJail()
{
        FRIZY_LOG(LOG_INFO, "start to excute base escapejail ");
        escapeJail();
}
int chassisBase::MakeBaseAlongWall(uint8_t mode,int direction)
{
        if(direction == right_allwalldirection){
                alongWall(mode,RightAlongwall);
        }
        if(direction == left_allwalldirection){
                alongWall(mode,LeftAlongwall);
        }
}
int chassisBase::MakeSmartRecharge()
{

}
int chassisBase::chassisSpeed(int16_t speed,int way)
{       
        
        int16_t leftWhell = 0;
        int16_t rightWhell = 0;

        if(way == goSTRAIGHT){
                leftWhell = speed;
                rightWhell = speed;  
        }
        //how to rotato angle??? to do
        if (way == turnLEFT){
                leftWhell = -speed;
                rightWhell = speed;
        }
        //how to rotato angle??? to do
        if(way == turnRIGHT){
                leftWhell = speed;
                rightWhell = -speed;
        }   


        if(GLOBAL_CONTROL==WHEEL_RUN)
        {
            FRIZY_LOG(LOG_INFO, "start to excute the chassisspeed ");
            UMAPI_MainWheelSpeed(1,leftWhell,rightWhell);}
        if(GLOBAL_CONTROL == WHEEL_PAUSE || GLOBAL_CONTROL == WHEEL_STOP)
        {
                FRIZY_LOG(LOG_INFO, "start to stop the chassisspeed ");
                UMAPI_MainWheelSpeed(1,0,0);
        }               
        return 0;
}

// int chassisBase::chassisSpeed(int16_t leftspeed,int16_t rightspeed,int mode)
// {
//         memset(&controlData,0,sizeof(controlData));
//         controlData.cmd = CHASSIS_CMD_ROAD;
//         controlData.len = 10;

//         controlData.data[0] =0xFF;
//         controlData.data[1] =0xFE;
//         controlData.data[2] =0x42;
//         controlData.data[3] =0x05;
//         if(leftspeed < 0 ){
//                 leftspeed = leftspeed + 0xFFFF;
//         }
//         if(rightspeed < 0 ){
//                 rightspeed = rightspeed + 0xFFFF;
//         }
//         controlData.data[4] = (leftspeed & 0x00FF);
//         controlData.data[5] = (leftspeed & 0xFF00) >> 8;
//         controlData.data[6] = (rightspeed & 0x00FF);
//         controlData.data[7] = (rightspeed & 0xFF00) >> 8;
//         controlData.data[8] = mode; //1 :normal mode 2:accelerate mode 
//         //calc 校验
//         for(int i = 0; i < controlData.len -1; i++){
//                 controlData.data[9] ^= controlData.data[i];
//         }
//         //共享内存写入 umweritemem ;
//         // UM_WriteControlMem(controlData);
//         return 0;        

// }

bool chassisBase::getSpeedOrder()
{
    return speedOrder;
}

int lastLeft = 0;
int lastRight = 0;
int lastLeft2 = 0;
int lastRight2 = 0;
static int stopTime = 0;
int chassisBase::chassisSpeed(int16_t leftspeed,int16_t rightspeed,int mode)
{
        if(leftspeed == 0 && rightspeed == 0)
            speedOrder = 0;
        else 
            speedOrder = 1;
        if(GLOBAL_CONTROL==WHEEL_RUN)
        {

                if (IsWall() == 0)
                {
                        if (leftspeed > 0 && rightspeed > 0 && lastLeft + lastRight == 0 && lastLeft2 + lastRight2 == 0  
                                && mode != 100)
                        {
                                stopTime = 3;
                        }

                        lastLeft2 = lastLeft;
                        lastRight2 = lastRight;
                        lastLeft = leftspeed;
                        lastRight = rightspeed;
                        FRIZY_LOG(LOG_DEBUG,"send.%d.%d",lastLeft,lastRight);
                        if (stopTime > 0)
                        {
                                FRIZY_LOG(LOG_DEBUG,"'stop!'!");
                                MotionControl chassisMotion;
                                chassisMotion.ClearPid();
                                leftspeed = 0,rightspeed = 0;
                                FRIZY_LOG(LOG_DEBUG,"'the process = %d , calibration_time = %d ",process,calibration_time);
                                if(process == PLAN && calibration_time >2400)
                                {
                                        calibration_time = 0; 
                                        Angle_calibration();                                        
                                }
                                stopTime --;   
                        }
                }
                // FRIZY_LOG(LOG_INFO, "start to excute the chassisspeed ");
                if(leftspeed == 0 && rightspeed == 0)
                {
                        FRIZY_LOG(LOG_INFO,"robot stop spin");
                }
                if (mode == 100) mode = 1;
                UMAPI_MainWheelSpeed(1,leftspeed,rightspeed);
        }
        if (mode == 100) mode = 1;
        if(GLOBAL_CONTROL == WHEEL_PAUSE || GLOBAL_CONTROL == WHEEL_STOP)
        {
                FRIZY_LOG(LOG_INFO, " start to stop chassisspeed  ");
                UMAPI_MainWheelSpeed(1,0,0);
        }
        return 0;        

}

int chassisBase::chassisRoadFIX()
{
        
}
int chassisBase::escapeJail()
{


        UMAPI_CtrlWalkState(7);
        return 0;
}
int chassisBase::chassisRecharge()
{
        //标准回冲


        FRIZY_LOG(LOG_INFO, "start to recharge ");
        UMAPI_RobotMode(5);
        return 0;
}


int chassisBase::alongWall(uint8_t mode,uint8_t direction)
{
        if (mode == 1) wall = 1;
        if (mode == 0) wall = 0;
        if (mode == 1 && direction == 2) wall = 2;
        
        
        //unwerite
        // UM_WriteControlMem(controlData);
        FRIZY_LOG(LOG_INFO, "start to alongwalk ");
        UMAPI_RobotMode(3);
        return 0;
}
void chassisBase::leaveChargerParaInit()
{
        leaveCharger_backTime = 200;
        leaveCharger_backsign = 1;
        leaveCharger_rotateSign = 1;
        leaveCharger_aimForward = 0.0;
        leaveCharger_recordForward = 0.0;
}
static bool gyo_correct =false;
int chassisBase::rechargeRetreat()
{
        
        FRIZY_LOG(LOG_INFO,"ENTER TO rechargeRetreat Mode");
        GetSensor(&curSensor);
        //是否在回充座上
        FRIZY_LOG(LOG_INFO,"curSensor.rechargeSign:%d",curSensor.rechargeSign);
        if(!curSensor.rechargeSign)
        {
               
                FRIZY_LOG(LOG_ERROR,"robot no charge");
                // return -1;
        }
        FRIZY_LOG(LOG_INFO,"从回充座上下来");
        leaveChargerParaInit();
        // FRIZY_LOG(LOG_INFO,"THE GLOBAL_CONTROL = %u",GLOBAL_CONTROL);
        while(GLOBAL_CONTROL == WHEEL_RUN)
        {       
                if(GLOBAL_CONTROL != WHEEL_RUN)
                {
                        FRIZY_LOG(LOG_ERROR,"robot not in wheel_run mode");
                        chassisSpeed(0, 0, 1);
                        usleep(100*1000);
                        return -1;
                }
                //后退50cm
                GetSensor(&curSensor);
                usleep(20 * 1000);
                // FRIZY_LOG(LOG_INFO,"curSensor.rechargeSign:%d , leaveCharger_backTime: %d",curSensor.rechargeSign,leaveCharger_backTime);
                if(leaveCharger_backTime&&(GLOBAL_CONTROL == WHEEL_RUN))
                {
                        chassisSpeed(-100, -100, 0);
                        leaveCharger_backTime --;
                        if(leaveCharger_backTime)
                                {
                                        continue;
                                        // FRIZY_LOG(LOG_ERROR,"continue test");
                                }
                        else
                        {
                                if(leaveCharger_backsign)
                                {
                                        chassisSpeed(0, 0, 1);
                                        leaveCharger_backsign = 0;
                                        FRIZY_LOG(LOG_INFO,"backtime:%d,后退完毕,发送停止轮速",leaveCharger_backTime);
                                }
                        }
                }
                
                if(curSensor.Initialized == 1)
                // if(curSensor.XAcc == 0.0 &&curSensor.XAngle == 0.0&&curSensor.YAcc ==0 && curSensor.YAngle ==0 &&curSensor.ZAcc ==0&& curSensor.ZAngle ==0)
                {
                      FRIZY_LOG(LOG_INFO,"陀螺仪初始化成功,等待旋转");
                      gyo_correct = true;
                                           
                }
                else
                {
                     usleep(100*1000);
                     gyo_correct = false;
                     continue;
                     
                }

                if(gyo_correct == true)
                {                        
                        if(leaveCharger_rotateSign)
                        {
                                usleep(3000*1000);
                                FRIZY_LOG(LOG_INFO,"确定目标角度");
                                // GridPoint(&curGrid);
                                // recordForward = curGrid.forward;
                                leaveCharger_aimForward = gyo_angle_*180/_Pi + 180;
                                if(leaveCharger_aimForward >= 360)
                                        leaveCharger_aimForward = leaveCharger_aimForward - 360;
                                FRIZY_LOG(LOG_DEBUG,"recordForward: %f, aimforward: %f",leaveCharger_recordForward, leaveCharger_aimForward);
                                leaveCharger_rotateSign = 0;
                        }
                        // if(gyo_angle_*180/_Pi != 0)
                        {
                        chassisSpeed(100,-100,1);
                        // GridPoint(&curGrid);
                        GetSensor(&curSensor);
                        FRIZY_LOG(LOG_DEBUG,"rotate cursensor.leftw: %d,cursensor.rightw: %d",curSensor.leftw,curSensor.rightw);
                        FRIZY_LOG(LOG_DEBUG,"当前角度: %f,目标角度:%f",gyo_angle_*180/_Pi,leaveCharger_aimForward);
                        if(fabs(leaveCharger_aimForward - gyo_angle_*180/_Pi) > 0 && fabs(leaveCharger_aimForward - gyo_angle_*180/_Pi) < 3)
                        {
                                FRIZY_LOG(LOG_INFO, "ROTATE SUCCESSFUL");     
                                chassisSpeed(0,0,1);
                                usleep(20 * 1000);
                                gyo_correct =false;
                                return 0;
                        }
                        }
                }                 
        }
}

// void chassisBase::gyo_calibration()
// {
        
//         if(GLOBAL_CONTROL != WHEEL_RUN)
//         {
//                 gyo_calibration_rotate_times = 50; 
//                 gyo_calibration_forward_times = 50;                
//         }
//         FRIZY_LOG(LOG_INFO, "GYO CALIBRATION");
//         FRIZY_LOG(LOG_DEBUG,"start to rotate for gyo_calibration");
//         if(gyo_calibration_rotate_times&&GLOBAL_CONTROL == WHEEL_RUN)
//         {
//                 chassisSpeed(100,-100,1);
//                 gyo_calibration_rotate_times-- ;
//                 return;  
//         }
//         FRIZY_LOG(LOG_DEBUG,"start to forward for gyo_calibration");
//         if(gyo_calibration_forward_times&&GLOBAL_CONTROL == WHEEL_RUN)
//         {
//                 chassisSpeed(100,100,0);
//                 gyo_calibration_forward_times--;
//                 return;
//         }

//         FRIZY_LOG(LOG_DEBUG,"start to send the calibration sign to mcu");
//         UMAPI_CtrlGyro(1,NULL,NULL);
//         GetSensor(&curSensor);
//         if(curSensor.Initialized != 1 && GLOBAL_CONTROL == WHEEL_RUN)
//         {
                
//                 if(curSensor.Initialized != 1)
//                 {
//                         usleep(20*1000);
//                         GetSensor(&curSensor);
//                         gyo_record_times ++;
//                         if(gyo_record_times > 200)
//                         {
//                                 gyo_calibration_index =false;
//                                 gyo_record_times =0;
//                                 return;
//                         }
//                         return;
                        
                        
//                 }
//                 else if (curSensor.Initialized ==1 )
//                 {
//                         // if(gyo_calibration_rotate_times == 0 && gyo_calibration_forward_times == 0)
//                         // {
//                         //         gyo_calibration_rotate_times = 100; 
//                         //         gyo_calibration_forward_times = 100;
//                         // }
//                         GetSensor(&curSensor);
//                         if(gyo_calibration_for_X_Y_angle_times)
//                         {
//                                 gyo_calibration_for_X_Y_angle_times-- ;
//                                 chassisSpeed(100,-100,1);
//                                 min(minXAngle ,curSensor.XAngle);
//                                 max(maxXAngle,curSensor.XAngle);
//                                 min(minYAngle ,curSensor.YAngle);
//                                 max(maxYAngle,curSensor.YAngle);                      
//                                 return;
                                
//                         }
//                         if(fabs(minXAngle -maxXAngle)<3 && fabs(minYAngle -maxYAngle)<3)
//                         {
//                                 gyo_calibration_index =true;
//                         }
//                         else
//                         {
//                                 gyo_calibration_index =false;
//                         }
                        
                        
//                 }
//         }

        

// }
void chassisBase::relocation()
{
        
        

}
static int calibration_state;
static int tmp_state;
static void gyro_stat_resp(int calibration_state)
{
        FRIZY_LOG(LOG_DEBUG,"gyro_stat_resp function");
        tmp_state = calibration_state;
}

static int calibration_index = 60;
void chassisBase::Angle_calibration()
{
        FRIZY_LOG(LOG_DEBUG,"start to Angle_calibration in uturn mode"); 
        // chassisSpeed(0,0,1);
        UMAPI_CtrlGyro(2,gyro_stat_resp,&tmp_state);
        // usleep(2500*1000);

        while(calibration_index&&tmp_state!=1)
        {
                usleep(100*1000);
                FRIZY_LOG(LOG_DEBUG,"wait for calibration for mcu ");
                calibration_index--;

        }
        FRIZY_LOG(LOG_DEBUG,"resume the mcu mode to auto "); 
        calibration_index = 60;
        UMAPI_RobotMode(1);
}

}