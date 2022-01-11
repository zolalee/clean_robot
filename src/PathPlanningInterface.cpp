/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  :
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-11 10:26:09
 * @Project      : UM_path_planning
 */

#include "PathPlanningInterface.h"
#include <sys/time.h>
#ifndef __UMUSERMODE__
#define __UMUSERMODE__ 444
#endif
// #include <functional>

static int backTime;
static const char *TAG = "server";
static long long lastTime;
double time_ = 0.025;			 //Robot signal interval
double x_  = 0.0;			 //Odometer x coordinates
double y_  = 0.0;			 //Odometer y coordinates
float angle_ = 0.0,gyo_angle_ =0 ;		 //Odometer angle
double time_now,lw_now,rw_now,as_now;				 //Robot signal data at current time
double lw_pre,rw_pre,as_pre;						 //Robot signal data of the previous time
double time_pre = -1;								 //The initialization timestamp is used to determine the start time
double lw,rw,as,l,x_delta, y_delta,angle_delta; 	 //Intermediate variables required for odometer data calculation

extern uint8_t run_road_flag;
extern int cur_x;
extern int cur_y;
useerobot::Maze _maze;

BlockCorner selectBlockCorner ;
namespace useerobot
{
    int recharge_first_index = 0;
	extern wheel_mode wheel_state;
    extern int lastLeft;
    extern int lastRight;
    int areaClean = 0;
    Grid tempAim;
    int spinSpeed = 0;
    share_map_t* current_full_map;
    dynamicMapInfo_t* dynamicMap; 

    allDirectionPointInfo_t* laserDis;

    // extern UserSensorData_t *sensordata;
    //PathPlanningInterface interfaces
    PathPlanningInterface::PathPlanningInterface()
    {
        // cleanCycleInit();
    }
    PathPlanningInterface::~PathPlanningInterface()
    {
        // some case to exit to addwansho
        delete motion_control;
        robot_current_state.planningState = IDLE;
        FRIZY_LOG(LOG_ERROR, "PLANNING MODULE EXIT");
    }

    void PathPlanningInterface::handle_navigation_reqeust(struct RobotCleanReq *req)
    {

        rev_message.data = req->data;
        rev_message.header = req->header;
        RobotCleanData rev_data;

        log_info(TAG, "got a reqet seq=%d scene=%d", rev_message.header.seq, rev_message.data.scene);
        if (rev_message.header.data_type == kNavigationMsgRobotPlanningStateData)
        {
            FRIZY_LOG(LOG_INFO,"  the order is get robot status request\n");
            getRobotState();
        }
        // using f1 = void(*)(RobotCleanReq);
        if(rev_message.header.data_type == kNavigationMsgRobotCleanData)
        {
            FRIZY_LOG(LOG_INFO,"  the order is planning request\n");
            cleanPlanningRun(rev_message);
        }
    }
    void _SensorDataCallback(SensorDataCallback_t callback)
    {


    }
    static void gyro_data_report(GyroData_t *data)
    {
        sensordata.X_AccOriginal = data->X_AccOriginal;
        sensordata.X_GyroOriginal = data->X_GyroOriginal;
        sensordata.Y_AccOriginal = data->Y_AccOriginal;
        sensordata.Y_GyroOriginal = data->Y_GyroOriginal;
        sensordata.Z_AccOriginal = data->Z_AccOriginal;
        sensordata.Z_GyroOriginal = data->Z_GyroOriginal;
        sensordata.X_AngleOriginal = data->X_AngleOriginal;
        sensordata.Y_AngleOriginal = data->Y_AngleOriginal;
        sensordata.Z_AngleOriginal = data->Z_AngleOriginal;
        sensordata.AddAngle = data->AddAngle;
		as_now = sensordata.Z_GyroOriginal;
        // FRIZY_LOG(LOG_INFO,"data->Z_GyroOriginal = %d\n",  data->Z_GyroOriginal);
    }
    static void mainwheel_data_report(MainWheelData_t *data)
    {
		static uint8_t first_run_flag = 1;

        sensordata.LeftWheel_Speed = data->LeftWheel_Speed;
        sensordata.RightWheel_Speed = data->RightWheel_Speed;
		lw_now = data->LeftWheel_Speed;
		rw_now =  data->RightWheel_Speed;
        sensordata.leftWheelElectricity_index = data->leftWheelElectricity;
        sensordata.rightWheelElectricity_index = data->rightWheelElectricity;


        if(process != ROAD)
        {
            x_ = 0.0;
            y_ = 0.0;
            lw_pre = 0.0;
            rw_pre = 0.0;
        }


        as = 0.5 *(as_pre + as_now);   //角速度
        as_pre = as_now;
        angle_delta = (as * time_) * 0.03056482125;
        angle_delta = angle_delta / 180.0 * _Pi;
        if((lw_now != 0) || (rw_now != 0))	//左右轮速
        {
        gyo_angle_ += angle_delta;

        }

        if (gyo_angle_ > 2*_Pi)
        {
            gyo_angle_ = (gyo_angle_ - 2*_Pi);
        }

        if (gyo_angle_ < 0)
        {
            gyo_angle_ = 2*_Pi + gyo_angle_;
        }
        // if(process == ROAD)
		{
			//Calculate the X and Y coordinates of the odometer at the current time, using a two wheel motion model
            angle_ = raw_share_pose.theta;
            if (angle_ <= 0 && angle_ >= -_Pi) 
                    angle_ = fabs(angle_); 
            else
                    angle_ = 2*_Pi- angle_;
            
            
            x_ =raw_share_pose.x;
            y_ =raw_share_pose.y;
		}
    }
    static void alert_mode_report(AlertMode_t *data)
    {
        if (data) {
            #if 0
            printf("\n");
            printf("alert mode = %u\n", *data); //警报模式
            printf("\n");
            #endif
        }
    }
    static void robot_mode_report(RobotMode_t *data)
    {

    }
    static void alongwall_data_report(AlongWallData_t *data)
    {
        sensordata.leftAlongWallValue = abs(data->leftAlongWallOnVal - data->leftAlongWallOffVal);
        sensordata.rightAlongWallValue = abs(data->rightAlongWallOnVal - data->rightAlongWallOffVal);
    }

    static void omnibearing_data_report(OmnibearingData_t *data)
    {
        sensordata.leftOmnibearingSlow_index = data->leftOmnibearingSlow;
        sensordata.leftOmnibearingTurn_index = data->leftOmnibearingTurn;
        sensordata.middleOmnibearingSlow_index = data ->middleOmnibearingSlow;
        sensordata.middleOmnibearingTurn_index = data->middleOmnibearingTurn;
        sensordata.rightOmnibearingSlow_index = data->rightOmnibearingSlow;
        sensordata.rightOmnibearingTurn_index = data->rightOmnibearingTurn;

        sensordata.leftOmnibearingOn_index = data->leftOmnibearingOnVal;
        sensordata.rightOmnibearingOn_index = data->rightOmnibearingOnVal;
        sensordata.midOmnibearingOn_index = data->middleOmnibearingOnVal;
        sensordata.leftOmnibearingOff_index = data->leftOmnibearingOffVal;
        sensordata.rightOmnibearingOff_index = data->rightOmnibearingOffVal;
        sensordata.midOmnibearingOff_index = data->middleOmnibearingOffVal;
    }

    static void grdcheck_data_report(GeologicalDetect_t *data)
    {
        sensordata.leftGeologicalDetect_index =data->leftGeologicalDetect;
        sensordata.middleGeologicalDetect_index = data->middleGeologicalDetect;
        sensordata.rightGeologicalDetect_index = data->rightGeologicalDetect;
    }

    static void sidebrush_data_report(SideBrushData_t *data)
    {
       sensordata.leftSideBrushElectricity = data->leftSideBrushElectricity;
       sensordata.rightSideBrushElectricity = data->rightSideBrushElectricity;
    //    FRIZY_LOG(LOG_DEBUG,"  leftSideBrushElectricity =  %d , rightSideBrushElectricity = %d\n",data->leftSideBrushElectricity,data->rightSideBrushElectricity);
    }

    static void ps_data_report(PowerSupplyData_t *data)
    {
        // FRIZY_LOG(LOG_INFO,"sensordata->BatVoltage %d\n",  data->batVoltage);
        sensordata.batVolume = data->batVolume;
        // FRIZY_LOG(LOG_INFO,"sensordata->BatVoltage %f\n",  sensordata.batVolume);
    }

    static void ir_data_report(InfraredData_t *data)
    {   
        if(((data->leftInfraredData >> 16) & 39233) & 9941)
            sensordata.leftInfrared_index = 1;
        else 
            sensordata.leftInfrared_index = 0;

        if(((data->rightInfraredData >> 16) & 39233) & 9941)
            sensordata.rightInfrared_index = 1;
        else 
            sensordata.rightInfrared_index = 0;

        if(((data->frontLeftInfraredData >> 16) & 39233) & 9941)
            sensordata.leftFrontInfrared_index = 1;
        else 
            sensordata.leftFrontInfrared_index = 0;

        if(((data->frontRightInfraredData >> 16) & 39233) & 9941)
            sensordata.rightFrontInfrared_index = 1;
        else 
            sensordata.rightFrontInfrared_index = 0;
        
        // if (data) {
        //     #if 0
        //     printf("\n");
            // printf("leftInfraredData = %u\n", data->leftInfraredData);              //左红外数据
            // printf("rightInfraredData = %u\n", data->rightInfraredData);            //右红外数据
            // printf("frontLeftInfraredData = %u\n", data->frontLeftInfraredData);    //前左红外数据
            // printf("frontRightInfraredData = %u\n", data->frontRightInfraredData);  //前右红外数据
            // printf("behindLeftInfraredData = %u\n", data->behindLeftInfraredData);  //后左红外数据
            // printf("behindRightInfraredData = %u\n", data->behindRightInfraredData);//后右红外数据
            // printf("\n");
        //     #endif
        // }
    }

    static void fan_pulse_report(uint16_t fanPulse)
    {

    }

    static void wakeup_timing_report(WakeupTiming_t *data)
    {
        #if 0
        //printf("\n");
        //printf("rechargeSeatWakeup = %u\n", data->rechargeSeatWakeup);  //回充座唤醒
        //printf("keyWakeup = %u\n", data->keyWakeup);                    //按键唤醒
        //printf("irWakeup = %u\n", data->irWakeup);                      //红外唤醒
        //printf("rtcWakeup = %u\n", data->rtcWakeup);                    //RTC唤醒
        //if (data->timing == 1)
        //    printf("timing = %u\n", data->timing);                          //定时
        //printf("\n");
        #endif
    }
    static void peripheral_detect_report(PeripheralState_t *data)
    {
        sensordata.rechargeShrapnel_index = data->rechargeShrapnel;
        // printf("data->rechargeShrapnel = %u\n", data->rechargeShrapnel);
        #if 0
        printf("\n");
        printf("\n");
        printf("\n");
        #endif
    }
    static void power_detect_report(PowerState_t *data)
    {
        #if 0
        printf("\n");

        printf("\n");
        #endif
    }
    static void appointment_resp_report(AppointmentAction_t *data)
    {
        #if 0
        printf("\n");
        printf("appointmentTimeClear = %u\n", data->appointmentTimeClear);    //清除预约时间
        printf("appointmentTimeCancel = %u\n", data->appointmentTimeCancel);  //预约时间取消
        printf("appointmentTimeSet = %u\n", data->appointmentTimeSet);        //预约时间设定
        printf("currentTimeSet = %u\n", data->currentTimeSet);                //当前时间设定
        printf("\n");
        #endif
    }
    static void imu_data_report(IMU_t *data)
    {
        #if 0

        #endif
    }
    static void bumper_data_report(BumperData_t *data)
    {
        // 遥控控制
        if (data->left || data->right) {
            auto remote_controller = RemoteControllerTask::getInstance();
            auto command = std::make_shared<RemoteControllerTask::ControlCommand>();
            command->collision = 1;
            remote_controller->addCommand(command);
            log_info(TAG, "got collision left=%d right=%d", data->left, data->right);
        }

        if(data->left == 0 &&data->right == 0)

        sensordata.Bump_Motor = 0;
        if(data->left == 1 &&data->right == 0)

        sensordata.Bump_Motor = 1;
        if(data->right == 1 && data->left == 0)

        sensordata.Bump_Motor = 2;
        if(data->right == 1 && data->left == 1)

        sensordata.Bump_Motor = 3;

    }
    static void virtual_wall_report(VirtualWall_t *data)
    {
        sensordata.left_virtulwall = data->left;              
        sensordata.right_virtulwall = data->right;             // 右虚拟墙
        sensordata.frontLeft_virtulwall = data->frontLeft;         // 前左虚拟墙
        sensordata.frontRight_virtulwall = data->frontRight;        // 前右虚拟墙
        sensordata.behindLeft_virtulwall = data->behindLeft;        // 后左虚拟墙
        sensordata.behindRight_virtulwall = data->behindRight;       // 后右虚拟墙
        
    }
    static void key_event_report(int key, int evt)
    {
        printf("\n\n\n");
        printf("key = %d, evt = %d\n", key, evt);

        switch (key)
        {
        case KEY_IR_AUTOCLEAN:                //弓字形清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_AUTOCLEAN ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_AUTOCLEAN ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_AUTOCLEAN ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_AUTOCLEANSTOP:            //停止弓字形清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_AUTOCLEANSTOP ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_AUTOCLEANSTOP ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_AUTOCLEANSTOP ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_RECHARGE:                 //回充
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_RECHARGE ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_RECHARGE ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_RECHARGE ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_RECHARGESTOP:             //停止回充
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_RECHARGESTOP ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_RECHARGESTOP ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_RECHARGESTOP ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_ALONGWALLCLEAN:           //沿墙清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_ALONGWALLCLEAN ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_ALONGWALLCLEAN ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_ALONGWALLCLEAN ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_ALONGWALLCLEANSTOP:       //停止沿墙清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_ALONGWALLCLEANSTOP ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_ALONGWALLCLEANSTOP ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_ALONGWALLCLEANSTOP ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_FIXEDPOINTCLEAN:          //定点清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_FIXEDPOINTCLEAN ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_FIXEDPOINTCLEAN ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_FIXEDPOINTCLEAN ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_FIXEDPOINTCLEANSTOP:      //停止定点清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_FIXEDPOINTCLEANSTOP ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_FIXEDPOINTCLEANSTOP ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_FIXEDPOINTCLEANSTOP ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_OKSUSPEND:                //开始、暂停
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_OKSUSPEND ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_OKSUSPEND ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_OKSUSPEND ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_FRONT_KEY:                //前按键
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_FRONT_KEY ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_FRONT_KEY ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_FRONT_KEY ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_LEFT_KEY:                 //左按键
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_LEFT_KEY ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_LEFT_KEY ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_LEFT_KEY ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_RIGHT_KEY:                //右按键
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_RIGHT_KEY ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_RIGHT_KEY ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_RIGHT_KEY ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_BEHIND_KEY:               //后按键
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_BEHIND_KEY ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_BEHIND_KEY ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_BEHIND_KEY ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_FINDSWEEPER:              //寻找扫地机
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_FINDSWEEPER ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_FINDSWEEPER ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_FINDSWEEPER ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_WATERFANGEARLOW:          //水量风机低
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARLOW ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARLOW ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARLOW ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_WATERFANGEARMID:          //水量风机中
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARMID ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARMID ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARMID ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_WATERFANGEARHIGH:         //水量风机高
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARHIGH ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARHIGH ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_WATERFANGEARHIGH ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_RANDOMCLEAN:              //随机清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_RANDOMCLEAN ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_RANDOMCLEAN ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_RANDOMCLEAN ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_RANDOMCLEANSTOP:          //停止随机清扫
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_RANDOMCLEANSTOP ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_RANDOMCLEANSTOP ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_RANDOMCLEANSTOP ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        case KEY_IR_APPOINTMENTCLEAR:         //预约记录清除
            if (evt == EVENT_KEY_SHORT_PRESS) {
                //按键处理程序
                printf("KEY_IR_APPOINTMENTCLEAR ==> EVENT_KEY_SHORT_PRESS\n");
            }
            if (evt == EVENT_KEY_DOUBLE_PRESS) {
                //按键处理程序
                printf("KEY_IR_APPOINTMENTCLEAR ==> EVENT_KEY_DOUBLE_PRESS\n");
            }
            if (evt == EVENT_KEY_LONG_PRESS) {
                //按键处理程序
                printf("KEY_IR_APPOINTMENTCLEAR ==> EVENT_KEY_LONG_PRESS\n");
            }
            break;

        default:
            break;
        }
    }
    void PathPlanningInterface::SetMap(int cols,int rows,int maptype)
    {
        Maze maze_temp(cols,rows);

        FRIZY_LOG(LOG_INFO,"  the map size is %d * %d\n",maze_temp.cols,maze_temp.rows);
        maze_temp.setMaze(cols,rows);

        for(int i =0; i < maze_temp.cols;i++)
        {
            for(int j =0; j < maze_temp.rows;j++)
            {
                maze_temp.Map[i][j]= new Point(i,j,0);
                maze_temp.recordMap[i][j]= new Point(i,j,0);
            }
        }

        _maze = maze_temp;
        // FRIZY_LOG(LOG_INFO,"the _maze recordmap size = %d \n",_maze.recordMap.size());
    }
    void PathPlanningInterface::InitRecordMap()
    {
        FRIZY_LOG(LOG_INFO,"  init the record map \n");
        FRIZY_LOG(LOG_DEBUG,"  the record map size is %d * %d\n",_maze.cols,_maze.rows);
        for(int i =0; i < _maze.cols;i++)
        {
            for(int j =0; j < _maze.rows;j++)
            {
                _maze.recordMap[i][j]->n = 0;
            }
        }        
    }
    void PathPlanningInterface::InitGlobalMap()
    {
        FRIZY_LOG(LOG_INFO,"  init the Global map \n");
        for(int i =0; i < _maze.cols;i++)
        {
            for(int j =0; j < _maze.rows;j++)
            {
                _maze.Map[i][j]->n = 0;
            }
        }        
    }    
    void PathPlanningInterface::Init()
    {

        FRIZY_LOG(LOG_INFO,"init all 01-10-1\n");
        spinSpeed = 0;
        tempAim.x = current_pos.x,tempAim.y = current_pos.y,tempAim.forward = current_pos.forward;
        
        InitRecordMap();
        InitGlobalMap();

        chassis.GridPoint(&current_pos);

        current_pos.x = current_pos.realx;
        current_pos.y = current_pos.realy;

        process = PLAN;

        ///
        arch.init();
        block.init();
        road.init();
        escape.Init();
        record_charge_aim.x =0;
        record_charge_aim.y =0;
        if (IsWall() != 0)
            StopWallFollow();

        //
        motion.ClearPid();
        fisrt_correcting_map = true;
        // 初始化禁区
        chassis.getPlanningInfo(&current_planning_info);
        _maze.current_planning_info = current_planning_info;
        // _maze.setForbindenInfo();
        _maze.mapArea = 0;
        thread_index = true;
        areaClean = 0;
        current_clean_area.clean_area = 0;
        remap_recharge_index = false;
        // batvolume_Index = true;

        //road.roadDwa.
    }
    void PathPlanningInterface::stop_task()
    {

        FRIZY_LOG(LOG_INFO,"init stop_task \n");
        spinSpeed = 0;
        tempAim.x = current_pos.x,tempAim.y = current_pos.y,tempAim.forward = current_pos.forward;
        
        // InitRecordMap();
        // current_pos.x = 0;
        // current_pos.y = 0;

        process = PLAN;

        ///
        arch.init();
        block.init();
        road.init();
        escape.Init();
        record_charge_aim.x =0;
        record_charge_aim.y =0;
        if (IsWall() != 0)
            StopWallFollow();
            
        motion.ClearPid();
        fisrt_correcting_map = true;
        //
        // wallstate_t == EXIT_WALL;
        thread_index = true;
        // batvolume_Index = true;

        //road.roadDwa.
    }
    void PathPlanningInterface::clear_state()
    {
        process = PLAN;

        ///
        arch.init();
        block.init();
        road.init();
        escape.Init();
        last_cleanTaskOverIndex = false;
        cleanTaskOverIndex = false;
        _maze.mapArea = 0;
        //
        motion.ClearPid();

    } 
    void PathPlanningInterface::updateRobotstate()
    {
        // share_mem_sync sync(robotstate);
        share_mem_lock(robotstate);
        // FRIZY_LOG(LOG_DEBUG, "share_mem_sync sync(robotstate) ");
        if(IsWall()!=0)
        {
            robotstate->robot_state = 1;
        FRIZY_LOG(LOG_DEBUG, "robotstate = ALONGWALL ");
        }
        else
        {
           robotstate->robot_state = 0;
           FRIZY_LOG(LOG_DEBUG, "robotstate = UTURN");
        }
        
        share_mem_unlock(robotstate);
    }
    void PathPlanningInterface::cleanCycleInit()
    {
        FRIZY_LOG(LOG_INFO, "start to init the clean cycle  PLANNING_NODE version 1.0.01.06");
        ModLogDef_t logs[] = {
            {DEBUG_TRANSCEIVER, DEBUG_DESC_TRANSCEIVER},
            {DEBUG_PROTOCOL, DEBUG_DESC_PROTOCOL},
            {DEBUG_ASYNC_INVOKE, DEBUG_DESC_ASYNC_INVOKE},
            {DEBUG_IPC, DEBUG_DESC_IPC},
            {DEBUG_KEY, DEBUG_DESC_KEY},
            {DEBUG_UART, DEBUG_DESC_UART},
            {DEBUG_CLIFF, DEBUG_DESC_CLIFF},
            {DEBUG_OBS, DEBUG_DESC_OBS},
            {DEBUG_DPM, DEBUG_DESC_DPM},
            {DEBUG_DPM_IPC, DEBUG_DESC_DPM_IPC}
        };
        logger_init(logs, sizeof(logs)/sizeof(ModLogDef_t), NULL);
        logger_level(ELEVEL_DEBUG); 

        ModMask_t mask = 0;
        mask |= (1<<DEBUG_TRANSCEIVER);
        // mask |= (1<<DEBUG_IPC);
        mask |= (1<<DEBUG_KEY);
        mask |= (1<<DEBUG_UART);
        mask |= (1<<DEBUG_CLIFF);
        mask |= (1<<DEBUG_OBS);
        mask |= (1<<DEBUG_DPM);
        mask |= (1<<DEBUG_DPM_IPC);
        logger_module_disable(mask);
                    
        SensorDataCallback_t sensordata_cb = {
        .gyro_data_cb = gyro_data_report,
        .mainwheel_data_cb = mainwheel_data_report,
        .alert_mode_cb = alert_mode_report,
        .robot_mode_cb = robot_mode_report,
        .alongwall_data_cb = alongwall_data_report,
        .omnibearing_data_cb = omnibearing_data_report,
        .grdcheck_data_cb = grdcheck_data_report,
        .sidebrush_data_cb = sidebrush_data_report,
        .ps_data_cb = ps_data_report,
        .ir_data_cb = ir_data_report,
        .fan_pulse_cb = fan_pulse_report,
        .wakeup_timing_cb = wakeup_timing_report,
        .peripheral_detect_cb = peripheral_detect_report,
        .power_detect_cb = power_detect_report,
        .appointment_resp_cb = appointment_resp_report,
        .key_event_cb = key_event_report,
        .imu_data_cb = imu_data_report,
        .bumper_data_cb = bumper_data_report,
        .virtual_wall_cb = virtual_wall_report
        };
        UMAPI_ChassisProtocolInit();
        UMAPI_ChassisCallbackRegister(sensordata_cb);

        // 分配共享共存，一块共享内存使用name来标识，
        // 在不同进程中使用同一个名字分配，打开的就是同一块内存。
        current_pose = (current_pose_t *)share_mem_alloc("share_pose", sizeof(*current_pose));
        //FRIZY_LOG(LOG_INFO, "current_pose theta is %f", current_pose->theta);
        if (!current_pose) {
        //log_error(POSE_TAG, "Alloc share pose failed");
        printf("Alloc share pose failed\n");
        perror("\033[41;30m Alloc share pose failed! \033[0m");
        exit(0);
        }
        memset(current_pose, 0, sizeof(current_pose_t));
        current_full_map = (share_map_t *)share_mem_alloc("share_map", sizeof(*current_full_map));
        if (!current_full_map) {
        //log_error(POSE_TAG, "Alloc share pose failed");
        printf("Alloc share map failed\n");
        perror("\033[41;30m Alloc share map failed! \033[0m");
        exit(0);}
        memset(current_full_map, 0, sizeof(share_map_t));

        //
        laserDis = (allDirectionPointInfo_t *)share_mem_alloc("share_allDirection", sizeof(*laserDis));
        if (!laserDis)
        {
            printf("share_app_info Alloc  memory_share.p_share_app_info failed\n");
            perror("\033[41;30m Alloc  memory_share.p_share_app_info! \033[0m");
            exit(0);
        }
        memset(laserDis, 0, sizeof(allDirectionPointInfo_t));
        
        dynamicMap = (dynamicMapInfo_t *)share_mem_alloc("share_dynamicMap", sizeof(*dynamicMap));
        if (!dynamicMap) {

            printf("Alloc dynamicmap failed\n");
            perror("\033[41;30m Alloc dynamicmap failed! \033[0m");
            exit(0);
        }
        memset(dynamicMap, 0, sizeof(dynamicMapInfo_t));

        robotstate = (Robotstate *)share_mem_alloc("robot_state", sizeof(*robotstate));
        if (!robotstate) {
        //log_error(POSE_TAG, "Alloc share pose failed");
        printf("Alloc robotstate failed\n");
        perror("\033[41;30m Alloc robotstate failed! \033[0m");
        exit(0);}
        memset(robotstate, 0, sizeof(Robotstate));

        current_path_planning = (share_path_planning_t *)share_mem_alloc("share_app_Info", sizeof(*current_path_planning));
        if (!current_path_planning) {
        //log_error(POSE_TAG, "Alloc share pose failed");
        printf("Alloc current_path_planning failed\n");
        perror("\033[41;30m Alloc current_path_planning failed! \033[0m");
        exit(0);}
        memset(current_path_planning, 0, sizeof(share_path_planning_t));

        dynamicMap = (dynamicMapInfo_t *)share_mem_alloc("share_dynamicMap", sizeof(*dynamicMap));
        if (!dynamicMap) {

        printf("Alloc dynamicmap failed\n");
        perror("\033[41;30m Alloc dynamicmap failed! \033[0m");
        exit(0);}
        memset(dynamicMap, 0, sizeof(dynamicMapInfo_t));

        // 临时使能接口，后面需要替换
        // UMAPI_CtrlWifiState(robot_sta_automatic,wifi_sta_islink,ir_ctrl_none,fan_gear_none,water_gear_none);
        // UMAPI_RobotMode(1);
        GLOBAL_CONTROL = WHEEL_RUN;
        robot_current_state.planningState = IDLE;
        CURRENT_GAMEPAD_STATE =GAMEPAD_CONTROL_ON;
        robot_current_clean_scene = STOP;
        motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
        // if(UMAPI_RobotMode(1) == -1)
        //     FRIZY_LOG(LOG_ERROR, "Robot Mode Control failed");
        // else
        //     FRIZY_LOG(LOG_INFO, "Robot Mode Control success");
        //初始化map
        SetMap(501,501,1);
        sleep(1);
        // 开启各服务线程
        _maze.mapUpdateStart();//开启地图服务
        slamPathPLanning_thread_ = std::make_shared<std::thread>(&PathPlanningInterface::SLAMPathPLanning, this);//建图清扫服务
        Alongwall_task.alongwallStart();//沿墙服务
        rondomPLanning_thread_ = std::make_shared<std::thread>(&PathPlanningInterface::randomClean, this); //随机清扫服务
        leaveCharger_thread_ =  std::make_shared<std::thread>(&PathPlanningInterface::LeaveCharger, this); // 下回充座服务
        // 建立Ｍｅｓｓａｇｅ　ｓｅｒｖｅｒ
        std::string server_address = "/tmp/navigation_socket";
        server = std::make_shared<NavigationRpcServer>();
        handler = std::make_shared<NavigationServerHandler>();
        handler->_planning_state.planningState = IDLE; // 机器人的状态需要在全局函数中实时的更新
        remote_controller = std::make_shared<RemoteControllerTask>();
        remote_controller->setMotionControl(motion_control);
        remote_controller->start();
        log_init("server", NULL);

        handler->registerRobotCleanCallback(std::bind(&useerobot::PathPlanningInterface::handle_navigation_reqeust,this,std::placeholders::_1));
        server->setServerAddress(server_address);
        server->setRpcHandler(handler);
        success = server->init();
        if (success == false) {
            log_error(TAG, "init server failed");
            exit(0);
        }
        handler->sendRobotPlanningState(robot_current_state);
        handler->start();
        server->start();
        bool log_mark = true;
        // alongwall task init
        // WallFallowPara.alongwalk_run_index = false;
        // alongwall_thread_ = std::make_shared<std::thread>(&AlongWall::chassisAlongWall, this);
        
        // Alongwall_task.alongwallStart();


        // motion_control->robotchassiscontrol.rechargeRetreat();

        while (true)
        {
            msleep(100);
            if (handler->isTransceiverAlive() && log_mark) {
            // if (handler->isRecvAlive() && handler->isSendAlive() && log_mark) {
                log_info(TAG, "server handler got connection");
                log_mark = false;
            }

        }
    }

    void PathPlanningInterface::cleanPlanningRun(struct RobotCleanReq &rev)
    {
        if(success == true)
        {
            log_info(TAG, "cleanPlanningRun start");

        }
        RobotCleanData rev_data;
        rev_data.scene = rev_message.data.scene;
        switch (rev_data.scene)
        {
        case 0:
            robot_current_clean_scene = RobotCleanScene::WHOLETRAVERSEROOMS;
            /* code */ //全屋逐房遍历 & 有自动分房和没有自动分房
            break;
        case 1:
            robot_current_clean_scene = RobotCleanScene::WHOLETRAVERSEBLOCKS;
            /* code */ //区块遍历
            break;
        case 2:
            {
            {
                {
                clear_state();
                Init();            
                robot_current_clean_scene = RobotCleanScene::SLAMPlANNING;
                FRIZY_LOG(LOG_INFO, "START TO  SLAMPathPLanning MODE");
                GLOBAL_CONTROL = WHEEL_RUN;
                slamPathPLanning_state_index = true;
                _maze.setForbindenInfo(left_charger_index);
                // prepare_thread_.reset();
                // prepare_thread_ = std::make_shared<std::thread>(&PathPlanningInterface::SLAMPathPLanning, this);
                
                log_info(TAG, "slamplanning start");
                }

                // sleep(1);
                // {
                    
                //     robot_current_state.planningState=SEARCHING_CHARGER;
                //     robot_current_clean_scene = RobotCleanScene::CHARGE;
                //     FRIZY_LOG(LOG_INFO, "START TO MOVE CHARGER");
                //     GLOBAL_CONTROL = WHEEL_RUN;
                //     RoadAim tmp_aim ;
                //     tmp_aim.x =-1;
                //     tmp_aim.y =0;
                //     tmp_aim.kind = recharge;
                //     setChargeMode(tmp_aim);
                // }

            }           
            break;
            }
        case 3:
            {
                robot_current_clean_scene = RobotCleanScene::TRAVERSESELECTEDROOMS;
                /* code */ //指定房间遍历
                break;
            }
        case 4:
            {
                FRIZY_LOG(LOG_INFO, "SWICTH TO  SLAMPathPLanning MODE"); //重新激活规划清扫模式
                GLOBAL_CONTROL = WHEEL_RUN;
                slamPathPLanning_state_index = true;
                _maze.setForbindenInfo(left_charger_index);
                robot_current_clean_scene = RobotCleanScene::GOTOCLEANPOINT;
                FRIZY_LOG(LOG_INFO, "START TO  POINTCLEAN MODE");
                internal_planning.pointCleanTime=1200;
                robot_current_state.planningState=HEADING;
                getPointInfoIndex = true;
                while (getPointInfoIndex == true)
                {
                  chassis.getPlanningInfo(&current_planning_info);
                  usleep(20*1000);
                }
                pointCleanPosition.x = current_planning_info.cleanPointPos.x;
                pointCleanPosition.y = current_planning_info.cleanPointPos.y;
                pointCleanIdex = true ;
                FRIZY_LOG(LOG_DEBUG, "pointCleanIdex= %d",pointCleanIdex);
                break;
            }
        case 5:
            {
                
                FRIZY_LOG(LOG_INFO, "SWICTH TO  SLAMPathPLanning MODE"); //重新激活规划清扫模式
                GLOBAL_CONTROL = WHEEL_RUN;
                slamPathPLanning_state_index = true;
                
                _maze.setForbindenInfo(left_charger_index);
                robot_current_clean_scene = RobotCleanScene::GOTOBLOCKCLEAN;
                FRIZY_LOG(LOG_INFO, "START TO  BLOCKCLEAN MODE");


                robot_current_state.planningState=HEADING;
                getBlockInfoIdenx = true;
                while(getBlockInfoIdenx ==true)
                {
                  chassis.getPlanningInfo(&current_planning_info);
                  usleep(20*1000);
                }
                _maze.current_planning_info.cleanBlock->bottomLeftCorner = current_planning_info.cleanBlock->bottomLeftCorner;
                _maze.current_planning_info.cleanBlock->bottomRightCorner = current_planning_info.cleanBlock->bottomRightCorner;
                _maze.current_planning_info.cleanBlock->topLeftCorner = current_planning_info.cleanBlock->topLeftCorner;
                _maze.current_planning_info.cleanBlock->topRightCorner = current_planning_info.cleanBlock->topRightCorner;
                blockCenterPoint = current_planning_info.blockCenter;

                _maze.setMapArea();
                blockCleanIndex = true;
                
                break;
            }
        case 6:
            {
                // if(robot_current_state.planningState != SEARCHING_CHARGER)
                {
                    
                    //用于测试的随机模式下
                    // internal_planning.randomClean_start_index = false; // 随机切换到规划模式;
                    FRIZY_LOG(LOG_INFO, "SWICTH TO  SLAMPathPLanning MODE"); //重新激活规划清扫模式
                    GLOBAL_CONTROL = WHEEL_RUN;
                    slamPathPLanning_state_index = true;
                    _maze.setForbindenInfo(left_charger_index);
                    if(current_pos.correct_index != 0)
                    {
                        remap_recharge_index = true;
                    }
                    else
                    {
                        rechargeTask();
                    }


                }
         
                break;
            }
        case 7:
            {
                {
                robot_current_clean_scene = RobotCleanScene::ALONGWALL;
                FRIZY_LOG(LOG_INFO, "START TO ALONGWALL CLEAN MODE");
                _maze.setForbindenInfo(left_charger_index);
                GLOBAL_CONTROL = WHEEL_RUN;
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                }           
                break;
            }
        case 8:
            robot_current_clean_scene = RobotCleanScene::RELOCALIZATION;
            /* code */ //重定位
            break;
        case 9:
            // robot_current_clean_scene = RobotCleanScene::STOP;
            stopClean(); //停止任务
            break;
        case 10:
            robot_current_clean_scene = RobotCleanScene::PAUSE;
            suspendClean(); //暂停任务
            break;
        case 11:
            robot_current_clean_scene = RobotCleanScene::RESUME;
            resumeClean(); //恢复任务
            break;
        case GAMEPAD:
        {
            if(robot_current_clean_scene == STOP || robot_current_clean_scene == GAMEPAD ||robot_current_clean_scene ==REMOTECONTROL)
            {robot_current_clean_scene = RobotCleanScene::GAMEPAD;
            robot_current_state.planningState = IDLE;
            GLOBAL_CONTROL = WHEEL_RUN;
            FRIZY_LOG(LOG_INFO, "CURRENT_GAMEPAD_STATE = %d ",CURRENT_GAMEPAD_STATE);
            // if(CURRENT_GAMEPAD_STATE ==GAMEPAD_CONTROL_ON)
            // gamepadControl(); //遥控器控制
            auto command = std::make_shared<RemoteControllerTask::ControlCommand>();
            command->gamepad = rev.data.gamepadCommand;
            command->ir = 0;
            remote_controller->addCommand(command);
            log_info(TAG, "got gamepad command %d", rev.data.gamepadCommand);}

            else
                {
                    
                    FRIZY_LOG(LOG_INFO, "robot_current_clean_scene = %d",robot_current_clean_scene);
                    FRIZY_LOG(LOG_ERROR, "ROBOT STATE IS NOT IN EDLE  CAN NOT RESPOND");
                }
            break;
        }
        case REMOTECONTROL:
        {
            if(robot_current_clean_scene == STOP || robot_current_clean_scene == GAMEPAD ||robot_current_clean_scene ==REMOTECONTROL)
            {robot_current_clean_scene = RobotCleanScene::REMOTECONTROL;
            robot_current_state.planningState = IDLE;
            GLOBAL_CONTROL = WHEEL_RUN;
            FRIZY_LOG(LOG_INFO, "CURRENT_GAMEPAD_STATE = %d ",CURRENT_GAMEPAD_STATE);
            auto command = std::make_shared<RemoteControllerTask::ControlCommand>();
            command->gamepad = 0;
            command->ir = rev.data.remoteControlCommand;
            remote_controller->addCommand(command);
            log_info(TAG, "got remote control command %d", rev.data.remoteControlCommand);}

            else
                {
                    
                    FRIZY_LOG(LOG_INFO, "robot_current_clean_scene = %d",robot_current_clean_scene);
                    FRIZY_LOG(LOG_ERROR, "ROBOT STATE IS NOT IN EDLE  CAN NOT RESPOND");
                }
            break;
        }
        case LEFT_CHARGER:
        {
            
            {
                robot_current_clean_scene = RobotCleanScene::LEFT_CHARGER;
                GLOBAL_CONTROL = WHEEL_RUN;
                FRIZY_LOG(LOG_INFO, "START TO LEAVE THE CHAGER ");
                StopWallFollow();
                // chassis.leave_recharge_index = true;
                left_charger_index = true;
                // leaveCharger_thread_.reset();
                // leaveCharger_thread_ = std::make_shared<std::thread>(&PathPlanningInterface::LeaveCharger, this);
                leaveCharger_task_state = true;
            }
			break;
        }    
        case RONDOM:        
        {
            if(robot_current_state.planningState  == IDLE)
            {
                FRIZY_LOG(LOG_INFO, "START TO RONDOM CLEAN MODE");
                robot_current_clean_scene = RobotCleanScene::RONDOM;
                GLOBAL_CONTROL = WHEEL_RUN;
                thread_index = true;
                internal_planning.randomClean_start_index = true;
                StopWallFollow();

            }         
			break;
        }
        case FORBIDDEN_INFO_UPDATE:
        {
            FRIZY_LOG(LOG_INFO, "START TO FORBIDDEN_INFO_UPDATE");
            getForbbidenInfoIndex = true;
            while(getForbbidenInfoIndex ==true)
                {
                    usleep(100*1000);
                  chassis.getPlanningInfo(&current_planning_info);

                }
            _maze.current_planning_info = current_planning_info;
            _maze.setForbindenInfo(left_charger_index);

        }

        case RESUME_BROKEN_CLEAN:
        {
            FRIZY_LOG(LOG_INFO, "START TO RESUME BROKEN CLEAN");
            robot_current_clean_scene = RobotCleanScene::SLAMPlANNING;
            GLOBAL_CONTROL = WHEEL_RUN;
            slamPathPLanning_state_index = true;
            // batvolume_Index = true;
        }
        default:
        


            break;
        }

    }
    void PathPlanningInterface::getRobotState()
    {
        log_info(TAG, "update the robote status and get the robote status");
        handler->_planning_state.planningState = robot_current_state.planningState;

    }
    void PathPlanningInterface::cleanStartRun()
    {
        FRIZY_LOG(LOG_INFO, "start to clean ");

    }

    void PathPlanningInterface::cleanCycleDestroy()
    {

        FRIZY_LOG(LOG_INFO, "destroy  clean cycle ");

    }


    // void PathPlanningInterface::getCurrentPose()
    // {

    // }

    void PathPlanningInterface::divideRoomToBlock(BlockCorner &selectBlockCorner,RobotType &robotShape)
    {

    }
    void PathPlanningInterface::pointClean(Point &sweepPoint,RobotType &robotShape)
    {
        FRIZY_LOG(LOG_INFO, "start enter the  pointClean mode");
    }
    void PathPlanningInterface::sensorMemCallBack(SensorData_t sensorData)
    {

    }

    //这两个函数与获取和转换当前时间有关
    long long GetCurrentTime()
    {
        struct timeval time;

        gettimeofday(&time,NULL);

        return ((long long)time.tv_sec * 1000000 + (long long)time.tv_usec);
    }




    void ControlPeriod(int PERIOD)
    {
        long long curTime = GetCurrentTime();

        //printf("GetCurrentTime1.%lld\n",curTime);

        if (curTime - lastTime > 0 && curTime - lastTime < PERIOD)
        {
            int tempTime = PERIOD - int(curTime - lastTime);

            usleep(tempTime);
        }
        else if (curTime - lastTime <= 0)
        {
            usleep(PERIOD);
        }
        else
        {
            printf("too long !!\n");
        }

        FRIZY_LOG(LOG_INFO,"time2.%lld",GetCurrentTime() - lastTime);
        lastTime = GetCurrentTime();
    }

    
    
    //矩形试探
    void PathPlanningInterface::RectangleTest(Sensor sensor,Grid cur)
    {
        

	   if (sensor.bump != 0 || sensor.obs != 0 || abs(cur.x - tempAim.x) + abs(cur.y - tempAim.y) > 3)
        {
            FRIZY_LOG(LOG_DEBUG,"juxingchufa");
            spinSpeed = 1;
            tempAim.x = cur.x;
            tempAim.y = cur.y;
            
            if (cur.forward >= 0 && cur.forward < 270)
                tempAim.forward = cur.forward + 90;
            else
                tempAim.forward = cur.forward - 270;
		}

		
        if (spinSpeed == 1)
        {
		    if (abs(tempAim.forward - cur.forward) >10)
		    {
				chassis.chassisSpeed(120, -120, 1);
			}
			else
			{
				spinSpeed = 0;
			}
	    }
	    else
        {
            FRIZY_LOG(LOG_DEBUG,"juxingzhixing");
			chassis.chassisSpeed(200, 200, 1);
        }
        

		FRIZY_LOG(LOG_DEBUG,"bump.%d,tempAim.%d,%d,%f"
            ,sensor.bump,tempAim.x,tempAim.y,tempAim.forward);
	
    }
    
    
    
    void PathPlanningInterface::AppControl(){

        if (blockCleanIndex == true){
           
            blockCleanIndex = false;
            stop_task();

            arch.UTURN = ARCH;
            arch.ARCH_STATE = NO_ARCH;
           
            process = ROAD;
            RoadAim _aim;
            _aim.kind = zoning;
            _aim.x = blockCenterPoint.x;
            _aim.y = blockCenterPoint.y;
            areaClean = 1;
            for (int x = -200;x <= 200;++x)
               for (int y = -200;y <= 200;++y)
                    if (_maze.GetMapState(x,y,2) == 1)
                        _maze.InputRecord(x,y,0); 

            FRIZY_LOG(LOG_DEBUG,"huaqu.%d.%d",_aim.x,_aim.y);
                usleep(200*1000);
            road.SetroadAim(_aim);            
        }
        if(pointCleanIdex == true)
        {
            pointCleanIdex = false;
            stop_task();
            process = ROAD;
            RoadAim _aim;
            _aim.kind = appoint;
            _aim.x = pointCleanPosition.x;
            _aim.y = pointCleanPosition.y;;
            FRIZY_LOG(LOG_DEBUG,"point clean %d.%d",_aim.x,_aim.y);
            usleep(200*1000);
            // for(int i =-30 ; i<30;i++)
            // {
            //     for(int j = -30;j<30;j++)
            //     { 
            //         printf("@300=%04d,%04d,00%d\n",i,j,_maze.GetMapState(i,j,1));
            //     }
            // }
            road.SetroadAim(_aim);  
        }

    }




    void PathPlanningInterface::SLAMPathPLanning()
    {
        
        // SetMap(800,800);
        Init();
        // Alongwall_task.alongwallStart();
        robot_current_state.planningState = PLANNING;

        FRIZY_LOG(LOG_INFO, "GLOBAL_CONTROL CYCLE");

        while (1)
        {

            //控制周期
               
            if(slamPathPLanning_state_index != true)
            {
                usleep(2000*1000);
                continue;
            }

            if(slamPathPLanning_state_index == true)
            {
            
            ControlPeriod(50 * 1000);
            time_t now;    
            struct tm *tm_now;       
            time(&now);      
            tm_now = localtime(&now);   
            FRIZY_LOG(LOG_DEBUG,"Date: %d-%d-%d %d:%d:%d", tm_now->tm_year + 1900,           
                        tm_now->tm_mon + 1, tm_now->tm_mday, tm_now->tm_hour,         
                                tm_now->tm_min, tm_now->tm_sec);   

            //机器人状态上报
            updateRobotstate(); 
            
            //上传地图面积
            uploadArea();
            //脱困处理  
            escapescene();         

            //获取传感器和当前位置

            chassis.GetSensor(&current_sensor);

            chassis.GridPoint(&current_pos);


            if(current_sensor.magnVirWall == 1)
            {
                FRIZY_LOG(LOG_INFO, "UPDATE RECHARGE SEAT POSITION : %d %d",record_charge_aim.x,record_charge_aim.y); 
                record_charge_aim.x = current_pos.x;
                record_charge_aim.y = current_pos.y;
            }
            // if(current_sensor.batvolume<20 && batvolume_Index == true)
            // {
            //     FRIZY_LOG(LOG_WARNING,"LOW BATVOLUME");
            //     robot_current_state.planningState = BROKEN_CLEAN_RECHARGE;
            //     handler->sendRobotPlanningState(robot_current_state);
            //     FRIZY_LOG(LOG_INFO, "SEND  BROKEN_CLEAN_RECHARGE MESSAGE TO SERVICE ");
            //     batvolume_Index = false;
            //     rechargeTask();             
            // }
            FRIZY_LOG(LOG_DEBUG,"x == %d,y == %d,bumpstate.%d,left.%d,right.%d,size.%d,angle.%f,add.%d"
                 ,current_pos.x,current_pos.y,current_sensor.bump,current_sensor.leftw,current_sensor.rightw
                 ,current_sensor.size,current_pos.forward,current_pos.addAngle);

            FRIZY_LOG(LOG_DEBUG,"backtime.%d,process.%d,UTURN.%d,ARCH_STATE.%d,iswall.%d"
             ,backTime,process,arch.UTURN,arch.ARCH_STATE,IsWall());
            if((road.call_recharge_index == true&&recharge_first_index == 1) || remap_recharge_index == true)
            {
                robot_current_state.planningState = RECHARGER_REACHED;
                handler->sendRobotPlanningState(robot_current_state);
                slamPathPLanning_state_index = false;
                GLOBAL_CONTROL = WHEEL_STOP;
                StopWallFollow();
                Init();
                FRIZY_LOG(LOG_INFO, "SEND  RECHARGER_REACHED MESSAGE TO SERVICE "); 
            }
            FRIZY_LOG(LOG_INFO, "last_cleanTaskOverIndex = %d , cleanTaskOverIndex = %d",last_cleanTaskOverIndex,cleanTaskOverIndex); 
            if(last_cleanTaskOverIndex == false && cleanTaskOverIndex == true)
            {
                robot_current_state.planningState = CLEAN_FINISHED;
                handler->sendRobotPlanningState(robot_current_state);
                FRIZY_LOG(LOG_INFO, "SEND  CLEAN_FINISHED MESSAGE TO SERVICE "); 
                robot_current_state.planningState = SEARCHING_CHARGER;
                handler->sendRobotPlanningState(robot_current_state);
                FRIZY_LOG(LOG_INFO, "SEND  SEARCHING_CHARGER MESSAGE TO SERVICE ");                 
            }
            last_cleanTaskOverIndex = cleanTaskOverIndex;
            if (SignManage(current_sensor,current_pos))
                continue;
                
            
            //等待地图矫正
            
            FRIZY_LOG(LOG_DEBUG, "current_pos.correct_index　＝　%d",current_pos.correct_index);
            if (current_pos.correct_index != 0)
            {
                FRIZY_LOG(LOG_DEBUG, "等待地图矫正");
                if (fisrt_correcting_map == true)
                    {
                        Init();
                        FRIZY_LOG(LOG_DEBUG, "map correctted init info");
                        fisrt_correcting_map = false;
                    }               

                chassis.chassisSpeed(120,-120,1);
                //RectangleTest(current_sensor,current_pos);


                continue;           
            }
            fisrt_correcting_map = true;
            //更新地图
            _maze.RecordMap(current_sensor,current_pos);

            if (current_sensor.bump)
                FRIZY_LOG(LOG_DEBUG,"bump!!.%d",current_sensor.bump);
            
            //脱困
            Trouble trouble = escape.EscapeRecognition(current_sensor,current_pos);

            if (trouble.type == smallarea)
            {                
                FRIZY_LOG(LOG_INFO,"start to do smallarea escape");
                escape.EscapeSmallArea();   
                
            }
            
            AppControl();
        

            //程序执行
            switch (process)
            {
            case ROAD:
                FRIZY_LOG(LOG_DEBUG,"ROAD");
                road.StartRoad(current_sensor,current_pos,trouble);
                break;

            case BOUND:
                FRIZY_LOG(LOG_DEBUG,"BOUND");
                block.DivArea(current_sensor,current_pos,trouble);
                break;

            case PLAN:
                FRIZY_LOG(LOG_DEBUG,"PLAN");
                arch.InsidePlanning(current_sensor,current_pos,trouble);
                break;
            case POINT:
                FRIZY_LOG(LOG_DEBUG,"POINT");
                if(internal_planning.pointCleanTime>0)
                {
                    internal_planning.pointClean(cur_RobotType);
                    break;
                }
                
                rechargeTask();
                break;
            default:
                break;
            }
            }

        }
        // thread_index = false;

    }

    Point* PathPlanningInterface::judgeBlockCorner(const Point &startPoint, const BlockCorner &selectBlockCorner, RobotType &robotShape)
    {

    }

    void PathPlanningInterface::appointBlockClean(BlockCorner &selectBlockCorner,RobotType &robotShape)
    {
        
    }

    void PathPlanningInterface::movingCharge()
    {


        Alongwall_task.alongwallStart();

        if(chargeFunction(-1*0.15,0)==0)
        {
            robot_current_state.planningState  = IDLE;
        };

    }

    void PathPlanningInterface::alongWallClean()
    {
        thread_index = true;
        Alongwall_task.alongwallStart();
        sleep(1);
        int back_time =0 ;
        while(GLOBAL_CONTROL == WHEEL_RUN)
        {
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            usleep(50*1000);
            back_time++;
        }
    }
    void PathPlanningInterface::autoClean(RobotType &robotShape)
    {
        if(autodividerooms == true)
        {
            FRIZY_LOG(LOG_INFO, "start enter Intelligent cleaning mode ");

        }
        else
        {
            FRIZY_LOG(LOG_INFO, "start enter General cleaning mode ");
        }
    }

    void PathPlanningInterface::suspendClean()
    {
        FRIZY_LOG(LOG_INFO, "start pause clean");
        if(GLOBAL_CONTROL == WHEEL_RUN)
        {GLOBAL_CONTROL = WHEEL_PAUSE;
        robot_current_state.planningState = SUSPEND;}
        else
        {
            FRIZY_LOG(LOG_ERROR, "THe current mode is not Run MODE, CAN NOT TO SUSPEND");
        }

    }
    void  PathPlanningInterface::resumeClean()
    {
        FRIZY_LOG(LOG_INFO, "start resume clean");
        if(GLOBAL_CONTROL == WHEEL_PAUSE)
        {GLOBAL_CONTROL = WHEEL_RUN;
        robot_current_state.planningState = PLANNING;
        }

        else
        {
            FRIZY_LOG(LOG_ERROR, "THe current mode is not PAUSE MODE, CAN NOT TO RESUME");
        }
    }
    void  PathPlanningInterface::stopClean()
    {
        FRIZY_LOG(LOG_INFO, "stop the clean task");
        {

            motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
            GLOBAL_CONTROL = WHEEL_STOP;
            StopWallFollow();
            internal_planning.randomClean_start_index = false;
            rechargeindex = -1;
            FRIZY_LOG(LOG_INFO, "robot_current_clean_scene = %d",robot_current_clean_scene); 
            switch (robot_current_clean_scene)
            {
                case SLAMPlANNING:
                {
                    FRIZY_LOG(LOG_INFO, "stop the task for SLAMPlANNING");  
                    // Alongwall_task.alongwallStop();
                    slamPathPLanning_state_index = false;
                    stop_task();
                    // prepare_thread_->join();
                    // prepare_thread_ = nullptr;
                    robot_current_clean_scene = RobotCleanScene::STOP;
                    break ;
                }
                case GOTOCLEANPOINT:
                {
                    FRIZY_LOG(LOG_INFO, "stop the task for POINTCLEAN");
                    slamPathPLanning_state_index = false;
                    stop_task();
                    robot_current_clean_scene = RobotCleanScene::STOP;
                    break ;
                }
                case GOTOBLOCKCLEAN:
                {
                    
                    FRIZY_LOG(LOG_INFO, "stop the task for BLOCKCLEAN");
                    slamPathPLanning_state_index = false;
                    stop_task();
                    robot_current_clean_scene = RobotCleanScene::STOP;
                    break;
                }
                case LEFT_CHARGER:
                {
                    FRIZY_LOG(LOG_INFO, "stop the task for LEFT_CHARGER");  

                    // leaveCharger_thread_->join();
                    // leaveCharger_thread_ = nullptr;
                    leaveCharger_task_state = false;
                    chassis.leaveChargerParaInit();
                    robot_current_clean_scene = RobotCleanScene::STOP;
                    break ;
                }
                case RONDOM:
                {
                    FRIZY_LOG(LOG_INFO, "stop the task for RONDOM CLEAN");  
                    // Alongwall_task.alongwallStop();
                    // prepare_thread_->join();
                    // prepare_thread_ = nullptr; 
                                       
                    robot_current_clean_scene = RobotCleanScene::STOP;
                    break ;
                } 
                case ALONGWALL:
                {
                    FRIZY_LOG(LOG_INFO, "stop the task for ALONGWALL MODE");  
                    // Alongwall_task.alongwallStop();
                    // prepare_thread_->join();
                    // prepare_thread_ = nullptr;
                    
                    robot_current_clean_scene = RobotCleanScene::STOP;
                    break ;
                }
                case CHARGE:
                {
                    FRIZY_LOG(LOG_INFO, "stop the task for MOVECHARGE MODE");
                    // prepare_thread_->join();
                    // prepare_thread_ = nullptr;
                    slamPathPLanning_state_index = false;

                    break ;
                }
                default:
                {
                    slamPathPLanning_state_index = false;
                    stop_task();
                    robot_current_clean_scene = RobotCleanScene::STOP;                    
                    break;
                }
                    

            }
            usleep(100*1000);
            StopWallFollow();
            FRIZY_LOG(LOG_INFO, "clean the map data success");
            thread_index = false;
            robot_current_state.planningState  = IDLE;
            road.call_recharge_index = false;

        }
    }
    int PathPlanningInterface::SignManage(Sensor sensor,Grid cur)
    {
        if (process != ROAD)
            road.roadState = roadIdle;

        //碰撞后退
        if (backTime == 0 && IsWall() == 0 && lastLeft > 0 && lastRight > 0)
            // && ((wheel_state == forward && lastLeft > 0 && lastRight > 0)
            //     || process == ROAD))
        {
            if (sensor.bump != 0){
                FRIZY_LOG(LOG_DEBUG,"realbump");
                backTime = 10;
            }
                
            if (sensor.cliff != 0){
                FRIZY_LOG(LOG_DEBUG,"realcliff");
                backTime = 15;   
            }
                             
        }


        if (backTime)
        {
            backTime --;

            if (backTime > 1)
            {
                FRIZY_LOG(LOG_DEBUG,"back!");
                chassis.chassisSpeed(-150,-150,1);
                return 1;
            }
        }
        current_sensor.bump = backTime;
         
        if (current_sensor.obs)
            FRIZY_LOG(LOG_DEBUG,"obs1.%d",current_sensor.obs);
        // current_sensor.obs = 0;

        static int obsStop = 0;
        if (!current_sensor.bump && current_sensor.obs
            && IsWall() == 0 && lastLeft > 0 && lastRight > 0)
        {
            FRIZY_LOG(LOG_DEBUG,"obs2.%d",current_sensor.obs);
            obsStop = 1;
		}	

        if (obsStop)
        {
            if (sensor.leftw != 0 || sensor.rightw != 0)
            {
                do{
                    StopWallFollow();
                    chassis.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0);  
                FRIZY_LOG(LOG_DEBUG,"stopping");
                return 1;
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"obsstate");
                current_sensor.obs = obsStop;
                obsStop = 0;
            }
        }
        else
        {
            current_sensor.obs = 0;
        }
        
        return 0;
    }

    void PathPlanningInterface::ClearRecodrdMap()
    {
            for(int i =0; i < _maze.cols;i++)
            {
                for(int j =0; j < _maze.rows;j++)
                {
                    // delete _maze.Map[i][j];
                    _maze.recordMap[i][j]->n =0;
                }
            }
    }
    void PathPlanningInterface::LeaveCharger()
    {
        // FRIZY_LOG(LOG_DEBUG, "rechargeindex = chassis.rechargeRetreat(); ");
        thread_index = true;
        while(1)
        {
            if(leaveCharger_task_state ==false)
            {
                usleep(100*1000);
                continue;
            }
            if(leaveCharger_task_state == true)
            {
                rechargeindex = chassis.rechargeRetreat();
                if(rechargeindex == 0)
                {
                    robot_current_state.planningState = PREPARED_PLANNING;
                    handler->sendRobotPlanningState(robot_current_state);
                    FRIZY_LOG(LOG_INFO, "SEND THE PREPARED_PLANNING STATE TO SERVICE ");
                    
                }
                else
                {
                    robot_current_state.planningState = TASK_FAILED;
                    handler->sendRobotPlanningState(robot_current_state);
                }
                leaveCharger_task_state = false;
            }
        }
    }


    void PathPlanningInterface::randomClean()
    {
        FRIZY_LOG(LOG_INFO,"set randomclean mode");
        internal_planning.randomClean();
    }

    void PathPlanningInterface::escapescene()
    {
        escape.recordPitRol_10s();
        int tmp_escap =0;
        if(escape.ESCAPE_StuckTrigCheck_My_Test())
        {
            chassis.chassisSpeed(0, 0, 1);
            if(IsWall() != EXIT_WALL)
                {
                    StopWallFollow();
                    tmp_escap = 1;
                }
            if(escape.escDeal(tmp_escap)==true)
            {
                FRIZY_LOG(LOG_INFO,"escap successful");
                if(tmp_escap == 1)
                {
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                }
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"escap failed");

            }
        }
        else if(escape.ESCAPE_LeftFloorCheck())
        {
            chassis.chassisSpeed(0, 0, 1);
            if(IsWall() != EXIT_WALL)
                {
                    StopWallFollow();
                    tmp_escap = 1;
                }
            if(escape.escDeal(tmp_escap)==true)
            {
                FRIZY_LOG(LOG_INFO,"escap successful");
                if(tmp_escap == 1)
                {
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);     
                }
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"escap failed");

            }
        }
    }
    int PathPlanningInterface::chargeFunction(int charger_x,int charger_y)
    {
        FRIZY_LOG(LOG_ERROR, "PathPlanningInterface::chargeFunction interface");
        int backtime = 10;
        int x, y;
        RobotType r = RobotType::circle; 
        chassis.GridPoint(&current_pos);
        x = current_pos.x;
        y = current_pos.y;
        std::vector<std::pair<float, float>> dwaPath;
        dwaPath = astar.astarLength(x, y, charger_x, charger_y, r, 1,0);
        // dwaPath.push_back({current_pos.x,current_pos.y});
        // dwaPath.push_back({-1.0*0.15,0.0});
        if(dwaPath.empty())
        {
            FRIZY_LOG(LOG_ERROR, "find the path to recharge seat failed");
            {
                arch.SearchWall(current_sensor,current_pos);
                // need to do alongwalk road 
                return -1;
            }
        }

        while (GLOBAL_CONTROL == WHEEL_RUN)
        {
            backtime = 10;
            escapescene(); 
            chassis.GetSensor(&current_sensor);
            if(dwaRun.start_path_planner(current_sensor, current_pos, dwaPath))
            {
                if(current_sensor.bump)
                {
                    chassis.chassisSpeed(0, 0, 1);
                    while(backtime&&GLOBAL_CONTROL == WHEEL_RUN)
                    {
                        backtime--;
                        chassis.chassisSpeed(-150, -150, 1);
                        usleep(50 * 1000);
                    }
                    chassis.chassisSpeed(0, 0, 1);
                    chassis.GridPoint(&current_pos);
                    x = current_pos.x;
                    y = current_pos.y;
                    // dwaPath.clear();
                    // dwaPath = astar.astarLength(x, y, 0, 0, r, 1);
                    if(dwaPath.empty())
                    {
                        FRIZY_LOG(LOG_ERROR, "find the path to recharge seat failed");
                        arch.SearchWall(current_sensor,current_pos);
                        // need to do alongwalk road 
                        
                        return -1;
                    }
                }
                else 
                {
                    chassis.GridPoint(&current_pos);
                    x = current_pos.x;
                    y = current_pos.y;
                    if(abs(x)== 0 && abs(y) == 0)
                    {
                        chassis.chassisSpeed(0, 0, 1);
                        FRIZY_LOG(LOG_INFO, "arrive the recharge seat");
                        return 0;
                    }
                }
            }
            else 
            {
                FRIZY_LOG(LOG_DEBUG, "recharge mode dwa failed");
                arch.SearchWall(current_sensor,current_pos);
                // need to do alongwalk road 
                return 1;
            }
            ControlPeriod(50 * 1000);
        }
    }

    void PathPlanningInterface::setChargeMode(RoadAim charge_aim)
    {
        FRIZY_LOG(LOG_DEBUG,"start to charge moving,the recharge seat = %d.%d\n",charge_aim.x,charge_aim.y);
        arch._aim.kind = recharge;
        
        arch.roadPlan.SetroadAim(charge_aim);
        process = ROAD;
        arch.UTURN = ARCH,
        arch.ARCH_STATE = NO_ARCH;
    }
    void PathPlanningInterface::rechargeTask()
    {
        chassis.getPlanningInfo(&current_planning_info);

        robot_current_state.planningState=SEARCHING_CHARGER;
        robot_current_clean_scene = RobotCleanScene::CHARGE;
        FRIZY_LOG(LOG_INFO, "START TO MOVE CHARGER");
        StopWallFollow();
        GLOBAL_CONTROL = WHEEL_RUN;
        RoadAim tmp_aim ;
        if(current_planning_info.charger_front_position.x == 0 && current_planning_info.charger_front_position.y == 0)
        {
            if(record_charge_aim.x!=0&&record_charge_aim.y!=0)
            {
                FRIZY_LOG(LOG_INFO, "THE RECORD RECHARGE IS %d ,%d",record_charge_aim.x,record_charge_aim.y);
                tmp_aim.x = record_charge_aim.x;
                tmp_aim.y = record_charge_aim.y;
            }
            else
            {
                FRIZY_LOG(LOG_INFO, "NO SLAM RECHARGE INFO AND NO RECORD RECHARGE INFO");
                tmp_aim.x = 0;
                tmp_aim.y = 0;
            }
        }
        else
        {
            tmp_aim.x =round_doubel((current_planning_info.charger_front_position.x * 100)/15);
            tmp_aim.y =round_doubel((current_planning_info.charger_front_position.y * 100)/15);                        
        }
        
        tmp_aim.kind = recharge;
        setChargeMode(tmp_aim);
    }
    void PathPlanningInterface::uploadArea()
    {
        current_clean_area.clean_area = _maze.mapArea;
        handler->sendRobotCleanArea(current_clean_area);
        FRIZY_LOG(LOG_DEBUG, "SEND CLEAN AREA DATA TO SERVICE : %d",current_clean_area.clean_area);         
    }
}

