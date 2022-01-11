/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-10-20 15:19:54
 * @Project      : UM_path_planning
 */


#pragma once

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

//#include "um_chassis/um_chassis_user.h"
#include "um_chassis/chassisBase.h"
#include "common_function/MapFunction.h"
#include <thread>
using namespace std;
namespace useerobot
{
const float straight_speed = 0.4;

typedef struct
{
    int left;
    int right;
}wheel_speed;

typedef struct
{
    float acc1;
    float acc2;
    float acc3;
    float acc;
    float lastacc1;
}wheel_pid;

enum wheel_mode
{
    forward = 0,
    back = 1,
    left = 2,
    right = 3,
    turn = 4,
    stop = 5,
};
typedef struct 
{
    float speed;
    float leftspeed;
    float rightspeed;
    int16_t times;
    int16_t rotateangle; //左正右负
    GridPose movepose;
}move_data;


class MotionControl
    {
    public:
        MotionControl(/* args */);
        ~MotionControl();
        void chassisMotionInit();
        void WheelControl(Sensor sensor,Grid cur,Grid aim);
        void WheelBack();
        void WheelPid(wheel_speed* Speed,Grid cur,Grid aim);
        void ClearPid();
        wheel_speed wheelSpeed;
        
        
        Sensor mode_sensor;
        
        bool wheelControl(wheel_mode &mode,move_data &data);
        /**
         * @description: 
         * @event: 
         * @param {*}
         * @return {*}
         */        
        bool chassisRecharge();
        /**
         * @description: 
         * @event: 
         * @param {*}
         * @return {*}
         */        
        bool chassisWallAlong(uint8_t mode,int direction);
        // void pathTransformtoOrder(vector <cellIndex> &robotPath);
        /**
         * @description: 将路径转化为执行指令
         * @event: 
         * @param {list<Point *>} &path
         * @return {*}
         */        
        bool pathTransformtoOrder(list<Point *> &path);
        move_data robotData;
        useerobot::chassisBase robotchassiscontrol;
    private:
        /* data */
        // ControlData_t robotControldata;
        ControlData_t robotControldata;
        wheel_mode robotcontrolMode;        

        useerobot::MapFunction *robotMap;
        GridPose currentpose;
        GridPose targetpose;

        


    };    
}
#endif