/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-11-16 09:58:46
 * @Project      : UM_path_planning
 */

/*
回充
*/
#pragma once
#ifndef RECHARGE_H
#define RECHARGE_H

#include "common_function/MotionControl.h"
#include "navigation_algorithm/GlobalPlanning.h"
using namespace std;
enum chargeSeatOrientation
{
    east = 0, 
    west = 1,    
    north = 2, 
    south = 3,
};
namespace useerobot
{
    extern CHASSIS_CONTROL GLOBAL_CONTROL;
    class Recharge
    {
    private:
        /* data */
        MotionControl _motioncontrol;
        Point rechagePosition;
        int cleaning_interval = int(robot_radius/resolution);
        Point * astarResult = NULL;
        list<Point*> astarPath;
        Grid current_pos;
        aStar astar;
    public:
        Recharge(/* args */);
        ~Recharge();
        list<Point*> searchToRechargeSeat(Point &startPoint,Point &seatPoint,RobotType &robotType, chargeSeatOrientation &setOrientation);
        void chassisRecharge();
        Point getRechagePosition(Point &seatPoint,RobotType &robotType,chargeSeatOrientation &setOrientation);
        void chargeRoad(); //回充行走
    };
        
}
#endif
