/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-12-27 12:06:04
 * @Project      : UM_path_planning
 */


#pragma once
#ifndef ROADPLANNING_H
#define ROADPLANNING_H

#include "common_function/logger.h"
#include "navigation_algorithm/GlobalPlanning.h"
#include "common_function/MotionControl.h"
#include "navigation_algorithm/LocalPlanning.h"
#include "common_function/ExceptionHanding.h"
#include "navigation_algorithm/AlongWall.h"

using namespace std;

namespace useerobot
{
    extern int  recharge_first_index ;
    enum ROAD_KIND
    {
        idle = 0,
        searchUnclean,
        searchInside,
        searchBound,
        searchLast,
        archwallSign,
        backBound,
        recharge,
        searchWall,
        column,
        zoning,
        appoint,
        none,
    };
    enum ROAD_STATE
    {
        roadIdle,
        roadTrue,
        roadFalse,
    };


    struct RoadAim
    {
        int x;
        int y;
        enum ROAD_KIND kind;
    };


    using Points = std::vector<std::pair<int, int>>;
    class RoadPlanning
    {
    private:
        /* data */
        
        AlongWall _wall;
        chassisBase chassisRoad;
        MotionControl motionRoad;

        Trouble _trouble;
        int sum_recharge_times;
        
        
        //vector <pair<float, float>> astarArr;
        Points astarArr;
        void DwaRunning(Sensor sensor,Grid cur);
        void NullRunning(Sensor sensor,Grid cur);
        int ArriveRoad(Sensor sensor,Grid cur);
        
    public:
        RoadPlanning(/* args */);
        ~RoadPlanning();
        Grid ConAngle(Grid cur,Grid aim);
        int8_t BroadArea(int16_t x,int16_t y);
        
        Grid _aim;
        Grid ewall;
        int astarAgain;
        int ewallTime;
        int bumpCount;
        bool dewallSign;
        // Maze _maze;
        dwa roadDwa;
        aStar roadAstar;
        bool call_recharge_index = false;
        void init();
        void SetroadAim(RoadAim aim);
        void StartRoad(Sensor sensor,Grid cur,Trouble trouble);

    };
       
}
#endif

