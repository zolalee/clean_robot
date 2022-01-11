/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-12-09 15:13:35
 * @Project      : UM_path_planning
 */


#pragma once

#ifndef UTURN_PLANNING_H
#define UTURN_PLANNING_H
#include "common_function/MotionControl.h"
#include "navigation_algorithm/GlobalPlanning.h"
#include "navigation_algorithm/BlockPlanning.h"
#include "navigation_algorithm/RoadPlanning.h"
#include "navigation_algorithm/AlongWall.h"

using namespace std;
namespace useerobot
{
    enum _ARCH
    {
        NO_ARCH = 0,
        RIGHT_0,
        RIGHT_PRE_180,      
        RIGHT_180,
        RIGHT_PRE_0,
        LEFT_0,
        LEFT_PRE_180,      
        LEFT_180,
        LEFT_PRE_0,    
    };  

    struct Rearch
    {
        enum _ARCH refor;
        int redis;
    };

    struct GIVEUP
    {
        int x;
        int y;
        int count;
        int close;
    };

    enum _UTURN
    {
        ARCH,
        SEARCH_UNCLEAN,
        ROAD_RUN,  
        ARCH_WALL, 
        BUMP_WALL,
        SEARCH_WALL,  
    };  

    enum SEARCH
    {
        IDLE_AREA,
        INSIDE_AREA,
        BOUND_AREA,   
    };   

    enum ARCHWALL
    {
        RIGHTWALL_0,
        RIGHTWALL_180,
        LEFTWALL_0,
        LEFTWALL_180,
    };      

    class UTurnPlanning
    {
    private:
        /* data */
        MotionControl _motioncontrol;
        GridPose currentpose;
        int topLeftEuclidean;
        int topRightEuclidean;
        int bottomLeftEuclidean;
        int bottomRightEuclidean;
        int minEuclidean;
        pointCoordinate nearestCorner;
        blockCorner reselectBlockCorner;
        float outLineIndex  = 1.0;
        int outLineIndexCeil = 1 ;
        cornerIdex _cornerIdex ;
        wheel_mode _wheel_mode ;
        move_data  _move_data ;
        float rotato_yawl; 
        int temp_distance;
        Point _currentPoint;// 是否使用自己记录的？还是获取当前slam模块的定位信息？
        Point _endPoint;
        int cleaning_interval = int(robot_radius/resolution);
        bool first_planning_index =false ;
        int robot_orientation; // 1-> 向下；-1->向上
        bool turnIndex = false;
        
        //add
        AlongWall _wall;

        blockPlanning blockPlan;

        chassisBase chassisPlan;
      
        MotionControl controlw;


        boundary archBound;
        vector <boundary> archBoundArr;
       
        
        Rearch rearch;

        
        
        Trouble _trouble;
        aStar astarPlan;
        void ArchWall(Sensor abc,Grid cur);
        void BumpWall(Sensor sensor,Grid cur);
        void Arch(Sensor abc,Grid cur);
        bool StopArch(Sensor sensor,Grid cur);
        
        int JudgeGiveup(Grid cur,ROAD_KIND kind);
        int SameLine(Grid cur,Grid aim,int state);
        int ClearOB(Grid cur);
        int8_t Virob(int x,int y);
        void Kmeans(int x,int y,int kinds);

    public:
        UTurnPlanning(/* args */);
        ~UTurnPlanning();
        bool CleanRecharge();
        
        int Dsearch(int x,int y);
        Rearch UporDown(Grid cur,Grid aim,int othersign);
        void init();
        int CheckFor(Grid cur,DIVB forward);
        void SearchUnlean(Sensor abc,Grid cur);

        void InsidePlanning(Sensor sensor,Grid cur,Trouble trouble);

        void PlanSearch(RoadAim aim,int flag);
        void SearchWall(Sensor sensor,Grid cur);
        // Maze _maze;
        
        _UTURN UTURN;
        _ARCH ARCH_STATE;
        Grid aim;
        Grid bumpwall;
        vector<Grid> CloseArr;
        int keepwallTime;
        /**
         * @description: 查找路径
         * @event: 
         * @param {Point} &startPoint
         * @param {blockCorner} &selectBlockCorner
         * @param {RobotType} &robotShape
         * @return {*}
         */        
        void uTurnMovePath(Point &startPoint, blockCorner &selectBlockCorner,RobotType &robotShape);
        /**
         * @description: 用于找定义大小区块的弓子型清扫模式
         * @event: 
         * @param {Point} &startPoint
         * @param {int} &length
         * @param {int} &height
         * @param {RobotType} &robotShape
         * @return {*}
         */        
        void uTurnMovePath(Point &startPoint,int &length,int &height,RobotType &robotShape);
        /**
         * @description: 执行路径 输入 startPoint 是 selectBlockCorner 中的一个
         * @event: 
         * @param {Point} *result
         * @return {*}
         */        
        list<Point*> uTurnGetPath();
        /**
         * @description: 获取最近的角落
         * @event: 
         * @param {Point} &startPoint
         * @param {blockCorner} &selectBlockCorner
         * @return {*}
         */        
        pointCoordinate getNearestCoordinate(Point &startPoint, blockCorner &selectBlockCorner);
        /**
         * @description: 运动到角点
         * @event: 
         * @param {Point} &startPoint
         * @param {blockCorner} &selectBlockCorner
         * @param {RobotType} &robotShape
         * @return {*}
         */        
        bool goCorner(Point &startPoint, blockCorner &selectBlockCorner,RobotType &robotShape);
        /**
         * @description: 
         * @event: 
         * @param {float} &rotatoyawl
         * @return {*}
         */        
        bool turnRoundCorner(float &rotatoyawl);
        /**
         * @description: 
         * @event: 
         * @param {Point} &startPoint
         * @param {blockCorner} &selectBlockCorner
         * @return {*}
         */        
        void moveTrajectory(Point &startPoint,blockCorner &selectBlockCorner,cornerIdex &_cornerIdex,RobotType &robotShape);
        /**
         * @description: 
         * @event: 
         * @param {Point} &currentPoint
         * @param {Point} &targetPoint
         * @return {*}
         */        
        bool uTurnPointIsCanReach(Point* &currentPoint,Point* &targetPoint);
        /**
         * @description: 以Y轴直线导航 用于覆盖清扫功能模块
         * @event: 
         * @param {Point} &startPoint
         * @param {Point} &endPoint
         * @return {*}
         */        
        void uTurnLinePathControl(Point &startPoint,Point &endPoint);
        /**
         * @description: 
         * @event: 
         * @param {Point} &startPoint
         * @param {cornerIdex} &_cornerIdex
         * @return {*}
         */        
        bool turnMove(Point &startPoint,cornerIdex &_cornerIdex);
        Maze * maze = NULL;
        Point * astarResult = NULL;
        list<Point*> astarPath;
        list<Point*> uTurnPath;
        list<Point*> uTurnPoint;
        RoadAim _aim;
        RoadPlanning roadPlan; 
        Planning_info charger_planning_info; 
        
        
    };
    

    
}
#endif
