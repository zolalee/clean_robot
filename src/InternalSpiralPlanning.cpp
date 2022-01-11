/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-12-24 11:48:32
 * @Project      : UM_path_planning
 */


#include "navigation_algorithm/InternalSpiralPlanning.h"
extern useerobot::Maze _maze;
namespace useerobot
{
    
    InternalSpiralPlanning::InternalSpiralPlanning()
    {
    }    

    InternalSpiralPlanning::~InternalSpiralPlanning()
    {
        
    }

    bool InternalSpiralPlanning::goPoint(Point &startPoint,Point &endPoint,RobotType &robotShape)
    {
        FRIZY_LOG(LOG_INFO, "start to go to the clean point in InternalSpiralPlanning::goPoint model ");
        auto astarMaze = Maze(_maze);
        astarMaze.startPoint = startPoint;
        astarMaze.endPoint =  endPoint;
        auto Astar = aStar();
        auto Motioncontrol = MotionControl();

        if(robotShape == RobotType::circle)
        {
            astarResult = Astar.findPath(astarMaze.startPoint, astarMaze.endPoint, robotShape);
            astarPath = Astar.getPath(astarResult);
            Motioncontrol.pathTransformtoOrder(astarPath); //路径到motioncontrol模块 是否需要在这里先执行？？？
            FRIZY_LOG(LOG_INFO, "successful to reach the  clean point in InternalSpiralPlanning::goPoint model ");
            return true;
        }
        else if(robotShape == RobotType::rectangle)
        {
            //need to do
        }

    }

    bool InternalSpiralPlanning::internalSpiralCleanPoint(Point &cleanPoint,RobotType &robotShape,int &layers)
    {
        FRIZY_LOG(LOG_INFO, "start to clean point in InternalSpiralPlanning::goPoint model ");

        if(robotShape == RobotType::circle)
        {


            InteralSpiralPath.push_back(&cleanPoint);
            tempPoint = cleanPoint;
            for(int a = 0;a <layers ; a++ )
            {
                for(int i =0;i<2*(a+1);i++) //右Y轴boundary
                {
                    tempPoint.x = tempPoint.x + cleaning_interval;
                    tempPoint.y =  tempPoint.y + cleaning_interval*i;
                    InteralSpiralPath.push_back(&tempPoint);
                }
                for(int i =0;i<2*(a+1);i++)//底X轴boundary
                {
                    tempPoint.x = tempPoint.x - cleaning_interval*(i+1);
                    tempPoint.y =  tempPoint.y;
                    InteralSpiralPath.push_back(&tempPoint);                    
                }
                for(int i =0;i<2*(a+1);i++)//左Y轴boundary
                {
                    tempPoint.x = tempPoint.x;
                    tempPoint.y =  tempPoint.y - cleaning_interval*(i+1);
                    InteralSpiralPath.push_back(&tempPoint);                       
                }
                for(int i =0;i<2*(a+1);i++)//顶X轴boundary
                {
                    tempPoint.x = tempPoint.x + cleaning_interval*(i+1);
                    tempPoint.y =  tempPoint.y;
                    InteralSpiralPath.push_back(&tempPoint);                     
                }                                                

            }

        }

        else if(robotShape == RobotType::rectangle)
        {
            //need to do
        }        

    }
    bool InternalSpiralPlanning::pointClean(RobotType &robotShape)
    {
        // FRIZY_LOG(LOG_DEBUG, "start to clean point turn mode");
        // _wheel_mode = turn;
        // leftspeed =0.2 rightspeed =0.4 等于绕0.3m半径旋转
        // _move_data.times = times;
        // _move_data.leftspeed = 0.2;
        // _move_data.rightspeed =0.4;
        _motioncontrol.robotchassiscontrol.chassisSpeed(30,200,1);
        pointCleanTime--;
        FRIZY_LOG(LOG_DEBUG, "pointCleanTime*20 = %d",pointCleanTime);
        
    }
    void InternalSpiralPlanning::randomClean()
    {
        while(1)
        {
            if(randomClean_start_index == true)
            {
        
            _motioncontrol.robotchassiscontrol.GetSensor(&current_sensor);
            if(current_sensor.bump ==0 && current_sensor.obs == 0 && current_sensor.magnVirWall == 0)
            // if(current_sensor.obs == 0)
            {
                FRIZY_LOG(LOG_DEBUG, "START TO WALK STRAIGHT IN RANDOM MODE");
                _motioncontrol.robotchassiscontrol.chassisSpeed(250,250,1);
            }
            else
            {
                if((current_sensor.leftw != 0||current_sensor.rightw != 0)&&current_sensor.obs == 1)
                {
                    _motioncontrol.robotchassiscontrol.chassisSpeed(0,0,1);
                    current_sensor.obs = 0;
                }
                if((current_sensor.leftw != 0||current_sensor.rightw != 0)&&current_sensor.magnVirWall == 1)
                {
                    _motioncontrol.robotchassiscontrol.chassisSpeed(0,0,1);
                    current_sensor.magnVirWall = 0;
                }                
                _motioncontrol.robotchassiscontrol.chassisSpeed(100,-100,1);
                int rotate_time = 1000+rand()%2000+1;
                FRIZY_LOG(LOG_DEBUG, "THE ROTATE TIME IS %d",rotate_time);
                usleep(1000*rotate_time);
            }
            usleep(50*1000);
            }
            else if(randomClean_start_index == false)
            {
                usleep(50*1000);
                continue;
            }
            
        }
    }
}

