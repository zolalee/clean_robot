/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-11-16 09:58:21
 * @Project      : UM_path_planning
 */

#include "common_function/Recharge.h"

extern useerobot::Maze _maze;
namespace useerobot
{
    
    Recharge::Recharge(/* args */)
    {
    }   
    Recharge::~Recharge()
    {
    }
    list<Point*> Recharge::searchToRechargeSeat(Point &startPoint,Point &seatPoint,RobotType &robotType,chargeSeatOrientation &setOrientation)
    {
        FRIZY_LOG(LOG_INFO, "start searchToRechargeSeat");
        rechagePosition = getRechagePosition(seatPoint,robotType,setOrientation);

        auto Astar = aStar();

        // 点到点规划
        astarResult = Astar.findPath(startPoint, rechagePosition, robotType);
        astarPath = Astar.getPath(astarResult);
        return astarPath;      
    }
    void Recharge::chassisRecharge()
    {
        FRIZY_LOG(LOG_INFO, "start to chassisRecharge");
        if(_motioncontrol.chassisRecharge() == true)
        FRIZY_LOG(LOG_INFO, "succssful excute chassisRecharge ");
        else
        FRIZY_LOG(LOG_ERROR, "fail to done chassisRecharge ");

    }
    // 怎么确定充电座是在X轴还是Y轴方向？是否需要红外线数据才能确定？
    Point Recharge::getRechagePosition(Point &seatPoint,RobotType &robotType,chargeSeatOrientation &setOrientation)
    {
        // 如果是X轴靠边,充电口朝下
        //need to do more
        if(robotType == RobotType::circle)
        {
        switch (setOrientation)
        {
        case 0:
            rechagePosition = seatPoint;
            rechagePosition.x = rechagePosition.y+ cleaning_interval*2;
            return rechagePosition;
            break;
        case 1:
            rechagePosition = seatPoint;
            rechagePosition.x = rechagePosition.y- cleaning_interval*2;
            return rechagePosition;
            break;
        case 2:
            rechagePosition = seatPoint;
            rechagePosition.y = rechagePosition.y- cleaning_interval*2;
            return rechagePosition;
            break;
        case 3:
            rechagePosition = seatPoint;
            rechagePosition.y = rechagePosition.y+ cleaning_interval*2;
            return rechagePosition;
            break;                                
        default:
            break;
        }

        }
        if(robotType == RobotType::rectangle)
        {
            
        }
    }
}