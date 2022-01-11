/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-10-25 14:18:03
 * @Project      : UM_path_planning
 */



#include "common_function/logger.h"
#include "PathPlanningInterface.h"
#include "lcm/lcm-cpp.hpp"
#include <fstream>
#include <vector>
#include <iostream>
#include <time.h>
#include "common_function/Parse.h"
// #include <boost/bind.hpp>
// #include <boost/thread.hpp>

using namespace std;
using namespace useerobot;

Maze planningMaze;
Parse robot_parse;
PathPlanningInterface robotscene;

int main(int argc, char** argv)
{
    // log_init("srevice", NULL);
    // boost::shared_ptr<useerobot::PathPLanningInterface> planning(new useerobot::PathPLanningInterface());
    auto parseconfig = Parse();
    parseconfig.getParseConfig();
    cout << "clean_robot_type:" << parseconfig.config["clean_robot_type"].as<string>()<< endl;
    robotscene.autodividerooms = parseconfig.config["autodividerooms"].as<bool>();
    if(parseconfig.config["clean_robot_type"].as<string>() == "circle")
    robotscene.cur_RobotType = RobotType::circle;
    if(parseconfig.config["clean_robot_type"].as<string>() == "rectangle")
    robotscene.cur_RobotType = RobotType::rectangle;
    FRIZY_LOG(LOG_INFO, "robotscene.cur_RobotType = %d \n",robotscene.cur_RobotType);
    
    FRIZY_LOG(LOG_INFO, "start to enter the planning module ");
    corner_index = _topLeftCorner;

    robotscene.cleanCycleInit();
    // UMAPI_ChassisCallbackRegister(_SensorDataCallback,_ImuDataCallback);
    // // if(atoi(argv[1])== 1) {
    // //     UMAPI_CtrlWifiState(robot_sta_automatic,wifi_sta_islink,ir_ctrl_none,fan_gear_none,water_gear_none);

    // // }
    // // if(atoi(argv[1])== 2) {

    // //     UMAPI_CtrlWifiState(robot_sta_fixed_point,wifi_sta_islink,ir_ctrl_none,fan_gear_none,water_gear_none);
    // // }
    // // if(atoi(argv[1])== 3) {

    // //     UMAPI_CtrlWifiState(robot_sta_along_wall,wifi_sta_islink,ir_ctrl_none,fan_gear_none,water_gear_none);
    // // }
    // // if(atoi(argv[1])== 4) {

    // //     UMAPI_CtrlWifiState(robot_sta_random,wifi_sta_islink,ir_ctrl_none,fan_gear_none,water_gear_none);
    // // }
    
    // UMAPI_CtrlWifiState(robot_sta_automatic,wifi_sta_islink,ir_ctrl_none,fan_gear_low,water_gear_none);
   
    // UMAPI_MainWheelSpeed(1,80,20);
    FRIZY_LOG(LOG_INFO, "UMAPI_MainWheelSpeed excute");
    
    return 0;
}
