/*
 * Copyright (C) 2021 Useerobot. All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-08-27 11:03:11
 * @LastEditTime : 2021-12-17 11:07:16
 * @Project      : UM_path_planning
 */
#ifndef NAVIAGTION_SERVER_HANDLER_H
#define NAVIAGTION_SERVER_HANDLER_H

#include <core/rpc_handler.h>
#include "navigation_msg.h"

typedef std::function<void(struct RobotCleanReq *)> robot_clean_cb;

class NavigationServerHandler : public core::RpcHandler
{
public:
    NavigationServerHandler();

    void registerRobotCleanCallback(robot_clean_cb cb);

    void sendRobotPlanningState(RobotPlanningStateData &cur_state);
    void sendRobotCleanArea(RobotPlanningAreaData &cur_Area);
    
    struct RobotPlanningStateData _planning_state;
protected:
    virtual bool handleRequest(struct core::header &header, std::vector<char> &payload);

private:
    robot_clean_cb callback;
};

#endif
