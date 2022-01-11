/*
 * Copyright (C) 2021 Useerobot. All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-08-27 11:02:25
 * @LastEditTime : 2021-12-17 11:07:49
 * @Project      : UM_path_planning
 */
#include <unistd.h>

#include <core/log.h>
#include <message/navigation_msg.h>
#include <message/navigation_server_handler.h>

using namespace core;

#define TAG ("navigation_rpc")


NavigationServerHandler::NavigationServerHandler()
{
    callback = nullptr;
}

void NavigationServerHandler::registerRobotCleanCallback(robot_clean_cb cb)
{
    callback = cb;
}

void NavigationServerHandler::sendRobotPlanningState(RobotPlanningStateData &cur_state)
{
    int rc;
    struct header header_res;
    struct RobotPlanningStateData state_data;
    state_data.planningState = cur_state.planningState;
    // header_res.seq = kHeaderResponse;
    header_res.type = kHeaderRequest;
    header_res.data_type = kNavigationMsgRobotPlanningStateData;
    header_res.data_len = 0;
    log_info(TAG, "sendRobotPlanningState =%d", state_data.planningState);
    uint32_t seq = sendRequest(header_res, state_data);
    
}
void NavigationServerHandler::sendRobotCleanArea(RobotPlanningAreaData &cur_Area)
{
    int rc;
    struct header header_res;
    struct RobotPlanningAreaData Area_data;
    Area_data.clean_area = cur_Area.clean_area;
    // header_res.seq = kHeaderResponse;
    header_res.type = kHeaderRequest;
    header_res.data_type = kNavigationMsgRobotCleanArea;
    header_res.data_len = 0;
    // log_info(TAG, "sendRobotCLeanArea =%d", Area_data.clean_area);
    uint32_t seq = sendRequest(header_res, Area_data);
    
}

bool NavigationServerHandler::handleRequest(struct header &header, std::vector<char> &payload)
{
    int rc;
    struct header header_res;
    struct RobotCleanReq clean_req;

    if (header.data_type == kNavigationMsgRobotCleanData) {
        msgpack::object_handle oh = msgpack::unpack(payload.data(), payload.size());
        msgpack::object deserialized = oh.get();

        deserialized.convert(clean_req.data);

        clean_req.header = header;
        // pass data to callback


        header_res.seq = header.seq;
        header_res.type = kHeaderResponse;
        header_res.data_type = header.data_type;
        header_res.seq = header.seq;
        header_res.data_len = 0;
        sendResponse(header_res);
        if (callback != nullptr) {
            callback(&clean_req);
        }
    } else if (header.data_type == kNavigationMsgRobotPlanningStateData) {
        struct RobotPlanningStateData planning_state;
        // planning_state.planningState = PLANNING;
        planning_state.planningState = _planning_state.planningState;
        log_info(TAG, "call getPlanningState success state=%d", planning_state.planningState);
        header_res.seq = header.seq;
        header_res.type = kHeaderResponse;
        header_res.data_type = header.data_type;
        header_res.seq = header.seq;
        header_res.data_len = 0;
        sendResponse(header_res, planning_state);
    } else {
        log_error(TAG, "bad request data type %d", header.data_type);
        return false;
    }

    return true;
}

