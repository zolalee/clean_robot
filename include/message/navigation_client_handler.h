#ifndef NAVIAGTION_CLIENT_HANDLER_H
#define NAVIAGTION_CLIENT_HANDLER_H

#include <map>
#include <vector>
#include <future>
#include <memory>
#include <condition_variable>

#include <core/queue.h>
#include <core/rpc_handler.h>
#include "navigation_msg.h"


class NavigationClientHandler : public core::RpcHandler
{
public:
    NavigationClientHandler();

    // bool wholeTraverseRooms();
    // bool wholeTraverseBlocks();
    bool slamPlanning();
    bool traverseSelectedRooms();
    bool gotoCleanPoinit();
    bool gotoBlockClean();
    bool charge();
    bool alongWall();
    bool relocalization();
    bool stop();
    bool pause();
    bool resume();

    bool getPlanningState(RobotPlanningStateData &planning_state);

protected:
    virtual bool handleRequest(struct core::header &header, std::vector<char> &payload);
};

#endif
