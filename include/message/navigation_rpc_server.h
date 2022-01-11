#ifndef NAVIGATION_RPC_SERVER_H
#define NAVIGATION_RPC_SERVER_H

#include <memory>

#include <core/skeleton_thread.h>
#include <core/rpc_server.h>
#include <core/rpc_handler.h>


class NavigationRpcServer : public core::RpcServer
{
public:
    NavigationRpcServer();

    void setRpcHandler(std::shared_ptr<core::RpcHandler> handler);

protected:
    virtual void handleNewConnection(int fd);

private:
    std::shared_ptr<core::RpcHandler> handler;
};

#endif
