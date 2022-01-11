#ifndef NAVIGATION_RPC_CLIENT_H
#define NAVIGATION_RPC_CLIENT_H

#include <memory>
#include <core/rpc_client.h>
#include <core/rpc_handler.h>

class NavigationRpcClient : public core::RpcClient
{
public:
    NavigationRpcClient();

    void setRpcHandler(std::shared_ptr<core::RpcHandler> handler);

protected:
    virtual void handleNewConnection(int fd);

private:
    std::shared_ptr<core::RpcHandler> handler;
};

#endif
