
#include <message/navigation_rpc_server.h>

using namespace core;

NavigationRpcServer::NavigationRpcServer()
{
    handler = nullptr;
}

void NavigationRpcServer::setRpcHandler(std::shared_ptr<RpcHandler> handler)
{
    this->handler = handler;
}

void NavigationRpcServer::handleNewConnection(int fd)
{
    if (handler != nullptr) {
        handler->setSocketFd(fd);
    }
}
