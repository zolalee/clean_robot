#ifndef RPC_SERVER_H
#define RPC_SERVER_H
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string>

#include <core/skeleton_thread.h>

namespace core {

class RpcServer : public SkeletonThread
{
public:
    RpcServer();

    void setServerAddress(std::string server_address);

    bool init();

    void exit();

protected:
    virtual bool threadLoop();

    virtual void handleNewConnection(int fd);

protected:
    std::string server_address;
    int server_socket;
    struct sockaddr_un server_sockaddr;
};

}

#endif
