#ifndef SKELETION_THREAD_H
#define SKELETION_THREAD_H

#include <string>
#include <thread>
#include <memory>

namespace core {

class SkeletonThread
{
public:
    SkeletonThread();
    ~SkeletonThread();

    // void setThreadName(std::string name);

    void start();

    void stop();

    bool isRunning();

protected:
    virtual void onStart();
    virtual void onStop();

    virtual bool threadLoop();

private:
    void threadFunction();

protected:
    std::shared_ptr<std::thread> thread_;
    bool running_;
    std::string thread_name_;
    std::string name_;
};

}

#endif
