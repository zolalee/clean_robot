#ifndef SKELETON_THREAD_H
#define SKELETON_THREAD_H

#include <map>
#include <mutex>
#include <string>
#include <sstream>
#include <memory>
#include <thread>
#include <vector>
#include <future>
#include <chrono>
#include <stdint.h>

#include <msgpack.hpp>
#include <core/queue.h>
#include <core/header.h>


namespace core {

class RpcHandler
{
public:
    class FutureObject {
    public:
        bool success;
        struct core::header header;
        std::vector<char> packed_data;
    };

    class RpcSendContext {
    public:
        uint32_t seq;
        bool send;
        bool recv;
        struct core::header header;
        std::vector<char> packed_data;
        bool has_promise;
        std::shared_ptr<std::promise<std::shared_ptr<FutureObject>>> promise;
    };

    class PromiseContext {
    public:
        int seq;
        std::shared_ptr<std::promise<std::shared_ptr<FutureObject>>> promise;
    };

public:
    RpcHandler() {
        fd = -1;
        seq = 1;
        send_running_ = false;
        recv_running_ = false;
        send_thread_ = nullptr;
        recv_thread_ = nullptr;
    }

    void setSocketFd(int fd) {
        this->fd = fd;
    }

    void start();

    void stop();

    // void restart();

    bool isSendAlive();

    bool isRecvAlive();

    bool isTransceiverAlive();

    bool waitTranseiverStart(int timeout = 2000);

    // void stopTranseiver(int timeout = 2000);

protected:
    virtual bool threadSendLoop();
    virtual bool threadRecvLoop();

    uint32_t generateSeq();

    void submitToSendQueue(std::shared_ptr<RpcSendContext> request);

    uint32_t sendRequestWithPayload(struct header &header, std::vector<char> &payload);
    uint32_t sendRequest(struct header &header);
    bool waitResponseWithPayload(uint32_t seq, struct header &header, std::vector<char> &payload, int timeout = 100);
    bool waitResponse(uint32_t seq, struct header &header, int timeout = 100);

    bool sendResponseWithPayload(struct header &header, std::vector<char> &payload);
    bool sendResponse(struct header &header);    

    virtual bool handleRequest(struct header &header, std::vector<char> &payload) = 0;

    template<typename T>
    uint32_t sendRequest(struct header &header, T &t) {
        std::stringstream buffer;
        msgpack::pack(buffer, t);
        buffer.seekg(0);
        std::string str(std::move(buffer.str()));
        std::vector<char> payload(str.begin(), str.end()); // fixme: rvalue later

        header.data_len = payload.size();

        return sendRequestWithPayload(header, payload);
    }

    template<typename T>
    bool sendResponse(struct header &header, T &t) {
        std::stringstream buffer;
        msgpack::pack(buffer, t);
        buffer.seekg(0);
        std::string str(std::move(buffer.str()));
        std::vector<char> payload(str.begin(), str.end()); // fixme: rvalue later

        header.data_len = payload.size();

        return sendResponseWithPayload(header, payload);
    }

    template<typename T>
    bool waitResponse(uint32_t seq, struct header &header, T &t, int timeout = 100)
    {
        bool success;
        std::vector<char> payload;

        success = waitResponseWithPayload(seq, header, payload, timeout);
        if (success == false) {
            return false;
        }

        msgpack::object_handle oh = msgpack::unpack(payload.data(), payload.size());
        msgpack::object deserialized = oh.get();

        deserialized.convert(t);

        return true;
    }

private:
    void threadSendFunction();
    void threadRecvFunction();

protected:
    int fd;
    std::shared_ptr<std::thread> send_thread_;
    std::shared_ptr<std::thread> recv_thread_;
    volatile bool send_running_;
    volatile bool recv_running_;
    uint32_t seq;
    std::mutex seq_mutex;
    core::Queue<std::shared_ptr<RpcSendContext>> send_queue;
    std::condition_variable event_condition_;
    std::mutex event_mutex_;
    std::map<uint32_t, std::shared_ptr<PromiseContext>> promise_map;
    std::mutex promise_mutex_;
    std::mutex stop_mutex_;
};

}

#endif
