
#ifndef consumer_h
#define consumer_h

#include <iostream>
#include <exception>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include <time.h>
#include <fstream>

#include <rabbitmq-c/amqp.h>
#include <amqp_tcp_socket.h>

static std::exception_ptr teptr = nullptr;

class consumerException : public std::exception
{
public:
    consumerException() : error("") {}
    consumerException(std::string error)
    {
        this->error = error;
    }
    virtual const char* what() const _NOEXCEPT
    {
        return error.c_str();
    }
private:
    std::string error;
};

class RabbitMQ_consumer
{
public:

    RabbitMQ_consumer(std::string producer_name, std::string hostname = "localhost", int port = 5672, std::string queue_name = "my_queue", std::string exchange_name = "my_exchange", std::string exchange_type = "direct", std::string routing_key = "my_key")
    {
        this->name = producer_name;
        this->hostname = hostname;
        this->port = port;
        this->queue_name = queue_name;
        this->exchange_name = exchange_name;
        this->exchange_type = exchange_type;
        this->routing_key = routing_key;
    }

    void credential(std::string username, std::string password)
    {
        this->username = username;
        this->password = password;
    }

    void run();
    
    void receive(std::string& msg);

    void receive(cv::Mat& img);

    void receive(cv::Mat& img, double& timestamp);

    std::thread& exit();

private:

    void handle_amqp_response(amqp_rpc_reply_t x, std::string context);
    
    void handle_response(int rc, std::string context);
    
    void receive(std::vector<uchar>& msg);

    int port;
    std::string name;
    std::string hostname;
    std::string username;
    std::string password;
    std::string queue_name;
    std::string exchange_name;
    std::string exchange_type;
    std::string routing_key;

    std::mutex mx;
    std::thread th;
    std::condition_variable waitrd;
    std::queue<std::vector<uchar>> msg_queue;
    bool do_exit = false;
};

inline int consumer(const std::string&, const std::string&, const std::string&, const std::string&, const std::string&);
#endif /* consumer_h */
