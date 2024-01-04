#include <iostream>
#include <exception>
#include <string>
#include <sstream>
#include <thread>
#include <queue>

#include <rabbitmq-c/amqp.h>
#include <amqp_tcp_socket.h>

#include <Eigen/Core>

static std::exception_ptr teptr = nullptr;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class producerException : public std::exception
{
public:
	producerException() : error("") {}
	producerException(std::string error)
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

class RabbitMQ_producer
{
public:
    RabbitMQ_producer()
    {
        this->name = "RabbitMQ_Producer";
        this->port = 5672;
        this->queue_name = "RabbitMQ_Queue";
        this->exchange_name = "RabbitMQ_Exchange";
        this->exchange_type = "direct";
        this->routing_key = "RabbitMQ";
    }
    
    /// RabbitMQ生产者构造
    /// \param producer_name
    /// \param hostname
    /// \param port
    /// \param queue_name
    /// \param exchange_name
    /// \param exchange_type
    /// \param routing_key
	RabbitMQ_producer(std::string producer_name, std::string hostname = "localhost", int port = 5672, std::string queue_name = "my_queue", std::string exchange_name = "my_exchange", std::string exchange_type = "direct", std::string routing_key = "my_key")
	{
		this->name = producer_name;
		this->port = port;
		this->queue_name = queue_name;
		this->exchange_name = exchange_name;
		this->exchange_type = exchange_type;
		this->routing_key = routing_key;
	}
    
    void setProducerName(const std::string name)
    {
        this->name = name;
    }
    
    void setQueue(const std::string queue, std::string routKey)
    {
        this->queue_name = queue;
        this->routing_key = routKey;
    }
    
    void setExchange(const std::string exchange, std::string property)
    {
        this->exchange_name = exchange;
        this->exchange_type = property;
    }
    
    void setHostName(const std::string& hostname, const std::string port)
    {
        this->hostname = hostname;
        this->port = std::stoi(port);
    }
    
	void credential(const std::string& username, const std::string& password)
	{
        this->username = username;
        this->password = password;
	}
    
    bool isOpen()
    {
        return is_open;
    }

    void run();

    void send(const std::string& msg);
    
    // 发送惯导数据
    void send(const Vector6d& imu, double timestamp);
    void send(const std::pair<Vector6d, double>& imu);
    
    // 发送图像数据
    void send(const cv::Mat& img, std::string encoding_format = ".png", int compress = 100);
    void send(const cv::Mat& img, double timestamp, std::string encoding_format = ".png", int compress = 100);
    void send(const std::pair<cv::Mat, double>& img, std::string encoding_format = ".png", int compress = 100);

    void send_EOT();

    void flush();
    
    std::thread& exit();

    std::thread& flush_and_exit();

private:

    void handle_amqp_response(amqp_rpc_reply_t x, std::string context);

    void handle_response(int rc, std::string context);
    
	int port;
	std::string name;
	std::string username;
	std::string password;
	std::string hostname;
	std::string queue_name;
	std::string exchange_name;
    std::string exchange_type;
    std::string routing_key;

	std::thread th;
    std::queue<std::vector<uchar>> msg_queue;
	std::mutex mx;
	std::condition_variable waitrd;
    
	bool do_exit = false;
    bool is_open = false;
};

inline int producer(const std::string&,const std::string&,const std::string&,const std::string&,const std::string&);
