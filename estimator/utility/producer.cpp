
#include <opencv2/opencv.hpp>
#include <json/json.h>
#include "utility/producer.h"

void RabbitMQ_producer::run()
{
    th = std::thread([this](){
        try
        {
            printf("[%s] Connecting to RabbitMQ...\n", name.c_str());
            amqp_connection_state_t connection = amqp_new_connection();
            amqp_socket_t* socket = amqp_tcp_socket_new(connection);
            if (!socket) throw producerException("Creating TCP socket");
            
            handle_response(amqp_socket_open(socket, hostname.c_str(), port), "Opening TCP socket");
            handle_amqp_response(amqp_login(connection, "/", 0, 131072, 0, AMQP_SASL_METHOD_PLAIN, username.c_str(), password.c_str()), "Logging in");
            
            amqp_channel_open(connection, 1);
            handle_amqp_response(amqp_get_rpc_reply(connection), "Opening channel");
            
            amqp_bytes_t amqp_queue_name = amqp_cstring_bytes(queue_name.c_str());
            amqp_queue_declare_ok_t* queue_status = amqp_queue_declare(connection, 1, amqp_queue_name, 0, 0, 0, 0, amqp_empty_table);
            handle_amqp_response(amqp_get_rpc_reply(connection), "Declaring queue");
            
            amqp_bytes_t amqp_exchange_name = amqp_cstring_bytes(exchange_name.c_str());
            amqp_bytes_t amqp_exchange_type = amqp_cstring_bytes(exchange_type.c_str());
            amqp_exchange_declare_ok_t* exchange_status = amqp_exchange_declare(connection, 1, amqp_exchange_name, amqp_exchange_type, 0, 0, 0, 0, amqp_empty_table);
            handle_amqp_response(amqp_get_rpc_reply(connection), "Declaring exchange");
            
            amqp_bytes_t amqp_routing_key = amqp_cstring_bytes(routing_key.c_str());
            amqp_queue_bind(connection, 1, amqp_queue_name, amqp_exchange_name, amqp_routing_key, amqp_empty_table);
            handle_amqp_response(amqp_get_rpc_reply(connection), "Binding queue");
            
            is_open = true;
            std::vector<uchar> msg;
            
            while (true)
            {
                {
                    std::unique_lock<std::mutex> lock(mx);
                    
                    while (msg_queue.size() == 0 && !do_exit)
                    {
                        waitrd.wait(lock);
                    }
                    
                    if (do_exit)
                    {
                        printf("[%s] Received exit flag\n", name.c_str());
                        break;
                    }
                    
                    msg = msg_queue.front();
                    msg_queue.pop();
                    
                    lock.unlock();
                }
                
                amqp_bytes_t message_bytes;
                message_bytes.len = msg.size();
                message_bytes.bytes = &(msg[0]);
                
                int rc = amqp_basic_publish(connection, 1, amqp_exchange_name, amqp_routing_key, 1, 0, NULL, message_bytes);
                handle_response(rc, "Sending Message");
            }
            
            is_open = false;
            printf("[%s] Closing down connections...\n", name.c_str());
            handle_amqp_response(amqp_channel_close(connection, 1, AMQP_REPLY_SUCCESS), "Closing channel");
            handle_amqp_response(amqp_connection_close(connection, AMQP_REPLY_SUCCESS), "Closing connection");
            handle_response(amqp_destroy_connection(connection), "Ending connection");
        }
        catch (const std::exception& x)
        {
            printf("[%s] A major exception has occurred! %s\n", name.c_str(), x.what());
            teptr = std::current_exception();
        }
        catch (...)
        {
            printf("[%s] A major exception has occurred!\n", name.c_str());
            teptr = std::current_exception();
        }
        printf("[%s] Finished\n", name.c_str());
    });
}

void RabbitMQ_producer::send(const std::string& msg)
{
    if (teptr) std::rethrow_exception(teptr);
    
    std::vector<uchar> binary_msg;
    std::copy(msg.begin(), msg.end(), back_inserter(binary_msg));
    printf("[%s] Queuing a binary message of size %lu : %s\n", name.c_str(), msg.size(), msg.c_str());
    
    {
        std::lock_guard<std::mutex> guard(mx);
        msg_queue.push(binary_msg);
    }
    
    waitrd.notify_one();
}

void RabbitMQ_producer::send(const Eigen::Matrix<double, 6, 1>& imu, double timestamp)
{
    if (teptr) std::rethrow_exception(teptr);
    
    Json::Value JSON_msg;
    JSON_msg["timestamp"] = timestamp;
    JSON_msg["acc_x"] = imu[0];
    JSON_msg["acc_y"] = imu[1];
    JSON_msg["acc_z"] = imu[2];
    JSON_msg["gyr_x"] = imu[3];
    JSON_msg["gyr_y"] = imu[4];
    JSON_msg["gyr_z"] = imu[5];
    std::string imuStrMsg = JSON_msg.toStyledString();
    
    std::vector<uchar> binary_msg;
    std::copy(imuStrMsg.begin(), imuStrMsg.end(), back_inserter(binary_msg));
    printf("[%s] Queuing a imu message: %f %f %f %f %f %f %f\n", name.c_str(),
           timestamp, imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
    
    {
        std::lock_guard<std::mutex> guard(mx);
        msg_queue.push(binary_msg);
    }
    
    waitrd.notify_one();
}

void RabbitMQ_producer::send(const std::pair<Vector6d, double>& imu)
{
    this->send(imu.first, imu.second);
}

void RabbitMQ_producer::send(const cv::Mat& img, std::string encoding_format, int compress)
{
    if (teptr) std::rethrow_exception(teptr);
    if (img.empty()) throw producerException("Error loading image!");
    
    std::vector<uchar> img_buffer;
    std::vector<int> image_params;
    image_params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
    image_params.emplace_back(compress);
    bool encodeStatus = cv::imencode(encoding_format, img, img_buffer, image_params);
    if (!encodeStatus) throw producerException("Error encoding image!");
    printf("[%s] Queuing an image of size: %dx%d\n", name.c_str(), img.cols, img.rows);
    
    {
        std::unique_lock<std::mutex> lock(mx);
        msg_queue.push(img_buffer);
    }
    
    waitrd.notify_one();
}

void RabbitMQ_producer::send(const cv::Mat& img, double timestamp, std::string encoding_format, int compress)
{
    if (teptr) std::rethrow_exception(teptr);
    if (img.empty()) throw producerException("Error loading image!");
    
    std::vector<uchar> img_buffer;
    std::vector<int> image_params;
    image_params.emplace_back(cv::IMWRITE_JPEG_QUALITY);
    image_params.emplace_back(compress);
    bool encodeStatus = cv::imencode(encoding_format, img, img_buffer, image_params);
    if (!encodeStatus) throw producerException("Error encoding image!");
    
    std::string strStamp = std::to_string(timestamp);
    while (strStamp.size() != 16)
    {
        if (strStamp.size() < 16)
            strStamp.push_back('0');
        else
            strStamp.pop_back();
    }
    img_buffer.insert(img_buffer.begin(), strStamp.begin(), strStamp.end());
    printf("[%s] Queuing an image of size: %dx%d timestamp: %f\n", name.c_str(), img.cols, img.rows, timestamp);
    
    {
        std::unique_lock<std::mutex> lock(mx);
        msg_queue.push(img_buffer);
    }
    
    waitrd.notify_one();
}

void RabbitMQ_producer::send(const std::pair<cv::Mat, double>& img, std::string encoding_format, int compress)
{
    this->send(img.first,img.second,encoding_format, compress);
}

void RabbitMQ_producer::send_EOT()
{
    if (teptr) std::rethrow_exception(teptr);
    printf("[%s] Queuing EOT\n", name.c_str());
    
    {
        std::unique_lock<std::mutex> lock(mx);
        std::vector<uchar> msg = { 4 };
        msg_queue.push(msg);
    }
    
    waitrd.notify_one();
}

std::thread& RabbitMQ_producer::exit()
{
    if (teptr) std::rethrow_exception(teptr);
    printf("[%s] Setting exit flag & notifying\n", name.c_str());
    
    {
        std::unique_lock<std::mutex> lock(mx);
        do_exit = true;
    }
    
    waitrd.notify_all();
    return th;
}

void RabbitMQ_producer::flush()
{
    if (teptr) std::rethrow_exception(teptr);
    printf("[%s] Flushing queue\n", name.c_str());
    
    bool queue_is_full = true;
    while (queue_is_full)
    {
        {
            std::unique_lock<std::mutex> lock(mx);
            if (msg_queue.empty())
                queue_is_full = false;
        }
        
        if (queue_is_full && teptr)
            std::rethrow_exception(teptr);
    }
}

std::thread& RabbitMQ_producer::flush_and_exit()
{
    this->flush();
    return exit();
}

void RabbitMQ_producer::handle_amqp_response(amqp_rpc_reply_t x, std::string context)
{
    std::string err;
    
    switch (x.reply_type)
    {
        case AMQP_RESPONSE_NORMAL:
            return;
        case AMQP_RESPONSE_NONE:
        {
            err = context + ": missing RPC reply type!";
            break;
        }
        case AMQP_RESPONSE_LIBRARY_EXCEPTION:
        {
            err = context + ": " + amqp_error_string2(x.library_error);
            break;
        }
        case AMQP_RESPONSE_SERVER_EXCEPTION:
        {
            switch (x.reply.id) {
                case AMQP_CONNECTION_CLOSE_METHOD:
                {
                    amqp_connection_close_t* m = (amqp_connection_close_t*) x.reply.decoded;
                    std::stringstream strm;
                    strm << context << ": server connection error " << m->reply_code << ", message: ";
                    for (int k = 0; k < m->reply_text.len; ++k)
                        strm << static_cast<unsigned char*>(m->reply_text.bytes)[k];
                    err = strm.str();
                    break;
                }
                case AMQP_CHANNEL_CLOSE_METHOD:
                {
                    amqp_channel_close_t* m = (amqp_channel_close_t*) x.reply.decoded;
                    std::stringstream strm;
                    strm << context << ": server channel error " << m->reply_code << ", message: ";
                    for (int k = 0; k < m->reply_text.len; ++k)
                        strm << static_cast<unsigned char*>(m->reply_text.bytes)[k];
                    err = strm.str();
                    break;
                }
                default:
                {
                    err = context + ": unknown server error, method id " + std::to_string(static_cast<int>(x.reply.id));
                    break;
                }
            }
            break;
        }
    }
    
    throw producerException(err.c_str());
}

void RabbitMQ_producer::handle_response(int rc, std::string context)
{
    if (rc < 0)
        throw producerException(context.c_str());
}

int producer(const std::string& type, const std::string& queue, const std::string& exchange, const std::string& ip, const std::string& port)
{
    int rc = EXIT_SUCCESS;
    
    std::string message_type    = type;
    std::string queue_name      = queue;
    std::string exchange_name   = exchange;
    std::string IP_ADDRESS      = ip;
    int PORT_ID = std::stoi(port);
    
    std::string name = "Message-Producer";
    RabbitMQ_producer prod(name, IP_ADDRESS, PORT_ID, queue_name, exchange_name, "direct", message_type);
    prod.credential("xinxin", "xhl6457398yy");
    
    try
    {
        prod.run();
        
        if (message_type.compare("image") == 0)
        {
            double timestamp = 1000000.00000;
            cv::Mat img = cv::imread("/Users/liuxianxian/Desktop/SLAM/Rabbit-Message/rabbitmq-logo.png");
            
            while (true)
                prod.send(img, timestamp, ".jpg");
        }
        else if (message_type.compare("imu") == 0)
        {
            Json::Value JSON_msg;
            JSON_msg["timestamp"] = 100000.0;
            JSON_msg["acc_x"] = 1.0;
            JSON_msg["acc_y"] = 2.0;
            JSON_msg["acc_z"] = 3.0;
            JSON_msg["gyr_x"] = 4.0;
            JSON_msg["gyr_y"] = 5.0;
            JSON_msg["gyr_z"] = 6.0;
            
            std::string strMsg = JSON_msg.toStyledString();
            
            while (true)
                prod.send(strMsg);
        }
        prod.send_EOT();
    }
    catch (const std::exception& x)
    {
        printf("[%s] Exception: %s\n", name.c_str(), x.what());
        rc = EXIT_FAILURE;
    }
    
    printf("[%s] Quitting\n", name.c_str());
    
    auto& th = prod.flush_and_exit();
    if (th.joinable()) th.join();
    return rc;
}
