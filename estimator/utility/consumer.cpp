
#include <opencv2/opencv.hpp>
#include <json/json.h>
#include "utility/consumer.h"

void RabbitMQ_consumer::run()
{
    th = std::thread([this]() {
        try
        {
            printf("[%s] Connecting to RabbitMQ...\n", name.c_str());
            amqp_connection_state_t connection = amqp_new_connection();
            amqp_socket_t* socket = amqp_tcp_socket_new(connection);
            if (!socket) throw consumerException("Creating TCP socket");

            handle_response(amqp_socket_open(socket, hostname.c_str(), port), "Opening TCP socket");
            handle_amqp_response(amqp_login(connection, "/", 0, 131072, 0, AMQP_SASL_METHOD_PLAIN, username.c_str(), password.c_str()), "Logging in");

            amqp_channel_open(connection, 1);
            handle_amqp_response(amqp_get_rpc_reply(connection), "Opening channel");

            amqp_bytes_t amqp_queue_name = amqp_cstring_bytes(queue_name.c_str());
            //amqp_queue_declare_ok_t* r = amqp_queue_declare(connection, 1, amqp_queue_name, 0, 0, 0, 0, amqp_empty_table);
            //handle_amqp_response(amqp_get_rpc_reply(connection), "Declaring queue");

            amqp_basic_consume(connection, 1, amqp_queue_name, amqp_empty_bytes, 0, 1, 0, amqp_empty_table);
            handle_amqp_response(amqp_get_rpc_reply(connection), "Consuming");

            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            printf("[%s] Waiting for messages...\n", name.c_str());
            
            while (true)
            {
                amqp_maybe_release_buffers(connection);
                amqp_envelope_t envelope;
                amqp_rpc_reply_t res = amqp_consume_message(connection, &envelope, &timeout, 0);

                if (res.reply_type != AMQP_RESPONSE_NORMAL &&
                    !(res.reply_type == AMQP_RESPONSE_LIBRARY_EXCEPTION && res.library_error == AMQP_STATUS_TIMEOUT))
                    handle_amqp_response(res, "Consuming messages");

                {
                    std::lock_guard<std::mutex> guard(mx);

                    if (do_exit)
                    {
                        printf("[%s] Received exit flag\n", name.c_str());
                        break;
                    }
                }
    
                if (res.reply_type == AMQP_RESPONSE_NORMAL)
                {
                    if (0)
                    {
                        std::cout << " [" << name.c_str() << "] Received message #" << static_cast<unsigned>(envelope.delivery_tag)
                            << " of length " << envelope.message.body.len;
                        if (envelope.message.properties._flags & AMQP_BASIC_CONTENT_TYPE_FLAG)
                        {
                            std::cout << " Content-type: ";
                            for (size_t k = 0; k < envelope.message.properties.content_type.len; ++k)
                                std::cout << (static_cast<uchar*>(envelope.message.properties.content_type.bytes))[k];
                        }
                        std::cout << std::endl;
                    }

                    std::vector<uchar> msg;
                    auto lens = envelope.message.body.len;
                    auto bytes = envelope.message.body.bytes;
                    msg.insert(msg.end(), static_cast<uchar*>(bytes), static_cast<uchar*>(bytes) + lens);

                    amqp_destroy_envelope(&envelope);

                    {
                        std::lock_guard<std::mutex> guard(mx);
                        msg_queue.push(msg);
                    }

                    waitrd.notify_one();
                }
            }

            printf("[%s] Closing down connections...\n", name.c_str());
            handle_amqp_response(amqp_channel_close(connection, 1, AMQP_REPLY_SUCCESS), "Closing channel");
            handle_amqp_response(amqp_connection_close(connection, AMQP_REPLY_SUCCESS), "Closing connection");
            handle_response(amqp_destroy_connection(connection), "Ending connection");
        }
        catch (const std::exception& x)
        {
            printf("[%s] A major exception has occurred! %s\n",name.c_str(), x.what());
            teptr = std::current_exception();
        }
        catch (...)
        {
            printf("[%s] A major exception has occurred!\n", name.c_str());
            teptr = std::current_exception();
        }

        printf("[%s] Finished\n",name.c_str());
    });
}

void RabbitMQ_consumer::receive(std::vector<uchar>& msg)
{
    if (teptr) rethrow_exception(teptr);

    {
        std::unique_lock<std::mutex> lock(mx);

        while (msg_queue.size() == 0 && !do_exit)
        {
            waitrd.wait(lock);
        }

        msg = msg_queue.front();
        msg_queue.pop();

        lock.unlock();
    }

    if (msg.size() == 1 && msg[0] == 4)
    {
        printf("[%s] Recieved EOT\n", name.c_str());
        throw consumerException();
    }
}

void RabbitMQ_consumer::receive(std::string& msg)
{
    std::vector<uchar> vmsg;
    this->receive(vmsg);
    msg = std::string(msg.begin(), msg.end());
}

void RabbitMQ_consumer::receive(cv::Mat& img)
{
    std::vector<uchar> vmsg;
    this->receive(vmsg);
    img = cv::imdecode(cv::Mat(vmsg), cv::IMREAD_UNCHANGED);
}

void RabbitMQ_consumer::receive(cv::Mat& img, double& timestamp)
{
    std::vector<uchar> vmsg;
    this->receive(vmsg);

    timestamp = std::stod(std::string(vmsg.begin(), vmsg.begin() + 16));
    vmsg.erase(vmsg.begin(), vmsg.begin() + 16);
    img = cv::imdecode(cv::Mat(vmsg), cv::IMREAD_UNCHANGED);
}

std::thread& RabbitMQ_consumer::exit()
{
    if (teptr) rethrow_exception(teptr);
    printf("[%s] Setting exit flag & notifying\n", name.c_str());

    {
        std::lock_guard<std::mutex> guard(mx);
        do_exit = true;
    }

    waitrd.notify_all();
    return th;
}

void RabbitMQ_consumer::handle_amqp_response(amqp_rpc_reply_t x, std::string context)
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
            switch (x.reply.id)
            {
                case AMQP_CONNECTION_CLOSE_METHOD:
                {
                    amqp_connection_close_t* m = (amqp_connection_close_t*)x.reply.decoded;
                    std::stringstream strm;
                    strm << context << ": server connection error " << m->reply_code << ", message: ";
                    for (int k = 0; k < m->reply_text.len; ++k)
                        strm << static_cast<unsigned char*>(m->reply_text.bytes)[k];
                    err = strm.str();
                    break;
                }
                case AMQP_CHANNEL_CLOSE_METHOD:
                {
                    amqp_channel_close_t* m = (amqp_channel_close_t*)x.reply.decoded;
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

    throw consumerException(err.c_str());
}

void RabbitMQ_consumer::handle_response(int rc, std::string context)
{
    if (rc < 0)
        throw consumerException(context.c_str());
}

int consumer(const std::string& type, const std::string& queue, const std::string& exchange, const std::string& ip, const std::string& port)
{
    int rc = EXIT_SUCCESS;

    std::string message_type    = type;
    std::string queue_name      = queue;
    std::string exchange_name   = exchange;
    std::string IP_ADDRESS      = ip;
    int PORT_ID = std::stoi(port);

    std::string name = "Message-Consumer";
    RabbitMQ_consumer cons(name, IP_ADDRESS, PORT_ID, queue_name, exchange_name, "direct", message_type);
    cons.credential("xinxin", "xhl6457398yy");

    try
    {
        cons.run();

        if (message_type.compare("image") == 0)
        {
            while (true)
            {
                cv::Mat img;
                double timestamp = 0.0;
                cons.receive(img, timestamp);
                
                printf("[%s] Received an image of size: %dx%d timestamp: %f\n",
                       name.c_str(), img.cols, img.rows,timestamp);

                cv::namedWindow("Received Image");
                cv::imshow("Received Image", img);
                cv::waitKey(10);
            }
        }
        else if (message_type.compare("imu") == 0)
        {
            std::string filename = "./iphone_imu.txt";
            std::ofstream fout(filename.c_str(), std::ios::app);

            while (true)
            {
                std::string msg;
                cons.receive(msg);
                
                Json::Value JSON_msg_convert;
                Json::Reader JSON_reader;
                if (JSON_reader.parse(msg.c_str(), msg.c_str() + msg.length(), JSON_msg_convert))
                {
                    double timestamp = JSON_msg_convert["timestamp"].asDouble();
                    double acc_x = JSON_msg_convert["acc_x"].asDouble();
                    double acc_y = JSON_msg_convert["acc_y"].asDouble();
                    double acc_z = JSON_msg_convert["acc_z"].asDouble();
                    double gyr_x = JSON_msg_convert["gyr_x"].asDouble();
                    double gyr_y = JSON_msg_convert["gyr_y"].asDouble();
                    double gyr_z = JSON_msg_convert["gyr_z"].asDouble();

                    std::cout << "[" << name.c_str() << "] ";
                    std::cout << std::setprecision(16) << timestamp << " ";
                    std::cout << std::setprecision(8)
                        << acc_x << " "
                        << acc_y << " "
                        << acc_z << " "
                        << gyr_x << " "
                        << gyr_y << " "
                        << gyr_z << std::endl;

                    fout << std::setprecision(16) << timestamp << " ";
                    fout << std::setprecision(8)
                        << acc_x << " "
                        << acc_y << " "
                        << acc_z << " "
                        << gyr_x << " "
                        << gyr_y << " "
                        << gyr_z << std::endl;
                }
                else
                    printf("[%s] %s\n", name.c_str(), JSON_reader.getFormatedErrorMessages().c_str());
            }

            fout.close();
        }
    }
    catch (const std::exception& x)
    {
        printf("[%s] Exception: %s", name.c_str(), x.what());
        rc = EXIT_FAILURE;
    }

    printf("[%s] Quitting\n", name.c_str());
    
    auto& th = cons.exit();
    if (th.joinable()) th.join();
    return rc;
}
