/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/09
 *
 * <h2><center>&copy; COPYRIGHT 2021 YUE JIANG TECHNOLOGY</center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <cstring>
#include <dobot_bringup/tcp_socket.h>

#pragma pack(push, 1)
// 数据 按照 8 字节 以及  48 字节对齐的模式,
// 大小设计为  30 * 8 * 6 = 30 *6*sizeof(double) = 30 * sizeof(double)
struct RealTimeData
{
    uint16_t len;                   // 0000 ~ 0001  字符长度
    uint16_t Reserve[3];            // 0002 ~ 0007  占位符
    uint64_t digital_input_bits;    // 0008 ~ 0015  DI
    uint64_t digital_outputs;       // 0016 ~ 0023  DO
    uint64_t robot_mode;            // 0024 ~ 0031  机器人模式
    uint64_t controller_timer;      // 0032 ~ 0039
    uint64_t run_time;              // 0040 ~ 0047
    // 0048 ~ 0095                       //
    uint64_t test_value;            // 0048 ~ 0055  内存结构测试标准值  0x0123 4567 89AB CDEF
    double safety_mode;             // 0056 ~ 0063
    double speed_scaling;           // 0064 ~ 0071
    double linear_momentum_norm;    // 0072 ~ 0079
    double v_main;                  // 0080 ~ 0087
    double v_robot;                 // 0088 ~ 0095
    // 0096 ~ 0143                       //
    double i_robot;                         // 0096 ~ 0103
    double program_state;                   // 0104 ~ 0111
    double safety_status;                   // 0112 ~ 0119
    double tool_accelerometer_values[3];    // 0120 ~ 0143
    // 0144 ~ 0191                       //
    double elbow_position[3];    // 0144 ~ 0167
    double elbow_velocity[3];    // 0168 ~ 0191
    // 0192 ~ ...                        //
    double q_target[6];              // 0192 ~ 0239  //
    double qd_target[6];             // 0240 ~ 0287  //
    double qdd_target[6];            // 0288 ~ 0335  //
    double i_target[6];              // 0336 ~ 0383  //
    double m_target[6];              // 0384 ~ 0431  //
    double q_actual[6];              // 0432 ~ 0479  //
    double qd_actual[6];             // 0480 ~ 0527  //
    double i_actual[6];              // 0528 ~ 0575  //
    double i_control[6];             // 0576 ~ 0623  //
    double tool_vector_actual[6];    // 0624 ~ 0671  //
    double TCP_speed_actual[6];      // 0672 ~ 0719  //
    double TCP_force[6];             // 0720 ~ 0767  //
    double Tool_vector_target[6];    // 0768 ~ 0815  //
    double TCP_speed_target[6];      // 0816 ~ 0863  //
    double motor_temperatures[6];    // 0864 ~ 0911  //
    double joint_modes[6];           // 0912 ~ 0959  //
    double v_actual[6];              // 960  ~ 1007  //
    double dummy[9][6];              // 1008 ~ 1439  //
};
#pragma pack(pop)

/**
 * CR5Commander
 */
class CR5Commander
{
protected:
    static constexpr double PI = 3.1415926;

private:
    std::mutex mutex_;
    double current_joint_[6];
    double tool_vector_[6];
    RealTimeData real_time_data_;
    std::atomic<bool> is_running_;
    std::unique_ptr<std::thread> thread_;
    std::shared_ptr<TcpClient> motion_cmd_tcp_;
    std::shared_ptr<TcpClient> real_time_tcp_;
    std::shared_ptr<TcpClient> dash_board_tcp_;

public:
    explicit CR5Commander(const std::string& ip)
        : current_joint_{}, tool_vector_{}, real_time_data_{}, is_running_(false)
    {
        motion_cmd_tcp_ = std::make_shared<TcpClient>(ip, 30003);
        real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
        dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
    }

    ~CR5Commander()
    {
        is_running_ = false;
        thread_->join();
    }

    void getCurrentJointStatus(double* joint)
    {
        mutex_.lock();
        memcpy(joint, current_joint_, sizeof(current_joint_));
        mutex_.unlock();
    }

    void getToolVectorActual(double* val)
    {
        mutex_.lock();
        memcpy(val, tool_vector_, sizeof(tool_vector_));
        mutex_.unlock();
    }

    void recvTask()
    {
        uint32_t has_read;

        while (is_running_)
        {
            if (real_time_tcp_->isConnect())
            {
                try
                {
                    if (real_time_tcp_->tcpRecv(&real_time_data_, sizeof(real_time_data_), has_read, 5000))
                    {
                        if (real_time_data_.len != 1440)
                            continue;

                        mutex_.lock();
                        for (uint32_t i = 0; i < 6; i++)
                            current_joint_[i] = deg2Rad(real_time_data_.q_actual[i]);

                        memcpy(tool_vector_, real_time_data_.tool_vector_actual, sizeof(tool_vector_));
                        mutex_.unlock();
                    }
                    else
                    {
                        //                        ROS_WARN("tcp recv timeout");
                    }
                }
                catch (const TcpClientException& err)
                {
                    real_time_tcp_->disConnect();
                    ROS_ERROR("real time tcp recv error : %s", err.what());
                }
            }
            else
            {
                try
                {
                    real_time_tcp_->connect();
                }
                catch (const TcpClientException& err)
                {
                    ROS_ERROR("move cmd tcp recv error : %s", err.what());
                    sleep(3);
                }
            }

            if (!dash_board_tcp_->isConnect())
            {
                try
                {
                    dash_board_tcp_->connect();
                }
                catch (const TcpClientException& err)
                {
                    ROS_ERROR("dash tcp recv error : %s", err.what());
                    sleep(3);
                }
            }

            if (!motion_cmd_tcp_->isConnect())
            {
                try
                {
                    motion_cmd_tcp_->connect();
                }
                catch (const TcpClientException& err)
                {
                    ROS_ERROR("tcp recv error : %s", err.what());
                    sleep(3);
                }
            }
        }
    }

    void init()
    {
        try
        {
            is_running_ = true;
            thread_ = std::unique_ptr<std::thread>(new std::thread(&CR5Commander::recvTask, this));
        }
        catch (const TcpClientException& err)
        {
            ROS_ERROR("Commander : %s", err.what());
        }
    }

    bool isEnable() const
    {
        return real_time_data_.robot_mode == 5;
    }

    bool isConnected() const
    {
        return dash_board_tcp_->isConnect() && motion_cmd_tcp_->isConnect();
    }

    void dashboardDoCmd(const char* cmd, int32_t& err_id)
    {
        std::vector<std::string> result;
        tcpDoCmd(dash_board_tcp_, cmd, err_id, result);
    }

    void dashboardDoCmd(const char* cmd, int32_t& err_id, std::vector<std::string>& result)
    {
        tcpDoCmd(dash_board_tcp_, cmd, err_id, result);
    }

    void motionDoCmd(const char* cmd, int32_t& err_id)
    {
        std::vector<std::string> result;
        tcpDoCmd(motion_cmd_tcp_, cmd, err_id, result);
    }

    void motionDoCmd(const char* cmd, int32_t& err_id, std::vector<std::string>& result)
    {
        tcpDoCmd(motion_cmd_tcp_, cmd, err_id, result);
    }

    static void parseString(const std::string& str, const std::string& send_cmd, int32_t& err,
                            std::vector<std::string>& result)
    {
        if (str.find(send_cmd) == std::string::npos)
            throw std::logic_error(std::string("Invalid string : ") + str);

        std::size_t pos = str.find(',');
        if (pos == std::string::npos)
            throw std::logic_error(std::string("Has no ',' found : ") + str);

        // parse err id
        char buf[200];
        assert(pos < sizeof(buf));
        str.copy(buf, pos, 0);
        buf[pos] = 0;

        char* end;
        err = (int32_t)strtol(buf, &end, 10);
        if (*end != '\0')
            throw std::logic_error(std::string("Invalid err id: ") + str);

        // parse result
        std::size_t start_pos = str.find('{');
        if (start_pos == std::string::npos)
            throw std::logic_error(std::string("Has no '{': ") + str);
        std::size_t end_pos = str.find('}');
        if (end_pos == std::string::npos)
            throw std::logic_error(std::string("Has no '}': ") + str);

        assert(end_pos > start_pos);
        char* buf_str = new char[str.length() + 1];
        str.copy(buf_str, end_pos - start_pos - 1, start_pos + 1);

        std::stringstream ss;
        ss << buf_str;
        delete[] buf_str;

        while (ss.getline(buf, sizeof(buf), ','))
            result.emplace_back(buf);
    }

private:
    static void tcpDoCmd(std::shared_ptr<TcpClient>& tcp, const char* cmd, int32_t& err_id,
                         std::vector<std::string>& result)
    {
        try
        {
            uint32_t has_read;
            char buf[1024];
            memset(buf, 0, sizeof(buf));

            ROS_INFO("tcp send cmd : %s", cmd);
            tcp->tcpSend(cmd, strlen(cmd));

            char* recv_ptr = buf;

            while (true)
            {
                bool err = tcp->tcpRecv(recv_ptr, 1, has_read, 0);
                if (!err)
                {
                    ROS_ERROR("tcpDoCmd : recv timeout");
                    return;
                }

                if (*recv_ptr == ';')
                    break;
                recv_ptr++;
            }

            ROS_INFO("tcp recv cmd : %s", buf);
            parseString(buf, cmd, err_id, result);
        }
        catch (const std::logic_error& err)
        {
            ROS_ERROR("tcpDoCmd failed : %s", err.what());
        }
    }

    static inline double rad2Deg(double rad)
    {
        return rad * 180.0 / PI;
    }

    static inline double deg2Rad(double deg)
    {
        return deg * PI / 180.0;
    }
};
