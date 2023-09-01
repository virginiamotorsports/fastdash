/** @file fastdash.h
 *
 *  @ingroup ROS2CAN_bridge
 *  @author Philipp Wuestenberg
 *  @brief  bidirectional ROS2 to CAN interface with topics and service
 */

#ifndef __fastdash_H__
#define __fastdash_H__

//#include <termios.h> //n
//#include <fstream> //n
//#include <sstream>//n
//#include <chrono>//n
//#include <iostream>//n
//#include <stdio.h>
//#include <stdint.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <string.h>
//#include <net/if.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <sys/ioctl.h>


//define pin names because numbers are confusing
#define a_pin 17
#define b_pin 27
#define c_pin 22
#define d_pin 5
#define e_pin 6
#define f_pin 26
#define g_pin 23

//#include <linux/can.h>
#include <linux/can/raw.h>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <float.h>
#include <cmath>
#include <sys/types.h>
#include <pwd.h>
#include <math.h>
#include <sstream>

#include <boost/asio.hpp>
//#include <boost/asio/io_service.hpp>
//#include <boost/asio/signal_set.hpp>
//#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "can_msgs/msg/frame.hpp"

#include <sys/stat.h>
// #include "can_msgs/srv/can_request.hpp"
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "dash_msgs/msg/imu_report.hpp"
#include "dash_msgs/msg/dash_report.hpp"
#include "dash_msgs/msg/brake_report.hpp"
#include "dash_msgs/msg/motec_report.hpp"
#include "dash_msgs/msg/suspension_report.hpp"
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/writer.hpp>

// #include "log.h"

const std::string version = "1.00 from: " + std::string(__DATE__) + " " + std::string(__TIME__);
const std::string programdescr = "ROS 2 to CAN-Bus Bridge\nVersion: " + version;

/**
 * @brief The fastdash bridge connects a canbus with the ROS2 topic system. 
 * @details A nodes is provided, which provides the bridge from a ROS topic to the CAN bus and from the CAN bus to a ROS topic. The node functions as a bidirectional bridge and provides a service to publish a message and receive the answer with the fitting message id. 
 * 
 */
class fastdash : public rclcpp::Node
{
    public:
        /**
         * @brief constructor for fastdash class
         * @details Within the constructor the topic and service naming is done. 
         */
        fastdash(std::string can_socket = "can0");//boost::asio::io_service& ios);
        
        /**
         * @brief Within the Init() fucntin the ROS and CAN setup is done.
         * @details Within the Init() function the ROS2 publisher, subscriber and the service server is initialized. In addition the socketcan interface is configured and assigned to the socket. The Init function is necessary as the topics need a fully constructed node class to be added to.
         */
        // void Init(const char* can_socket = "can0");//boost::asio::io_service& ios);
        /**
         * @brief destructor
         */
        ~fastdash();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        bool prev_bag_state = false;
        bool curr_bag_state = false;

        sensor_msgs::msg::Imu imu_msg;
        sensor_msgs::msg::NavSatFix gps_msg;
        dash_msgs::msg::BrakeReport brake_msg;
        dash_msgs::msg::MotecReport motec_msg;
        dash_msgs::msg::SuspensionReport sus_msg;
        dash_msgs::msg::DashReport dash_msg;
        std::shared_ptr<rcutils_uint8_array_t> ser_data_;
        std::unique_ptr<rosbag2_cpp::Writer> writer_;
        rosbag2_storage::StorageOptions storage_options_;
        rosbag2_cpp::ConverterOptions converter_options_;
        long data_collection_hyst;
        rclcpp::Publisher<dash_msgs::msg::DashReport>::SharedPtr publisher_;
        
        can_msgs::msg::Frame current_frame;
        
        /**
         * @brief The CanSendConfirm function is needed by the .async_write_some function and is called as confirmation for a successfull send process.
         */
        void log_brake(can_msgs::msg::Frame frame);
        void log_motec(can_msgs::msg::Frame frame);
        void log_imu(can_msgs::msg::Frame frame);

        void CanSendConfirm();

        void start_bag();

        void stop_bag();

        void initialize_headers();

        void publish_msg();

        int get_gear(float Mph, float RPM);

        /**
         * @brief The CanPublisher is listening to a ROS2 Topic and calls the CanSend Method.
         */
        void CanPublisher(const can_msgs::msg::Frame::SharedPtr msg);
        
        /**
         * @brief The CanSend method sends a ROS message to the CAN bus.
         * @details The CanSend function is Called by the CanPublisher and ther ros2can_srv. It converts the ROS message to a can_frame and adds the CAN Flags to the message ID.  
         */
        void CanSend(const can_msgs::msg::Frame msg);
        
        /**
         * @brief The CanListener listens to the CAN Bus and publishes the message to a ROS2 Topic.
         * @details The CanListener function is Called by the .async_read_some when a Message is received on the Can Socket. It converts the message to a ROS Message and publishes it to a ROS2 Topic. Afterwards .async_read_some must be called again to wait for further CAN Messages.
         */
        void CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream);
        
        /**
         * @brief The ros2can_service provides the possibility to send a can message and wait for a specific can message with a give CAN Message ID.
         */
        // void ros2can_srv(
        // const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        // const std::shared_ptr<can_msgs::srv::CanRequest::Request> request,
        // std::shared_ptr<can_msgs::srv::CanRequest::Response> response);
        
        /**
         * @biref The Stop method is needed as the interuped handler must be configered to the asio libary.
         */
        void stop();
        
        boost::asio::io_service ios;
        boost::asio::posix::basic_stream_descriptor<> stream;
        boost::asio::signal_set signals;

        struct sockaddr_can addr;
        struct can_frame frame;
        struct can_frame rec_frame;
        struct ifreq ifr;

        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        std::stringstream topicname_receive;
        std::stringstream topicname_transmit;
        std::stringstream servername;
};
#endif