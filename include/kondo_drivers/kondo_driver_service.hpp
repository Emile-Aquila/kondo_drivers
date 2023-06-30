//
// Created by emile on 23/06/30.
//

#ifndef ROS2_WS_KONDO_DRIVER_SERVICE_HPP
#define ROS2_WS_KONDO_DRIVER_SERVICE_HPP

#include <functional>
#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <kondo_drivers/visibility.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <kondo_drivers/msg/cmd_set_pos_b3m.hpp>
#include <kondo_drivers/msg/cmd_write_b3m.hpp>
#include <kondo_drivers/srv/kondo_b3m_srv.hpp>


namespace kondo_drivers{
    class KondoB3mDriverService : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit KondoB3mDriverService(const rclcpp::NodeOptions & options);
        ~KondoB3mDriverService();
    private:
        using b3m_srv = kondo_drivers::srv::KondoB3mSrv;
        using b3m_msg = kondo_drivers::msg::B3mServoMsg;

        rclcpp::Service<b3m_srv>::SharedPtr _b3m_service;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _pub_serial;
        void _b3m_service_callback(std::shared_ptr<b3m_srv::Request> request, std::shared_ptr<b3m_srv::Response> response);
    };
}

#endif //ROS2_WS_KONDO_DRIVER_SERVICE_HPP
