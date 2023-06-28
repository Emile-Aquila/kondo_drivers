//
// Created by emile on 23/06/19.
//

#ifndef ROS2_WS_KONDO_DRIVER_NODE_HPP
#define ROS2_WS_KONDO_DRIVER_NODE_HPP

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


namespace kondo_drivers{
    class KondoB3mDriverNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit KondoB3mDriverNode(const rclcpp::NodeOptions & options);
        ~KondoB3mDriverNode();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;


        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _pub_serial;
        rclcpp::Subscription<kondo_msg>::SharedPtr _sub_kondo;

        void _b3m_subscriber_callback(const kondo_msg &msg);
    };
}


#endif //ROS2_WS_KONDO_DRIVER_NODE_HPP
