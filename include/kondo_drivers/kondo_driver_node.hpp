//
// Created by emile on 23/06/19.
//

#ifndef ROS2_WS_KONDO_DRIVER_NODE_HPP
#define ROS2_WS_KONDO_DRIVER_NODE_HPP

#include <functional>
#include <algorithm>
#include <chrono>
#include <queue>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <kondo_drivers/visibility.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <kondo_drivers/msg/b3m_set_pos_cmd.hpp>
#include <kondo_drivers/msg/b3m_write_cmd.hpp>


namespace kondo_drivers{
    class KondoB3mDriverNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit KondoB3mDriverNode(const rclcpp::NodeOptions & options);
        ~KondoB3mDriverNode();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        using SerialData = std_msgs::msg::UInt8MultiArray;

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _pub_serial;
        rclcpp::Subscription<kondo_msg>::SharedPtr _sub_kondo;
        rclcpp::TimerBase::SharedPtr _timer_for_init;
        std::queue<SerialData> _cmd_quque_for_init;

        void _b3m_subscriber_callback(const kondo_msg &msg);
        void _serial_subscriber_callback(const std_msgs::msg::UInt8MultiArray &msg);
        void _timer_for_init_callback();
        kondo_drivers::msg::B3mServoMsg gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address);
    };
}


#endif //ROS2_WS_KONDO_DRIVER_NODE_HPP
