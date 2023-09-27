//
// Created by emile on 23/06/28.
//

#ifndef ROS2_WS_B3M_COMMANDS_HPP
#define ROS2_WS_B3M_COMMANDS_HPP

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <kondo_drivers/msg/b3m_set_pos_cmd.hpp>
#include <kondo_drivers/msg/b3m_write_cmd.hpp>


enum class B3M_COMMANDS: uint8_t {
    LOAD = 1,
    SAVE = 2,
    READ = 3,
    WRITE = 4,
    RESET = 5,
    POSITION = 6,
};

struct B3M_ResponseDataNormal{
    B3M_COMMANDS command;
    uint8_t status;
    uint8_t id;
};

struct B3M_ResponseDataSetPos : public B3M_ResponseDataNormal{
    float current_pos;
};


std_msgs::msg::UInt8MultiArray generate_b3m_set_pos_cmd(uint8_t servo_id, const kondo_drivers::msg::B3mSetPosCmd &set_pos_cmd);
std_msgs::msg::UInt8MultiArray generate_b3m_write_cmd(uint8_t servo_id, const kondo_drivers::msg::B3mWriteCmd &write_cmd);

std::variant<B3M_ResponseDataNormal, B3M_ResponseDataSetPos> get_b3m_response_data(const std_msgs::msg::UInt8MultiArray& byte_array);

#endif //ROS2_WS_B3M_COMMANDS_HPP
