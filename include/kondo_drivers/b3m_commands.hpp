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
#include <kondo_drivers/msg/cmd_set_pos_b3m.hpp>
#include <kondo_drivers/msg/cmd_write_b3m.hpp>

struct B3M_response_data_normal{
    uint8_t command;
    uint8_t status;
    uint8_t id;
};

struct B3M_response_data_set_pos : public B3M_response_data_normal{
    float current_pos;
};


std_msgs::msg::UInt8MultiArray generate_b3m_set_pos_cmd(uint8_t servo_id, const kondo_drivers::msg::CmdSetPosB3m &set_pos_cmd);
std_msgs::msg::UInt8MultiArray generate_b3m_write_cmd(uint8_t servo_id, const kondo_drivers::msg::CmdWriteB3m &write_cmd);

std::variant<B3M_response_data_normal, B3M_response_data_set_pos> get_b3m_response_data(const std_msgs::msg::UInt8MultiArray& byte_array);

#endif //ROS2_WS_B3M_COMMANDS_HPP
