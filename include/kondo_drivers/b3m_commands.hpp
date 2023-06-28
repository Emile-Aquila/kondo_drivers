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



std_msgs::msg::UInt8MultiArray generate_b3m_set_pos_cmd(uint8_t servo_id, const kondo_drivers::msg::CmdSetPosB3m &set_pos_cmd);
std_msgs::msg::UInt8MultiArray generate_b3m_write_cmd(uint8_t servo_id, const kondo_drivers::msg::CmdWriteB3m &write_cmd);

#endif //ROS2_WS_B3M_COMMANDS_HPP
