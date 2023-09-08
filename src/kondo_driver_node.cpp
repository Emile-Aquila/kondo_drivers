//
// Created by emile on 23/06/19.
//
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <kondo_drivers/kondo_driver_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <kondo_drivers/b3m_commands.hpp>


namespace kondo_drivers{

    KondoB3mDriverNode::KondoB3mDriverNode(const rclcpp::NodeOptions & options)
    : Node("b3m_driver_node_component", options) {
        _pub_serial = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
//        _sub_serial = this->create_subscription<std_msgs::msg::UInt8MultiArray>("serial_read", 10);
        using namespace std::placeholders;

        _sub_kondo = this->create_subscription<kondo_msg>("kondo_b3m_topic",10,
                       std::bind(&KondoB3mDriverNode::_b3m_subscriber_callback, this, std::placeholders::_1));
    }

    KondoB3mDriverNode::~KondoB3mDriverNode(){}

    void KondoB3mDriverNode::_b3m_subscriber_callback(const kondo_msg &msg) {
        using SerialData = std_msgs::msg::UInt8MultiArray;

        // TODO: b3mからのfbの扱い
        SerialData b3m_cmd_data;
        switch(msg.command_type){
            case kondo_msg::CMD_WRITE_B3M:
                b3m_cmd_data = generate_b3m_write_cmd(msg.servo_id, msg.cmd_write);
                _pub_serial->publish(b3m_cmd_data);
                break;
            case kondo_msg::CMD_SET_POS_B3M:
                b3m_cmd_data = generate_b3m_set_pos_cmd(msg.servo_id, msg.cmd_set_pos);
                _pub_serial->publish(b3m_cmd_data);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "[KONDO::B3M] Unknown Command Received!");
                break;
        }
    }

    void KondoB3mDriverNode::_serial_subscriber_callback(const std_msgs::msg::UInt8MultiArray &msg){
        using SerialData = std_msgs::msg::UInt8MultiArray;
        if(msg.data.size() < 5)return;
        uint8_t received_id = msg.data[4];
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(kondo_drivers::KondoB3mDriverNode)
