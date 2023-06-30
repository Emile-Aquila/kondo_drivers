//
// Created by emile on 23/06/30.
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <kondo_drivers/kondo_driver_service.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <kondo_drivers/b3m_commands.hpp>


namespace kondo_drivers{

    KondoB3mDriverService::KondoB3mDriverService(const rclcpp::NodeOptions & options)
            : Node("b3m_driver_node_component", options) {
        _pub_serial = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
        using namespace std::placeholders;
        _b3m_service = this->create_service<b3m_srv>("practice_service",
                                                     std::bind(&KondoB3mDriverService::_b3m_service_callback, this, _1, _2));
    }

    KondoB3mDriverService::~KondoB3mDriverService(){}

    void KondoB3mDriverService::_b3m_service_callback(std::shared_ptr<b3m_srv::Request> request,
                                                      std::shared_ptr<b3m_srv::Response> response) {
        using SerialData = std_msgs::msg::UInt8MultiArray;

        // TODO: b3mからのfbの扱い
        SerialData b3m_cmd_data;
        switch(request->b3m_msg.command_type){
            case b3m_msg::CMD_WRITE_B3M:
                b3m_cmd_data = generate_b3m_write_cmd(request->b3m_msg.servo_id, request->b3m_msg.cmd_write);
                _pub_serial->publish(b3m_cmd_data);
                break;
            case b3m_msg::CMD_SET_POS_B3M:
                b3m_cmd_data = generate_b3m_set_pos_cmd(request->b3m_msg.servo_id, request->b3m_msg.cmd_set_pos);
                _pub_serial->publish(b3m_cmd_data);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "[KONDO::B3M] Unknown Command Received!");
                break;
        }
        response->is_success = true; // TODO: 実装
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(kondo_drivers::KondoB3mDriverService)
