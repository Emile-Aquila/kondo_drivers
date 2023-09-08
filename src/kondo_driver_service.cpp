//
// Created by emile on 23/06/30.
//

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <kondo_drivers/kondo_driver_service.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <kondo_drivers/b3m_commands.hpp>

using namespace std::chrono_literals;


namespace kondo_drivers{

    KondoB3mDriverService::KondoB3mDriverService(const rclcpp::NodeOptions & options)
            : Node("b3m_driver_node_component", options) {
        using namespace std::placeholders;
        _pub_serial = this->create_publisher<serial_data>("serial_write", 10);
        _sub_serial = this->create_subscription<serial_data>("serial_read", 10, std::bind(&KondoB3mDriverService::_b3m_sub_callback, this, _1));
        _b3m_service = this->create_service<b3m_srv>("kondo_b3m_service", std::bind(&KondoB3mDriverService::_b3m_service_callback, this, _1, _2));
    }

    KondoB3mDriverService::~KondoB3mDriverService(){}

    void KondoB3mDriverService::_b3m_service_callback(std::shared_ptr<b3m_srv::Request> request,
                                                      std::shared_ptr<b3m_srv::Response> response) {
        using SerialData = std_msgs::msg::UInt8MultiArray;
        b3m_msg b3m_request = request->b3m_msg;

        SerialData b3m_cmd_data, recv_data;
        switch(b3m_request.command_type){
            case b3m_msg::CMD_WRITE_B3M:
                b3m_cmd_data = generate_b3m_write_cmd(b3m_request.servo_id, b3m_request.cmd_write);
                _pub_serial->publish(b3m_cmd_data);
                break;
            case b3m_msg::CMD_SET_POS_B3M:
                b3m_cmd_data = generate_b3m_set_pos_cmd(b3m_request.servo_id, b3m_request.cmd_set_pos);
                _pub_serial->publish(b3m_cmd_data);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "[KONDO::B3M] Unknown Command Received!");
                response->is_success = false;
                return;
        }
        response->is_success = rclcpp::wait_for_message(recv_data, this->shared_from_this(), "serial_read", 200ms);
        if(!response->is_success)return;


        if(_received_datas.count(b3m_request.servo_id)){  // 何らかのserialのデータを受け取った時
            if((b3m_request.command_type == b3m_msg::CMD_WRITE_B3M) && (_received_datas[b3m_request.servo_id] == B3M_COMMANDS::WRITE)){
                return;
            }else if((b3m_request.command_type == b3m_msg::CMD_SET_POS_B3M) && (_received_datas[b3m_request.servo_id] == B3M_COMMANDS::POSITION)){
                return;
            }else{
                response->is_success = false;
            }
        }else{
            response->is_success = false;
        }
    }

    void KondoB3mDriverService::_b3m_sub_callback(const serial_data& msg) {
        // msgはset_posは7、それ以外は基本的に5
        auto data = get_b3m_response_data(msg);
        std::visit([this](auto& tmp){
            this->_received_datas[tmp.id] = tmp.command;
        }, data);
        if(std::holds_alternative<B3M_ResponseDataNormal>(data)){
            B3M_ResponseDataNormal response = std::get<B3M_ResponseDataNormal>(data);

        }else if(std::holds_alternative<B3M_ResponseDataSetPos>(data)){
            B3M_ResponseDataSetPos response = std::get<B3M_ResponseDataSetPos>(data);

        }else{
            RCLCPP_ERROR(this->get_logger(), "[KONDO::B3M] Unknown Feedback Received!");
        };
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(kondo_drivers::KondoB3mDriverService)
