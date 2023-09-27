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
        using namespace std::chrono_literals;
        using namespace std::placeholders;
        _pub_serial = this->create_publisher<SerialData>("serial_write", 10);
        _sub_kondo = this->create_subscription<kondo_msg>("b3m_topic", 10,
                       std::bind(&KondoB3mDriverNode::_b3m_subscriber_callback, this, std::placeholders::_1));
        _timer_for_init = this->create_wall_timer(50ms, std::bind(&KondoB3mDriverNode::_timer_for_init_callback, this));
    }

    KondoB3mDriverNode::~KondoB3mDriverNode(){}

    void KondoB3mDriverNode::_b3m_subscriber_callback(const kondo_msg &msg) {
        // TODO: b3mからのfbの扱い
        SerialData b3m_cmd_data;
        switch(msg.command_type){
            case kondo_msg::CMD_WRITE:
                b3m_cmd_data = generate_b3m_write_cmd(msg.servo_id, msg.cmd_write);
                _pub_serial->publish(b3m_cmd_data);
                break;
            case kondo_msg::CMD_SET_POS:
                b3m_cmd_data = generate_b3m_set_pos_cmd(msg.servo_id, msg.cmd_set_pos);
                _pub_serial->publish(b3m_cmd_data);
                break;
            case kondo_msg::CMD_INIT:
                _cmd_quque_for_init.push(generate_b3m_write_cmd(msg.servo_id, gen_b3m_write_msg(msg.servo_id, 0x02, 0x28).cmd_write));
                _cmd_quque_for_init.push(generate_b3m_write_cmd(msg.servo_id, gen_b3m_write_msg(msg.servo_id, 0x00, 0x29).cmd_write));
                _cmd_quque_for_init.push(generate_b3m_write_cmd(msg.servo_id, gen_b3m_write_msg(msg.servo_id, msg.cmd_init.traj_type, 0x5c).cmd_write));
                _cmd_quque_for_init.push(generate_b3m_write_cmd(msg.servo_id, gen_b3m_write_msg(msg.servo_id, 0x00, 0x28).cmd_write));
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "[KONDO::B3M] Unknown Command Received!");
                break;
        }
    }

    kondo_drivers::msg::B3mServoMsg KondoB3mDriverNode::gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address) {
        kondo_drivers::msg::B3mServoMsg ans;
        ans.servo_id = servo_id;
        ans.command_type = kondo_drivers::msg::B3mServoMsg::CMD_WRITE;
        ans.cmd_write.txdata = TxData;
        ans.cmd_write.address = address;
        return ans;
    }


    void KondoB3mDriverNode::_serial_subscriber_callback(const std_msgs::msg::UInt8MultiArray &msg){
        if(msg.data.size() < 5)return;
        uint8_t received_id = msg.data[4];
    }

    void KondoB3mDriverNode::_timer_for_init_callback() {
        if(!_cmd_quque_for_init.empty()){
            auto b3m_cmd_data = _cmd_quque_for_init.front();
            _pub_serial->publish(b3m_cmd_data);
            _cmd_quque_for_init.pop();
        }
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(kondo_drivers::KondoB3mDriverNode)
