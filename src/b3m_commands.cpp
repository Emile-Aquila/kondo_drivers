//
// Created by emile on 23/06/28.
//


#include <kondo_drivers/b3m_commands.hpp>

using SerialData = std_msgs::msg::UInt8MultiArray;
using set_pos_msg = kondo_drivers::msg::B3mSetPosCmd;
using write_msg = kondo_drivers::msg::B3mWriteCmd;


SerialData generate_b3m_set_pos_cmd(uint8_t servo_id, const set_pos_msg &set_pos_cmd){
    std_msgs::msg::UInt8MultiArray serial_tx_cmd;
    serial_tx_cmd.layout.data_offset = 0;
    serial_tx_cmd.layout.dim.resize(9);
    std::for_each(serial_tx_cmd.layout.dim.begin(), serial_tx_cmd.layout.dim.end(), [](auto& tmp){
        tmp.size = 1;
        tmp.stride = 1;
    });

    int target_pos = static_cast<int>(set_pos_cmd.target_pos * 100.0);
    // 0 ~ 32000(0x7d00)で指定する。(320.00 -> 32000)
    int move_time = set_pos_cmd.move_time;  // 0~65535ms (NORMAL mode?のときは最速で動くらしい?)

    serial_tx_cmd.data.resize(9);
    serial_tx_cmd.data[0] = 0x09; // size
    serial_tx_cmd.data[1] = static_cast<uint8_t>(B3M_COMMANDS::POSITION); // cmd (0x06: POSITION)
    serial_tx_cmd.data[2] = 0x00; // option (0x00: ERROR_STATUを返す)
    serial_tx_cmd.data[3] = servo_id; // ID
    serial_tx_cmd.data[4] = (target_pos & 0xff); // pos_L
    serial_tx_cmd.data[5] = ((target_pos>>8) & 0xff); // pos_H
    serial_tx_cmd.data[6] = (move_time & 0xff); // tim_L
    serial_tx_cmd.data[7] = ((move_time>>8) & 0xff); // tim_H
    serial_tx_cmd.data[8] = 0x00; // check sum

    std::for_each(serial_tx_cmd.data.begin(), serial_tx_cmd.data.end()-1, [&serial_tx_cmd](const auto& tmp){
        serial_tx_cmd.data[8] = (serial_tx_cmd.data[8] + tmp) & 0xff;
    });
    return serial_tx_cmd;
}


SerialData generate_b3m_write_cmd(uint8_t servo_id, const write_msg &write_cmd) {
    std_msgs::msg::UInt8MultiArray serial_tx_cmd;
    serial_tx_cmd.layout.data_offset = 0;
    serial_tx_cmd.layout.dim.resize(8);
    std::for_each(serial_tx_cmd.layout.dim.begin(), serial_tx_cmd.layout.dim.end(), [](auto& tmp){
        tmp.size = 1;
        tmp.stride = 1;
    });

    serial_tx_cmd.data.resize(8);
    serial_tx_cmd.data[0] = 0x08; // size
    serial_tx_cmd.data[1] = static_cast<uint8_t>(B3M_COMMANDS::WRITE); // cmd (0x04: WRITE)
    serial_tx_cmd.data[2] = 0x00; // option (0x00: ERROR_STATUを返す)
    serial_tx_cmd.data[3] = servo_id; // ID
    serial_tx_cmd.data[4] = write_cmd.txdata;
    serial_tx_cmd.data[5] = write_cmd.address;
    serial_tx_cmd.data[6] = 0x01; // count
    serial_tx_cmd.data[7] = 0x00; // check_sum

    std::for_each(serial_tx_cmd.data.begin(), serial_tx_cmd.data.end()-1, [&serial_tx_cmd](const auto& tmp){
        serial_tx_cmd.data[7] = (serial_tx_cmd.data[7] + tmp) & 0xff;
    });
    return serial_tx_cmd;
}

std::variant<B3M_ResponseDataNormal, B3M_ResponseDataSetPos> get_b3m_response_data(const std_msgs::msg::UInt8MultiArray& byte_array){
    std::variant<B3M_ResponseDataNormal, B3M_ResponseDataSetPos> ans;
    if(byte_array.data.size() == 5){
        if(byte_array.data[1] == 0x84){  // WRITEコマンド
            ans = B3M_ResponseDataNormal{static_cast<B3M_COMMANDS>(byte_array.data[1]), byte_array.data[2], byte_array.data[3]};
        }
    }else if(byte_array.data.size() == 7){
        if(byte_array.data[1] == 0x86){  // SET POSコマンド
            int tmp_value = byte_array.data[5] + (byte_array.data[6] << 8);
            if(tmp_value & (1 << 15)){
                tmp_value = -((tmp_value - 1) ^ (0xffff - 1));
            }
            float pos = (float)(tmp_value) / 100.0f;
            ans = B3M_ResponseDataSetPos{{static_cast<B3M_COMMANDS>(byte_array.data[1]), byte_array.data[2], byte_array.data[3]}, pos};
        }
    }
    return ans;
}
