uint8 servo_id
uint8 command_type  # CMD_WRITE / CMD_SET_POS / CMD_INIT
B3mSetPosCmd cmd_set_pos
B3mWriteCmd cmd_write
B3mInitCmd cmd_init

uint8 CMD_WRITE = 0
uint8 CMD_SET_POS = 1
uint8 CMD_INIT = 2
