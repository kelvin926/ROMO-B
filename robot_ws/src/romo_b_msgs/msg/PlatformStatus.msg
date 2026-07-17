# Wheel array order is always FL, FR, RL, RR.
uint8 STATE_DISCONNECTED=0
uint8 STATE_CONNECTED_SAFE=1
uint8 STATE_ARMED_AUTO=2
uint8 STATE_ESTOP=3

std_msgs/Header header
uint8 state
bool connected
bool auto_mode
bool estop
uint8 steer_mode
float32[4] wheel_speed_mps
float32[4] wheel_steer_rad
uint8 pcu_alive
uint8 hlv_alive
bool command_timed_out
bool feedback_timed_out
