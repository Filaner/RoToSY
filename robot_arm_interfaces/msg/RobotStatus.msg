std_msgs/Header header

# Robot state machine value (mirrors Doosan STATE_* constants)
# 0=INITIALIZING, 1=STANDBY, 2=MOVING, 3=SAFE_OFF, 5=SAFE_STOP, 15=NOT_READY
int32  robot_state
string robot_state_str

bool   servo_on       # true when state == STANDBY or MOVING
bool   is_moving      # true when state == MOVING
bool   teaching_mode  # true when SetRobotMode(MANUAL) is active

float64[6] current_joints_deg  # joint positions [degrees]
float64[6] current_tcp         # TCP pose [mm, mm, mm, deg, deg, deg]

int32  error_code
string error_message
