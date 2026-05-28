std_msgs/Header header

bool   connected          # Modbus TCP 연결 상태

bool[] di                 # DI 상태 (di[0]=센서0, di[1]=센서1, ...)
bool[] do_state           # DO 상태 (do_state[0]=컨베이어, do_state[1]=추가출력, ...)

uint8  di_mask            # DI 비트마스크 (편의용)
uint8  do_mask            # DO 비트마스크 (편의용)
