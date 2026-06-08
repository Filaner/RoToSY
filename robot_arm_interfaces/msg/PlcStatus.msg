std_msgs/Header header

# PLC connection state and cached I/O values.
bool connected

# Digital input/output states indexed from the configured Modbus start address.
bool[] di
bool[] do_state

# Bit masks for compact consumers. Bit N corresponds to di[N] or do_state[N].
uint32 di_mask
uint32 do_mask
