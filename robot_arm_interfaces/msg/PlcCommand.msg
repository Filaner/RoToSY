# Modbus coil or holding-register write command forwarded to plc_controller_node.
string target     # human-readable label for logging (e.g. 'PLC', 'INVERTER')
string command    # 'COIL' or 'REGISTER'
uint16 address    # Modbus coil / register address
int32  value      # 0|1 for COIL, 0..65535 for REGISTER
uint8  slave_id   # Modbus slave / station number
