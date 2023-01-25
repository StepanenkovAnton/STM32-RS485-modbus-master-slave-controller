# STM32-RS485-modbus-master-slave-controller
STM32- based RS-485-enabled modbus master+slave compound device
The device receives multiple input signals (analog, 4-20 mA current loop and logic, 0/10 V logic level) and NMEA data from GPS module.
Two RS-485 ports are used for communication to external HMI panel (the device acts like a MODBUS slave) and to current sensors (the device acts like a MODBUS master).
One RS-232 port is used for communication with satellite modem for logging data to external system.
