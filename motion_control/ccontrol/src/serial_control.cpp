#include "serial_control.h"

namespace SerialController {

SerialControllerInterface::SerialControllerInterface(const int& motor_id,
                                                    const int& main_can_id,
                                                    const string& port,
                                                    const BaudRate& baudrate,
                                                    const float& timeout) {
    motor_id_ = motor_id;
    main_can_id_ = main_can_id;
    port_ = port;
    baudrate_ = baudrate;
    timeout_ = timeout;
    try{
        serial_port_.Open(port_);
    } catch(exception& e) {
        cout << "Error: Cannot open serial port" << endl;
    }
    // Set the baud rates.
    serial_port_.SetBaudRate(BaudRate::BAUD_115200);
    // Set the number of data bits.
    serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    // Set the hardware flow control.
    serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
     // Set the parity.
    serial_port_.SetParity(Parity::PARITY_NONE);
    // Set the number of stop bits.
    serial_port_.SetStopBits(StopBits::STOP_BITS_1) ;
        
    
}

void SerialControllerInterface::enter_AT_mode_() {
    int hex = 0x41542b41540d0a;
    std::stringstream ss;
    ss << std::hex << hex;
    serial_port_.Write(ss.str()) ;
}

} // namespace SerialController