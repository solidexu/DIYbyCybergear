#include "serial_control/include/serial_control.h"

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
    serial_port_.SetBaudRate(BaudRate::BAUD_921600);
    // Set the number of data bits.
    serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    // Set the hardware flow control.
    serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
     // Set the parity.
    serial_port_.SetParity(Parity::PARITY_NONE);
    // Set the number of stop bits.
    serial_port_.SetStopBits(StopBits::STOP_BITS_1) ;
        
    
}

void SerialControllerInterface::enable_motor(){
    if(!serial_port_.IsOpen()){
        try{
            serial_port_.Open(port_);
        } catch(exception& e) {
            cout << "Error: Cannot open serial port" << endl;
        }
    }
    // serial_port_.FlushIOBuffers(); // 清除缓冲区
    enter_AT_mode_();
    DataBuffer data_buffer = encode_data_(CmdModes::MOTOR_ENABLE);
    serial_port_.Write(data_buffer); // 发送数据
    std::string dataString;
    usleep(10000); // 等待10ms
    serial_port_.ReadLine(dataString,'\n',250);
    std::cout << "dataString: " << dataString << std::endl;
    if (dataString.find("OK") != std::string::npos) 
    {
        dataString.clear();
        serial_port_.ReadLine(dataString,'\n',250); // 读取下一行数据
    }
    std::cout << "dataString: " << dataString << std::endl;
    
    // DataBuffer read_buffer ;
    // size_t ms_timeout = 250 ;
    // usleep(1000);
    // try
    // {
    //     // Read as many bytes as are available during the timeout period.
    //     serial_port_.Read(read_buffer, 0, ms_timeout) ;
    // }
    // catch (const ReadTimeout&)
    // {
    //     for (size_t i = 0 ; i < read_buffer.size() ; i++)
    //     {
    //         std::cout << read_buffer.at(i) << std::flush ;
    //     }

    //     std::cerr << "The Read() call timed out waiting for additional data." << std::endl ;
    // }


}

void SerialControllerInterface::disable_motor(){
    if(!serial_port_.IsOpen()){
        try{
            serial_port_.Open(port_);
        } catch(exception& e) {
            cout << "Error: Cannot open serial port" << endl;
        }
    }
    serial_port_.FlushIOBuffers(); // 清除缓冲区
    enter_AT_mode_();
    DataBuffer data_buffer = encode_data_(CmdModes::MOTOR_STOP);
    serial_port_.Write(data_buffer); // 发送数据
    std::string dataString;
    serial_port_.ReadLine(dataString);
    std::cout << "dataString: " << dataString << std::endl;
}

void SerialControllerInterface::enter_AT_mode_() {
    string str;
    str.push_back(0x41);
    str.push_back(0x54);
    str.push_back(0x2b);
    str.push_back(0x41);
    str.push_back(0x54);
    str.push_back(0x0d);
    str.push_back(0x0a);
    serial_port_.Write(str) ;
    // std::cout << "Enter AT mode" << std::endl;
}

std::vector<uint8_t> SerialControllerInterface::encode_data_(uint8_t cmd_mode, 
                                            const std::vector<uint8_t>& index, 
                                            const std::string& format, 
                                            float value, 
                                            float min, 
                                            float max, 
                                            const std::vector<uint8_t>& bit29){
    std::vector<uint8_t> data_frame(17);
    // 帧头两个字节
    data_frame[0] = 0x41;
    data_frame[1] = 0x54;
    if (bit29.empty()) {
        // 计算4个字节长度的ex_can_id
        std::vector<uint8_t> ex_can_id = encode_canid_(cmd_mode);
        // 将ex_can_id转换为字节，并填充到字节数组中
        std::copy(ex_can_id.begin(), ex_can_id.end(), data_frame.begin() + 2);
    } else {
        std::copy(bit29.begin(), bit29.end(), data_frame.begin() + 2);
    }
    // 数据位个数放到第六位上
    data_frame[6] = 0x08;
    // 组合数据帧
    if (index.empty() && value != 0.0) {
        std::copy(reinterpret_cast<const uint8_t*>(&value), reinterpret_cast<const uint8_t*>(&value) + sizeof(value), data_frame.begin() + 7);
    } else {
        std::vector<uint8_t> data = encode_8_bytes_data_(index, format, value, min, max);
        std::copy(data.begin(), data.end(), data_frame.begin() + 7);
    }
    // 帧尾两个字节
    data_frame[15] = 0x0d;
    data_frame[16] = 0x0a;
    return data_frame;
}

std::vector<uint8_t> SerialControllerInterface::encode_canid_(uint8_t cmd_mode) {
    // 0. 声明一个字节数组，长度为4
    std::vector<uint8_t> canid(4);
    // 1. 将通讯模式cmd_mode转换为字节
    canid[0] = cmd_mode;
    // 2. 填充0x00到第二个字节
    canid[1] = 0x00;
    // 3. 将主机canid转换为字节，并填充到字节数组中
    canid[2] = main_can_id_;
    // 4. 将电机canid转换为字节，并填充到字节数组中
    canid[3] = motor_id_;
    // 5. 用16进制形式打印组合后的字节数组
    // print("组合扩展帧canid后的字节数组:", canid.hex())
    // 6. 将字节数组左移三位并把第二位置为1
    uint32_t canid_int = merge_big_endian(canid);
    canid_int = canid_int << 3;
    canid_int = canid_int | 0x04;
    // print("最终的扩展帧canid字节数组:", hex(canid))
    return split_big_endian(canid_int);
}


std::vector<uint8_t> SerialControllerInterface::encode_8_bytes_data_(const std::vector<uint8_t>& index, 
                                                const std::string& format, 
                                                float value, 
                                                float x_min, 
                                                float x_max){
     std::vector<uint8_t> data_frame(8);
        // 将index转换为字节，并填充到字节数组中,
        if (!index.empty()) {
            std::copy(index.begin(), index.end(), data_frame.begin());
        }
        data_frame[2] = 0x00;
        data_frame[3] = 0x00;
        // 将value转换为字节，并填充到字节数组中
        if (value != 0.0 && format != "ba") {
            value = std::max(std::min(value, x_max), x_min);
            if (format == "u8") {
                data_frame[4] = static_cast<uint8_t>(value);
            } else if (format == "f") {
                memcpy(data_frame.data() + 4, &value, sizeof(value));
            }
        } else if (value != 0.0 && format == "ba") {
            std::copy(reinterpret_cast<const uint8_t*>(&value), reinterpret_cast<const uint8_t*>(&value) + sizeof(value), data_frame.begin() + 4);
        }
        return data_frame;
}

Decode8BytesData SerialControllerInterface::_parse_received_msg(const std::vector<uint8_t>& received_msg_data,
                                                                const std::string& format) {
        
        std::vector<uint8_t> ex_can_id_msg(received_msg_data.begin()+2, received_msg_data.begin()+6);
        std::vector<uint8_t> ex_can_id = decode_canid_(ex_can_id_msg);
        std::cout << "ex_can_id is: " << merge_big_endian(ex_can_id) << std::endl;
        // 数据帧
        std::vector<uint8_t> data(received_msg_data.begin()+7, received_msg_data.begin()+15);
        Decode8BytesData result = _decode_8_bytes_data(data, format);
        // if (format == "ba") {
        //     std::cout << "pos: " << result.pos << std::endl;
        //     std::cout << "vel: " << result.vel << std::endl;
        //     std::cout << "torque: " << result.torque << std::endl;
        //     std::cout << "temperature_celsius: " << result.temperature_celsius << std::endl;
        // } else if (format == "u8"){
        //     std::cout << "result is: " << result.u8_res << std::endl;
        // } else {
        //     std::cout << "result is: " << result.f_res << std::endl;
        // }
        return result;
    }

std::vector<uint8_t> SerialControllerInterface::decode_canid_(const std::vector<uint8_t>& ex_can_id) {
        // 判断ex_can_id的type是否是bytearray
        uint32_t canid_int = merge_big_endian(ex_can_id);
        canid_int = canid_int >> 3;
        // print("解析后的canid字节数组: ", hex(canid))
        // 将int转换为bytearray
        return split_big_endian(canid_int);
}

Decode8BytesData SerialControllerInterface::_decode_8_bytes_data(const std::vector<uint8_t>& data, const std::string& format) {
    Decode8BytesData res;
    res.format = format;
    if (format == "u8") {
        // 将字节转位uint8
        res.u8_res =  static_cast<uint8_t>(data[4]);
    } else if (format == "f") {
        // 将字节数组转换为浮点数
        memcpy(&res.f_res, data.data(), sizeof(res.f_res));
    
    } else if (format == "ba") {
        // 将前两个字节[0-65535]映射到[-4pi , 4pi]
        res.pos = _uint_to_float((data[0] << 8) + data[1], P_MIN, P_MAX, TWO_BYTES_BITS);
        // 将第3和第4字节[0-65535]映射到[-30.0 , 30.0]
        res.vel = _uint_to_float((data[2] << 8) + data[3], V_MIN, V_MAX, TWO_BYTES_BITS);
        // 将第5和第6字节[0-65535]映射到[-12.0 , 12.0]
        res.torque = _uint_to_float((data[4] << 8) + data[5], T_MIN, T_MAX, TWO_BYTES_BITS);
        // 将第7和第8字节[0-65535]除以10.0得到温度值
        res.temperature_celsius = static_cast<float>((data[6] << 8) + data[7]) / 10.0;
        
    }
    return res;
}


uint32_t SerialControllerInterface::merge_big_endian(const std::vector<uint8_t>& bytes) {
    if (bytes.size() > 4) {
        std::cerr << "Input vector has more than 4 bytes, cannot fit into uint32_t." << std::endl;
        return 0;
    }

    uint32_t result = 0;
    // 大端法合并
    for (size_t i = 0; i < bytes.size(); ++i) {
        result |= static_cast<uint32_t>(bytes[i]) << ((3 - i) * 8);
    }
    return result;
}

std::vector<uint8_t> SerialControllerInterface::split_big_endian(uint32_t value) {
    std::vector<uint8_t> bytes(4);
    for (int i = 0; i < 4; ++i) {
        // 右移相应位数并取最低 8 位
        bytes[i] = static_cast<uint8_t>((value >> ((3 - i) * 8)) & 0xFF);
    }
    return bytes;
}

float SerialControllerInterface::_uint_to_float(uint16_t x, float x_min, float x_max, uint16_t bits) {
    /**
     * 将无符号整数转换为浮点数。
     *
     * 参数:
     * x: 输入的无符号整数。
     * x_min: 可接受的最小浮点数。
     * x_max: 可接受的最大浮点数。
     * bits: 输入无符号整数的位数。
     *
     * 返回:
     * 转换后的浮点数。
     */
    uint16_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    x = std::max(std::min(x, span), (uint16_t)0);  // Clamp x to the range [0, span]
    return offset * x / span + x_min;
}

float SerialControllerInterface::_linear_mapping(float value, float value_min, float value_max, float target_min, float target_max) {
    /**
     * 对输入值进行线性映射。
     *
     * 参数:
     * value: 输入值。
     * value_min: 输入值的最小界限。
     * value_max: 输入值的最大界限。
     * target_min: 输出值的最小界限。
     * target_max: 输出值的最大界限。
     *
     * 返回:
     * 映射后的值。
     */
    return static_cast<int>((value - value_min) / (value_max - value_min) * (target_max - target_min) + target_min);
}

} // namespace SerialController