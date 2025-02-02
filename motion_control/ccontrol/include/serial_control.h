#ifndef _SERIAL_CONTROL_H_
#define _SERIAL_CONTROL_H_

#include <stdint.h>
#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <tuple>
#include <libserial/SerialPort.h>
#include <string.h>
#include <algorithm>

#define PI 3.1415926
#define P_MIN -4 * PI
#define P_MAX 4 * PI
#define V_MIN -30.0
#define V_MAX 30.0
#define T_MIN -12.0
#define T_MAX 12.0
#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0
#define TWO_BYTES_BITS 16

namespace SerialController {
    
using namespace std;
using namespace LibSerial;

// 通信类型
enum class CmdModes {
    GET_DEVICE_ID = 0,
    MOTOR_CONTROL = 1,
    MOTOR_FEEDBACK = 2,
    MOTOR_ENABLE = 3,
    MOTOR_STOP = 4,
    SET_MECHANICAL_ZERO = 6,
    SET_MOTOR_CAN_ID = 7,
    PARAM_TABLE_WRITE = 8,
    SINGLE_PARAM_READ = 17,
    SINGLE_PARAM_WRITE = 18,
    FAULT_FEEDBACK = 21
};

// 控制模式
enum class RunModes {
    CONTROL_MODE = 0, // 运控模式
    POSITION_MODE = 1, // 位置模式
    SPEED_MODE = 2, // 速度模式
    CURRENT_MODE = 3 // 电流模式
};

struct Decode8BytesData {
    string format;
    uint8_t u8_res;
    float f_res;
    float pos;
    float vel;
    float torque;
    float temperature_celsius;
    // 重写输出<<，用于输出结构体内容
    friend ostream& operator<<(ostream& os, const Decode8BytesData& data) {
        os << "format: " << data.format ;
        if (data.format == "ba") {
            os << "pos: " << data.pos << " vel: " << data.vel << 
                " torque: " << data.torque << 
                " temperature: " << data.temperature_celsius;
        } else if (data.format == "u8"){
            os << "result is: " << data.u8_res;
        } else {
            os << "result is: " << data.f_res;
        }
    }
};

// 串口控制类接口
class SerialControllerInterface {
public:
    // 禁止默认构造函数
    SerialControllerInterface() = delete;

    SerialControllerInterface(const int& motor_id = 1,
                            const int& main_can_id = 253,
                            const string& port = "/dev/ttyUSB0",
                            const BaudRate& baudrate = BaudRate::BAUD_921600,
                            const float& timeout = 0.1);

    ~SerialControllerInterface();

// public 方法
public: 

// private 方法
private:
    // 进入AT模式   
    void enter_AT_mode_();
    // 编码数据
    std::vector<uint8_t> encode_data_(uint8_t cmd_mode, 
                    const std::vector<uint8_t>& index = {}, 
                    const std::string& format = "", 
                    float value = 0.0, 
                    float min = 0.0, 
                    float max = 0.0, 
                    const std::vector<uint8_t>& bit29 = {}); 

    std::vector<uint8_t> encode_canid_(uint8_t cmd_mode);

    std::vector<uint8_t> encode_8_bytes_data_(const std::vector<uint8_t>& index, 
                                                const std::string& format, 
                                                float value, 
                                                float x_min, 
                                                float x_max);
    
    // 解码数据
    Decode8BytesData _parse_received_msg(const std::vector<uint8_t>& received_msg_data, 
                                    const std::string& format = "f");

    std::vector<uint8_t> decode_canid_(const std::vector<uint8_t>& ex_can_id);

    Decode8BytesData _decode_8_bytes_data(const std::vector<uint8_t>& data, const std::string& format);

private:
    SerialPort serial_port_;
    int motor_id_;
    int main_can_id_;
    string port_;
    BaudRate baudrate_;
    float timeout_;


private:
    map<string, tuple<int, string>> param_table_;
    map<string, tuple<int, string, float, float>> parameters;
};



uint32_t merge_big_endian(const std::vector<uint8_t>& bytes) {
    if (bytes.size() > 4) {
        std::cerr << "Input vector has more than 4 bytes, cannot fit into uint32_t." << std::endl;
        return 0;
    }

    uint32_t result = 0;
    // 大端法合并
    for (size_t i = 0; i < bytes.size(); ++i) {
        result |= static_cast<uint32_t>(bytes[i]) << ((3 - i) * 8);
    }
}

// 将 uint32_t 按照大端法拆分成 std::vector<uint8_t>
std::vector<uint8_t> split_big_endian(uint32_t value) {
    std::vector<uint8_t> bytes(4);
    for (int i = 0; i < 4; ++i) {
        // 右移相应位数并取最低 8 位
        bytes[i] = static_cast<uint8_t>((value >> ((3 - i) * 8)) & 0xFF);
    }
    return bytes;
}

float _uint_to_float(uint16_t x, float x_min, float x_max, uint16_t bits) {
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

    float _linear_mapping(float value, float value_min, float value_max, float target_min = 0.0, float target_max = 65535.0) {
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
#endif // _SERIAL_CONTROL_H_