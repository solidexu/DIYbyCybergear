#ifndef _SERIAL_CONTROL_H_
#define _SERIAL_CONTROL_H_

#include <unistd.h>
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
#include <memory>


namespace SerialController {

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
#define MAX_TIMEOUT 25
    
using namespace std;
using namespace LibSerial;

// 通信类型
enum CmdModes : uint8_t{
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
enum RunModes : uint8_t {
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
        os << "format: " << data.format << " " ;
        if (data.format == "ba") {
            os << "pos: " << data.pos << " vel: " << data.vel << 
                " torque: " << data.torque << 
                " temperature: " << data.temperature_celsius;
        } else if (data.format == "u8"){
            os << "result is: " << data.u8_res;
        } else {
            os << "result is: " << data.f_res;
        }
        return os;
    }
};

// 串口控制类接口
class SerialControllerInterface {
public:
    // 禁止默认构造函数
    SerialControllerInterface() = delete;

    SerialControllerInterface(std::shared_ptr<SerialPort> serial_port_ptr = nullptr,
                            const int& motor_id = 1,
                            const int& main_can_id = 253,
                            const string& port = "/dev/ttyUSB0",
                            const BaudRate& baudrate = BaudRate::BAUD_921600,
                            const float& timeout = 0.1
                            );

    ~SerialControllerInterface();

// public 方法
public: 
    void enable_motor();
    void disable_motor();
    void set_motor_0position();
    void set_run_mode(const RunModes& run_mode);
    void write_single_param(const std::string& param_name, float value);
    float read_single_parameter(const std::string& parameter_name);
    void set_motor_position_control(const float& limit_spd, const float& loc_ref);
    bool read_standard_msg(Decode8BytesData& data); // 标准数据读取封装
// private 方法
private:
    // 进入AT模式   
    void enter_AT_mode_();
    void standard_write_preprocess_();
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
    void standard_parse_received_msg_(); // 标准解析封装
    Decode8BytesData parse_received_msg_(const std::vector<uint8_t>& received_msg_data, 
                                    const std::string& format = "f");

    std::vector<uint8_t> decode_canid_(const std::vector<uint8_t>& ex_can_id);

    Decode8BytesData decode_8_bytes_data_(const std::vector<uint8_t>& data, const std::string& format);

    // 辅助计算函数
    uint32_t merge_big_endian(const std::vector<uint8_t>& bytes);
    // 将 uint32_t 按照大端法拆分成 std::vector<uint8_t>
    std::vector<uint8_t> split_big_endian(uint32_t value);
    float _uint_to_float(uint16_t x, float x_min, float x_max, uint16_t bits);
    float _linear_mapping(float value, float value_min, float value_max, float target_min = 0.0, float target_max = 65535.0);
private:
    std::shared_ptr<SerialPort> serial_port_ptr_;
    int motor_id_;
    int main_can_id_;
    string port_;
    BaudRate baudrate_;
    float timeout_;


private:
    map<string, tuple<uint16_t, string>> param_table_; // 读取参数表
    map<string, tuple<uint16_t, string, float, float>> parameters_; // 写入参数表

private:
    void init_param_table_();
    void init_parameters_();
};


} // namespace SerialController
#endif // _SERIAL_CONTROL_H_