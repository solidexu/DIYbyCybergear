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

namespace SerialController {
    
using namespace std;
using namespace LibSerial;



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


} // namespace SerialController
#endif // _SERIAL_CONTROL_H_