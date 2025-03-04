#include "serial_control/include/serial_control.h"

// Gtest 模板
#include <gtest/gtest.h>

#include <unistd.h>
#include <memory>
// TEST(SerialControlTest, Enable_Disable) {
//     using namespace SerialController;
//     SerialControllerInterface *serial = new SerialControllerInterface(1);
//     serial->enable_motor();
//     sleep(1); // sleep 1s
//     serial->set_motor_0position(); // set motor 0 position
//     sleep(1); // sleep 1s
//     serial->disable_motor();
// }
TEST(SerialControlTest, POSITION_MODE) {
    sleep(1);
    using namespace SerialController;
    std::shared_ptr<SerialPort> serial = std::make_shared<SerialPort>();
    std::shared_ptr<SerialControllerInterface> motor1 = std::make_shared<SerialControllerInterface>(serial, 1);
    std::shared_ptr<SerialControllerInterface> motor2 = std::make_shared<SerialControllerInterface>(serial, 2);
    std::shared_ptr<SerialControllerInterface> motor3 = std::make_shared<SerialControllerInterface>(serial, 3);
    std::shared_ptr<SerialControllerInterface> motor4 = std::make_shared<SerialControllerInterface>(serial, 4);
    motor4->set_motor_0position(); // set motor 0 position
    motor3->set_motor_0position(); // set motor 0 position
    motor2->set_motor_0position(); // set motor 0 position
    motor1->set_motor_0position(); // set motor 0 position
    sleep(1); // sleep 1s
    motor4->enable_motor();
    motor3->enable_motor();
    motor2->enable_motor();
    motor1->enable_motor();
    sleep(1); // sleep 1s
    motor4->set_run_mode(RunModes::POSITION_MODE);
    motor3->set_run_mode(RunModes::POSITION_MODE);
    motor2->set_run_mode(RunModes::POSITION_MODE);
    motor1->set_run_mode(RunModes::POSITION_MODE);
    sleep(1); // sleep 1s
    motor2->set_motor_position_control(2.0, -0.8);
    motor4->set_motor_position_control(1.0, 3.14);
    motor3->set_motor_position_control(1.0, 0.5);
    motor1->set_motor_position_control(2.0, -0.5);
    sleep(2); // sleep 1s

    Decode8BytesData data;
    motor1->read_standard_msg(data);
    std::cout << "motor1: " << data << std::endl;
    motor2->read_standard_msg(data);
    std::cout << "motor2: " << data << std::endl;
    motor3->read_standard_msg(data);
    std::cout << "motor3: " << data << std::endl;
    motor4->read_standard_msg(data);
    std::cout << "motor4: " << data << std::endl;
    
    motor4->set_motor_position_control(3.0, -3.14);
    motor1->set_motor_position_control(2.0, -0.0);
    motor3->set_motor_position_control(1.0, 0.0);
    motor2->set_motor_position_control(1.0, 0.2);
    sleep(2); // sleep 1s
    motor4->disable_motor();
    motor3->disable_motor();
    motor2->disable_motor();
    motor1->disable_motor();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}