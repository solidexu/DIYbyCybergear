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
    SerialControllerInterface *serial = new SerialControllerInterface(4);
    serial->set_motor_0position(); // set motor 0 position
    sleep(1); // sleep 1s
    serial->enable_motor();
    sleep(1); // sleep 1s
    serial->set_run_mode(RunModes::POSITION_MODE);
    sleep(1); // sleep 1s
    serial->set_motor_position_control(1.0, 3.14);
    sleep(1); // sleep 1s
    serial->set_motor_position_control(3.0, -3.14);
    sleep(1); // sleep 1s
    serial->disable_motor();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}