#include "serial_control/include/serial_control.h"

// Gtest 模板
#include <gtest/gtest.h>

#include <unistd.h>
#include <memory>
TEST(SerialControlTest, Enable_Disable) {
    using namespace SerialController;
    SerialControllerInterface *serial = new SerialControllerInterface(1);
    serial->enable_motor();
    usleep(1000000); // sleep 1s
    serial->disable_motor();
}
TEST(SerialControlTest, Test2) {
    
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}