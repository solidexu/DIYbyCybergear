import sys
sys.path.append('/disk1/repo/DIYbyCybergear')
from motion_control.serial_control_interface import *

if __name__ == '__main__':
    motor1 = SerialControllerInterface(1,port='/dev/ttyUSB0')
    motor2 = SerialControllerInterface(2,port='/dev/ttyUSB0')
    motor3 = SerialControllerInterface(3,port='/dev/ttyUSB0')   
    motor1.enable_motor()
    motor2.enable_motor()
    motor3.enable_motor()
    motor1.set_motor_0position()
    motor2.set_motor_0position()
    motor3.set_motor_0position()
    motor3.set_run_mode( RunModes.POSITION_MODE)
    motor2.set_run_mode( RunModes.POSITION_MODE)
    motor1.set_run_mode( RunModes.POSITION_MODE)
    time.sleep(1)
    
    motor3.set_motor_position_control(1.0, -1.3)
    # time.sleep(0.1)
    # motor1.set_run_mode( RunModes.POSITION_MODE)
    
    motor2.set_motor_position_control(1.0, -0.5)
    # time.sleep(0.1)
    
    motor1.set_motor_position_control(1.0, 1)
    time.sleep(1.5)
    motor1.set_motor_position_control(1.0, 0.0)
    # time.sleep(0.1)
    motor2.set_motor_position_control(1.0, 0.0)
    # time.sleep(0.1)
    motor3.set_motor_position_control(1.0, 0)
    time.sleep(3)
    
    time.sleep(1)
    motor1.disable_motor()
    motor2.disable_motor()
    motor3.disable_motor()
    
    del motor1
    del motor2
    del motor3