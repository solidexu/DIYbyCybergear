
#include "myrobot_hardware_interface.h"


MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {

    serial_port_ =  std::make_shared<SerialPort>();
    motors_ptrs_.resize(4, nullptr);
    motors_ptrs_[0] = std::make_shared<SerialControllerInterface>(serial_port_, 1);
    motors_ptrs_[1] = std::make_shared<SerialControllerInterface>(serial_port_, 2);
    motors_ptrs_[2] = std::make_shared<SerialControllerInterface>(serial_port_, 3);
    motors_ptrs_[3] = std::make_shared<SerialControllerInterface>(serial_port_, 4);

    motors_ptrs_[0]->set_motor_0position(); // set motor 0 position
    motors_ptrs_[1]->set_motor_0position(); // set motor 0 position
    motors_ptrs_[2]->set_motor_0position(); // set motor 0 position
    motors_ptrs_[3]->set_motor_0position(); // set motor 0 position
    sleep(1); // sleep 1s
    std::cout << "Finish: set motor 0 position" << std::endl;
    motors_ptrs_[0]->enable_motor();
    motors_ptrs_[1]->enable_motor();
    motors_ptrs_[2]->enable_motor();
    motors_ptrs_[3]->enable_motor();
    sleep(1); // sleep 1s
    // Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
    //Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
    //Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}



MyRobot::~MyRobot() {
    for (size_t i = 0; i < motors_ptrs_.size(); i++)
    {
        motors_ptrs_[i]->disable_motor();
    }
    
}


void MyRobot::init() {
    /* *********************************************************************
    ************************************Joint A ****************************
    ************************************************************************ */ 
    // Create joint_state_interface for JointA
    hardware_interface::JointStateHandle jointStateHandleA("JointA", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandleA);
    
    // Create position joint interface as JointA accepts position command.
    hardware_interface::JointHandle jointPositionHandleA(jointStateHandleA, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandleA);
    // Create Joint Limit interface for JointA
    joint_limits_interface::getJointLimits("JointA", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleA(jointPositionHandleA, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleA);  

    /* *********************************************************************
    ************************************Joint B ****************************
    ************************************************************************ */ 
    // Create joint_state_interface for JointB
    hardware_interface::JointStateHandle jointStateHandleB("JointB", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandleB);
    // Create position joint interface as JointB accepts position command.
    hardware_interface::JointHandle jointPositionHandleB(jointStateHandleB, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandleB);
    // Create Joint Limit interface for JointB
    joint_limits_interface::getJointLimits("JointB", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleB(jointPositionHandleB, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleB);

    /* *********************************************************************
    ************************************Joint C ****************************
    ************************************************************************ */ 
    // Create joint_state_interface for JointC
    hardware_interface::JointStateHandle jointStateHandleC("JointC", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandleC);

    // Create position joint interface as JointC accepts position command.
    hardware_interface::JointHandle jointPositionHandleC(jointStateHandleC, &joint_position_command_[2]);
    position_joint_interface_.registerHandle(jointPositionHandleC);
    // Create Joint Limit interface for JointC
    joint_limits_interface::getJointLimits("JointC", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    

    /* *********************************************************************
    ************************************Joint D ****************************
    ************************************************************************ */ 
    // Create joint_state_interface for JointD
    hardware_interface::JointStateHandle jointStateHandleD("JointD", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandleD);

    // Create position joint interface as JointD accepts position command.
    hardware_interface::JointHandle jointPositionHandleD(jointStateHandleD, &joint_position_command_[3]);
    position_joint_interface_.registerHandle(jointPositionHandleD);
    // Create Joint Limit interface for JointD
    joint_limits_interface::getJointLimits("JointD", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleD(jointPositionHandleD, limits);
    

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
    registerInterface(&positionJointSaturationInterface);    
}


//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}



void MyRobot::read() {

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       

  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]
    Decode8BytesData data;
    for (size_t i = 0; i < motors_ptrs_.size(); i++)
    {
        motors_ptrs_[i]->read_standard_msg(data);
        // std::cout << "motor" << i << ": " << data << std::endl;
        joint_position_[i] = data.pos;
        joint_velocity_[i] = data.vel;
        joint_effort_[i] = data.torque;
    }
    
}



void MyRobot::write(ros::Duration elapsed_time) {
    // Safety
    effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
    positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC


    // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
    for (size_t i = 0; i < motors_ptrs_.size(); i++)
    {
        motors_ptrs_[i]->set_motor_position_control(1, joint_position_command_[i]);
    }

}



int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "myrobot_hardware_interface");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2); 
    
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    
    return 0;
}

