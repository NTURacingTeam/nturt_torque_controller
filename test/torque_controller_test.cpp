#include "torque_controller.hpp"

int main(int argc, char **argv) {
    // Register as a ros node
    ros::init(argc, argv, "nturt_torque_controller_test_node");

    // Create a node handle
    std::shared_ptr<ros::NodeHandle> nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // Initialize torque controller
    TorqueController torque_controller(nh);
    
    // Frequancy 1 Hz
    ros::Rate loop_rate(1);

    // Main loop
    while (ros::ok()) {
        // torque_controller.test1();
        torque_controller.test();
        torque_controller.test1();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
