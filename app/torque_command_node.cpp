#include "torque_controller.hpp"

int main(int argc, char **argv) {
    // Register as a ros node
    ros::init(argc, argv, "nturt_torque_cmd_node");

    // Create a node handle
    std::shared_ptr<ros::NodeHandle> nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // Initialize torque controller
    TorqueController torque_controller(nh);

    // Frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    // Main loop
    while (ros::ok()) {
        // std::cout << "here" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
