#ifndef TORQUE_CONTROLLER_HPP
#define TORQUE_CONTROLLER_HPP

// STD include
#include <memory>
#include <signal.h>

// ROS include
#include <ros/ros.h>

// ROS message include
#include "can_msgs/Frame.h"
#include "std_msgs/Bool.h"

// NTURT include
// CAN parser
#include "NTURT_CAN_Parser.hpp"
#include "cp_can_id.hpp"

/// \brief Class for sending can signal to inverter
class TorqueController {
    public:
        TorqueController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// \brief Function for testing purposes
        void test();

        /// \brief Function for testing purposes
        void test1();

    private:
        /// \brief Callback function when receiving message form topic "sent_messages"
        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        /// \brief Callback function when receiving message from topic "node_state"
        void onState(const std_msgs::Bool::ConstPtr &_msg);

        /// \brief Pointer to ros node handle
        std::shared_ptr<ros::NodeHandle> nh_;
        
        /// \brief Publisher to mcu command
        ros::Publisher mcu_pub_;

        /// \brief Subscriber to can message
        ros::Subscriber can_sub_;

        /// \brief Subscriber to node state
        ros::Subscriber state_sub_;

        /// \brief Can parser
        Parser parser_;

        /// \brief Torque command
        double torque_cmd_;

        /// \brief Internal state to check if initialize is done
        bool state_ = false;
};

#endif // TORQUE_CONTROLLER_HPP
