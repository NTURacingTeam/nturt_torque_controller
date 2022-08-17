#ifndef TORQUE_CONTROLLER_HPP
#define TORQUE_CONTROLLER_HPP

// NTURT include
// CAN parser
#include <NTURT_CAN_Parser.hpp>
#include <cp_can_id.hpp>

// ROS message include
#include "can_msgs/Frame.h"
#include "std_msgs/Bool.h"

// STD include
#include <memory>
#include <signal.h>

// ROS include
#include <ros/ros.h>

/// \brief Class for sending can signal to inverter
class TorqueController {
    public:
        TorqueController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// \brief Function for testing purposes
        void test();

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

        /// \brief Motor spped
        double motor_speed_ = 0;

        /// \brief Threshold for motor spped to trigger soft start [rpm]
        double motor_speed_threshold_ = 60;

        /// \brief Torque slope for soft start [N * m / s]
        double torque_slope_ = 13;

        /// \brief Maximum torque output [N * m]
        double torque_max_ = 130;

        /// \brief Last torque command for soft start
        double torque_cmd_last_ = 0;

        /// \brief Last timestemp for soft start
        double timestemp_last_;

        /// \brief Error time duration for checking pedal pausibility
        double error_duration_ = 0;

        /// \brief Time threshold before trigger error for pedal pausibility [s]
        double error_duration_threshold_ = 1;

        /// \brief Error duraion discount factor when pedal is pausible
        double error_duration_discount_ = 0.1;

        /// \brief Control flag to determine if pedal is not pausible
        bool error_state_ = false;

        /// \brief Internal state to check if initialize is done
        bool state_ = false;
};

#endif // TORQUE_CONTROLLER_HPP
