#ifndef TORQUE_CONTROLLER_HPP
#define TORQUE_CONTROLLER_HPP

// NTURT can id
#include "cp_can_id.hpp"
#include "NTURT_CAN_Parser.hpp"

// ROS include
#include <ros/ros.h>

// ROS message include
#include "can_msgs/Frame.h"
#include "std_msgs/Bool.h"

// STD include
#include <bitset>
#include <memory>

/// \brief Class for sending can signal to inverter
class TorqueController {
    public:
        TorqueController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// \brief Update function for publishing can data to inverter
        void update();
        
        /// \brief Function for testing purposes
        void test();

    private:
        /// \brief Callback function when receiving message form topic "sent_messages"
        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        /// \brief Callback function when receiving message from topic "node_state"
        void onState(const std_msgs::Bool::ConstPtr &_msg);

        /// \brief Function for checking the plausibility (accelerator and brake) of padels
        /// \return Accelerator pedal travel (0 ~ 1) after checking the plausibility of padels
        double plausibility_check(double _time_period);
        
        /// \brief Function for handling soft start of the motor
        /// \return Torque command (0 ~ torque_max_)
        double soft_start(double _accelerator_travel, double _time_period);

        /// \brief Function for handling activation of inverter
        void activate_inverter();

        /// \brief Function for handling 

        /// \brief Pointer to ros node handle
        std::shared_ptr<ros::NodeHandle> nh_;

        /// \brief Publisher to mcu command
        ros::Publisher mcu_pub_;

        /// \brief Subscriber to can message
        ros::Subscriber can_sub_;

        /// \brief Subscriber to node state
        ros::Subscriber state_sub_;

        /// \brief Can message to send to inverter
        can_msgs::Frame can_msg_;

        /// \brief Can parser
        Parser parser_;

        // data input
        /// \brief Signal to activate this node controlled by topic "node_state"
        bool is_activated_ = false;

        /// \brief Accelerator pedal level a
        double accelerator_level_a_ = 0;

        /// \brief Accelerator pedal level b
        double accelerator_level_b_ = 0;

        /// \brief Accelerator pedal trigger
        bool accelerator_triger_ = false;

        /// \brief Brake pedal level
        double brake_level_ = 0;

        /// \brief Brake pedal trigger
        bool brake_trigger_ = false;

        /// \brief Motor spped [rpm]
        double motor_speed_ = 0;

        // internal control parameters
        /// \brief Last timestemp when "update" is called [s]
        double timestemp_last_;

        // accelerator pedal plausibility check (appc)
        /// \brief Time duration when triggering accelerator pedal plausibility check
        double appc_duration_ = 0;

        /// \brief Accelerator pedal plausibility check error, when set to true, should disable inverter
        bool appc_error_ = false;

        // brake pedal plausibility cehck (bppc)
        /// \brief Brake pedal plausibility check error, when set to true, should disable inverter
        bool bppc_error_ = false;

        // soft start
        /// \brief Last torque command when "update" is called [N * m]
        double torque_command_last_ = 0;

        // parameters of the node
        // accelerator pedal plausibility check (appc)
        /// \brief Difference threshold of the accelerator pedal travel, when higher, trigger accelerator pedal plausibility check
        double appc_travel_threshold_ = 0.1;

        /// \brief Time threshold before triggering accelerator pedal plausibility check [s]
        double appc_duration_threshold_ = 0.1;

        /// \brief Error duraion discount factor when accelerator pedal signals are pausible
        double appc_duration_discount_ = 0.1;

        // brake pedal plausibility cehck (bppc)
        /// \brief Accelerator pedal travel threshold when higher, trigger brake pedal plausibility lock
        double bppc_trigger_threshold_ = 0.15;

        /// \brief Accelerator pedal travel threshold when lower, release brake pedal plausibility lock
        double bppc_release_threshold_ = 0.05;

        // soft start when motor speed is low
        /// \brief Threshold for motor spped when lower, trigger soft start [rpm]
        double soft_start_threshold_ = 60;

        /// \brief Torque slope for soft start [N * m / s]
        double soft_start_torque_slope_ = 10;

        // others
        /// \brief Maximum torque output [N * m]
        double torque_max_ = 100;

        /// \brief Scale factor when converting pedal level to pedal travel
        double pedal_level_to_travel_ = 1 / (254 - 1);
};

#endif // TORQUE_CONTROLLER_HPP
