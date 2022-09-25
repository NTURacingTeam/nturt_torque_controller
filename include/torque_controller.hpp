/**
 * @file torque_controller.hpp
 * @author quantumspawner jet22854111@gmail.com
 * @brief ROS package for controlling motor torque output.
 */

#ifndef TORQUE_CONTROLLER_HPP
#define TORQUE_CONTROLLER_HPP

// std include
#include <bitset>
#include <iostream>
#include <memory>

// ros include
#include <ros/ros.h>

// ros message include
#include "can_msgs/Frame.h"
#include "std_msgs/Bool.h"

// nturt include
#include "cp_can_id.hpp"
#include "NTURT_CAN_Parser.hpp"

/**
 * @author quantumspawner jet22854111@gmail.com
 * @brief Class for sending can signal to inverter.
 */
class TorqueController {
    public:
        TorqueController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// @brief Update function for updating torque_controller.
        void update();
        
        /// @brief Function for converting internal state to string.
        std::string get_string();

    private:
        /// @brief Callback function when receiving message form topic "/sent_messages".
        void onCan(const can_msgs::Frame::ConstPtr &_msg);

        /// @brief Callback function when receiving message from topic "/node_state".
        void onState(const std_msgs::Bool::ConstPtr &_msg);

        /**
         * @brief Function for checking the plausibility (accelerator and brake) of padels.
         * @param _dt Time dirrerence between this and last call of the function.
         * @return Accelerator pedal travel (0 ~ 1) after checking the plausibility of padels.
         */
        double plausibility_check(double _dt);
        
        /**
         * @brief Function for handling soft start of the motor.
         * @param _accelerator_travel Travel of accelerator (0 ~ 1).
         * @param _dt Time dirrerence between this and last call of the function.
         * @return Torque command (0 ~ torque_max_).
         */
        double soft_start(double _accelerator_travel, double _dt);

        /// @brief Pointer to ros node handle.
        std::shared_ptr<ros::NodeHandle> nh_;

        /// @brief Publisher to topic "/sent_messages".
        ros::Publisher can_pub_;

        /// @brief Subscriber to topic "/received_messages".
        ros::Subscriber can_sub_;

        /// @brief Subscriber to "/node_state".
        ros::Subscriber state_sub_;

        /// @brief Can message to send to inverter.
        can_msgs::Frame can_msg_;

        /// @brief CAN parser.
        Parser parser_;

        // data input
        /// @brief Signal to activate this node controlled by topic "node_state".
        bool is_activated_ = false;

        /// @brief Accelerator pedal level a.
        double accelerator_level_a_ = 0;

        /// @brief Accelerator pedal level b.
        double accelerator_level_b_ = 0;

        /// @brief Accelerator pedal trigger.
        bool accelerator_triger_ = false;

        /// @brief Brake pedal level.
        double brake_level_ = 0;

        /// @brief Brake pedal trigger.
        bool brake_trigger_ = false;

        /// @brief Motor spped [rpm].
        double motor_speed_ = 0;

        // internal control parameters
        /// @brief Last timestemp when "update" function is called [s].
        double timestemp_last_;

        // accelerator pedal position sensor (apps)
        /// @brief Time duration to trigger accelerator pedal position sensor error.
        double apps_duration_ = 0;

        /// @brief Accelerator pedal position sensor error, when set to true, should disable inverter.
        bool apps_error_ = false;

        // brake system encoder (bse)
        /// @brief Time duration to trigger brake system encoder error.
        double bse_duration_ = 0;

        /// @brief Brake system encoder error, when set to true, should disable inverter.
        bool bse_error_ = false;

        // brake pedal plausibility check (bpps)
        /// @brief Brake system encoder error, when set to true, should disable inverter.
        bool bppc_error_ = false;

        // soft start
        /// @brief Last torque command when "update" is called [N * m].
        double torque_command_last_ = 0;

        // parameters of the node
        // accelerator pedal position sensor (apps)
        /// @brief Difference threshold of the accelerator pedal travel, when higher, trigger accelerator pedal position sensor error.
        double apps_travel_threshold_ = 0.1;

        /// @brief Time threshold before triggering accelerator pedal position sensor error [s].
        double apps_duration_threshold_ = 0.1;

        /// @brief Error duraion discount factor when accelerator pedal position sensor signals are pausible.
        double apps_duration_discount_ = 0.1;

        // brake system encoder (bse)
        /// @brief Time threshold before triggering brake system encoder error [s].
        double bse_duration_threshold_ = 0.1;

        /// @brief Error duraion discount factor when brake system encoder signal is pausible.
        double bse_duration_discount_ = 0.1;

        // brake pedal plausibility cehck (bppc)
        /// @brief Accelerator pedal travel threshold when higher, trigger brake pedal plausibility lock.
        double bppc_trigger_threshold_ = 0.15;

        /// @brief Accelerator pedal travel threshold when lower, release brake pedal plausibility lock.
        double bppc_release_threshold_ = 0.05;

        // soft start when motor speed is low
        /// @brief Threshold for motor spped when lower, trigger soft start [rpm].
        double soft_start_threshold_ = 60;

        /// @brief Torque slope for soft start [N * m / s].
        double soft_start_torque_slope_ = 10;

        // others
        /// @brief Maximum torque output [N * m].
        double torque_max_ = 100;
};

#endif // TORQUE_CONTROLLER_HPP
