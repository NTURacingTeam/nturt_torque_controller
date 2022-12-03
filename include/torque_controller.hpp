/**
 * @file torque_controller.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS package for controlling motor torque output.
 */

#ifndef TORQUE_CONTROLLER_HPP
#define TORQUE_CONTROLLER_HPP

// std include
#include <memory>
#include <string>
#include <vector>

// ros include
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

// nturt include
#include "nturt_ros_interface/GetCanData.h"
#include "nturt_ros_interface/RegisterCanNotification.h"
#include "nturt_ros_interface/TorqueControllerData.h"
#include "nturt_ros_interface/UpdateCanData.h"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for controlling inverter.
 */
class TorqueController {
    public:
        TorqueController(std::shared_ptr<ros::NodeHandle> &_nh);

        /// @brief Update function for updating torque_controller.
        void update();
        
        /// @brief Function for converting internal state to string.
        std::string get_string() const;

    private:
        /// @brief Pointer to ros node handle.
        std::shared_ptr<ros::NodeHandle> nh_;
        
        /// @brief Publisher to "/torque_controller_data", for publishing the internal data of the torque_controller.
        ros::Publisher controller_data_pub_;

        /// @brief Publisher to "/publish_can_frame", for publishing can frames.
        ros::Publisher publish_frame_pub_;

        /// @brief Publisher to "/update_can_data", for updating can data.
        ros::Publisher update_data_pub_;

        /// @brief Subscriber to can data notification topic, for getting can data when they got updated.
        ros::Subscriber notification_sub_;
        
        /// @brief Subscriber to "/node_state", for being controlled by nturt_state_controller.
        ros::Subscriber state_sub_;

        /// @brief Service client to "/get_can_data", for getting can data (not used for now).
        ros::ServiceClient get_data_clt_;

        /// @brief Service client to "/register_can_notification", for registering to notification.
        ros::ServiceClient register_clt_;

        // data input
        /// @brief Signal to activate this node controlled by topic "node_state".
        bool is_activated_ = false;

        /// @brief Gear dial of inverter mode.
        int gear_dial_ = 0;

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

        /// @brief Motor spped \f$[rpm]\f$.
        double motor_speed_ = 0;

        // internal control parameters
        /// @brief Last timestemp when "update" function is called \f$[s]\f$.
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
        /// @brief Last torque command when "update" is called \f$[N\cdot m]\f$.
        double torque_command_last_ = 0;

        // parameters of the node
        // accelerator pedal position sensor (apps)
        /// @brief Difference threshold of the accelerator pedal travel, when higher, trigger accelerator pedal position sensor error.
        double apps_travel_threshold_ = 0.1;

        /// @brief Time threshold before triggering accelerator pedal position sensor error \f$[s]\f$.
        double apps_duration_threshold_ = 0.1;

        /// @brief Error duraion discount factor when accelerator pedal position sensor signals are pausible.
        double apps_duration_discount_ = 0.1;

        // brake system encoder (bse)
        /// @brief Time threshold before triggering brake system encoder error \f$[s]\f$.
        double bse_duration_threshold_ = 0.1;

        /// @brief Error duraion discount factor when brake system encoder signal is pausible.
        double bse_duration_discount_ = 0.1;

        // brake pedal plausibility cehck (bppc)
        /// @brief Accelerator pedal travel threshold when higher, trigger brake pedal plausibility lock.
        double bppc_trigger_threshold_ = 0.15;

        /// @brief Accelerator pedal travel threshold when lower, release brake pedal plausibility lock.
        double bppc_release_threshold_ = 0.05;

        // soft start when motor speed is low
        /// @brief Threshold for motor spped when lower, trigger soft start \f$[rpm]\f$.
        double soft_start_threshold_ = 60;

        /// @brief Torque slope for soft start \f$[N\cdot m \cdot s^{-1}]\f$.
        double soft_start_torque_slope_ = 10;

        // others
        /// @brief Maximum torque output if gear_dail=0 \f$[N\cdot m]\f$.
        double torque_max_ = 134;

        /// @brief Maximum torque output if gear_dail=1 \f$[N\cdot m]\f$.
        double torque_max_slow_ = 80;

        /// @brief Callback function when receiving message form can data notification.
        void onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg);

        /// @brief Callback function when receiving message from topic "/node_state".
        void onState(const std_msgs::Bool::ConstPtr &_msg);

        /**
         * @brief Function for checking the plausibility (accelerator and brake) of padels.
         * @param _dt Time dirrerence between this and last call of the function.
         * @return Accelerator pedal travel (0 ~ 1) after checking the plausibility of padels.
         */
        double plausibility_check(double _dt);
        
        /**
         * @brief Function for handling soft start and gear dial of the motor.
         * @param _accelerator_travel Travel of accelerator (0 ~ 1).
         * @param _dt Time dirrerence between this and last call of the function.
         * @return Torque command (0 ~ torque_max_).
         */
        double soft_start(double _accelerator_travel, double _dt);
};

#endif // TORQUE_CONTROLLER_HPP
