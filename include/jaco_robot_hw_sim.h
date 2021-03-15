#ifndef JACO_ROBOT_HW_SIM_H
#define JACO_ROBOT_HW_SIM_H

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pr_ros_controllers/joint_mode_interface.h>
#include <pr_hardware_interfaces/PositionCommandInterface.h>

//#include <hardware_interface/controller_info_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

// ros
#include <ros/ros.h>
#include <ros/console.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros_control/robot_hw_sim.h>

#include <urdf/model.h>

// c++
#include <stdexcept>
#include <limits>
#include <iostream>

// kinova api
#include <kinova/KinovaTypes.h>
#include <kinova/Kinova.API.USBCommandLayerUbuntu.h>

using namespace std;

// This makes the reported joint values to start within urdf limits
static const double hardcoded_pos_midpoints[6] = { 0.0, M_PI, M_PI, 0.0, 0.0, 0.0 };
static const int num_full_dof = 8;
static const int num_arm_dof = 6;
static const int num_finger_dof = 2;

class JacoRobotHWSim: public gazebo_ros_control::RobotHWSim
{
    public:
        void initializeOffsets();

        inline double degreesToRadians(double degrees);
        inline double radiansToDegrees(double radians);
        inline double radiansToFingerTicks(double radians);
        inline double fingerTicksToRadians(double ticks);

        void sendPositionCommand(const std::vector<double>& command);
        void sendVelocityCommand(const std::vector<double>& command);
        void sendTorqueCommand(const std::vector<double>& command);
        void sendFingerPositionCommand(const std::vector<double>& command);

        // Pure virtual methods required for RobotHWSim.
        virtual bool initSim(
            const std::string& robot_namespace,
            ros::NodeHandle model_nh,
            gazebo::physics::ModelPtr parent_model,
            const urdf::Model *const urdf_model,
            std::vector<transmission_interface::TransmissionInfo> transmissions);
        virtual void readSim(ros::Time time, ros::Duration period);
        virtual void writeSim(ros::Time time, ros::Duration period);

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::EffortJointInterface jnt_eff_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::JointModeInterface jm_interface;

        pr_hardware_interfaces::PositionCommandInterface movehand_interface;
        pr_hardware_interfaces::MoveState movehand_state = pr_hardware_interfaces::IDLE;

        //JacoArm *arm;
        vector<double> cmd_pos;
        vector<double> cmd_vel;
        vector<double> cmd_eff;
        vector<double> pos; // contains full dof
        vector<double> finger_pos; // just fingers, used for finger position control
        vector<double> vel;
        vector<double> eff;
        vector<double> pos_offsets;
        vector<double> soft_limits;
        vector<double> zero_velocity_command;
        int joint_mode; // this tells whether we're in position or velocity control mode
        int last_mode;
};

#endif
