#include "jaco_robot_hw_sim.h"
#include <cmath>        // std::abs

using namespace std;


bool JacoRobotHWSim::initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
    ROS_INFO("Starting to initialize jaco_simulation control plugin");
    int i;
    cmd_pos.resize(num_full_dof);
    cmd_vel.resize(num_full_dof);
    cmd_eff.resize(num_full_dof);
    zero_velocity_command.resize(num_full_dof, 0.0);
    pos.resize(num_full_dof);
    vel.resize(num_full_dof);
    eff.resize(num_full_dof);
    finger_pos.resize(num_finger_dof);
    pos_offsets.resize(num_arm_dof);
    soft_limits.resize(num_full_dof);

    for(std::size_t i = 0; i < pos_offsets.size(); ++i)
        pos_offsets[i] = 0.0;

    for(std::size_t i = 0; i < cmd_vel.size(); ++i)
       cmd_vel[i] = 0.0;

    // connect and register the joint state interface.
    // this gives joint states (pos, vel, eff) back as an output.
    hardware_interface::JointStateHandle state_handle_base("j2n6s200_joint_1", &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_handle_shoulder("j2n6s200_joint_2", &pos[1], &vel[1], &eff[1]);
    hardware_interface::JointStateHandle state_handle_elbow("j2n6s200_joint_3", &pos[2], &vel[2], &eff[2]);
    hardware_interface::JointStateHandle state_handle_wrist0("j2n6s200_joint_4", &pos[3], &vel[3], &eff[3]);
    hardware_interface::JointStateHandle state_handle_wrist1("j2n6s200_joint_5", &pos[4], &vel[4], &eff[4]);
    hardware_interface::JointStateHandle state_handle_wrist2("j2n6s200_joint_6", &pos[5], &vel[5], &eff[5]);
    hardware_interface::JointStateHandle state_handle_finger0("j2n6s200_joint_finger_1", &pos[6], &vel[6], &eff[6]);
    hardware_interface::JointStateHandle state_handle_finger1("j2n6s200_joint_finger_2", &pos[7], &vel[7], &eff[7]);

    jnt_state_interface.registerHandle(state_handle_base);
    jnt_state_interface.registerHandle(state_handle_shoulder);
    jnt_state_interface.registerHandle(state_handle_elbow);
    jnt_state_interface.registerHandle(state_handle_wrist0);
    jnt_state_interface.registerHandle(state_handle_wrist1);
    jnt_state_interface.registerHandle(state_handle_wrist2);
    jnt_state_interface.registerHandle(state_handle_finger0);
    jnt_state_interface.registerHandle(state_handle_finger1);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    // this takes joint velocities in as a command.
    hardware_interface::JointHandle vel_handle_base(jnt_state_interface.getHandle("j2n6s200_joint_1"), &cmd_vel[0]);
    hardware_interface::JointHandle vel_handle_shoulder(jnt_state_interface.getHandle("j2n6s200_joint_2"), &cmd_vel[1]);
    hardware_interface::JointHandle vel_handle_elbow(jnt_state_interface.getHandle("j2n6s200_joint_3"), &cmd_vel[2]);
    hardware_interface::JointHandle vel_handle_wrist0(jnt_state_interface.getHandle("j2n6s200_joint_4"), &cmd_vel[3]);
    hardware_interface::JointHandle vel_handle_wrist1(jnt_state_interface.getHandle("j2n6s200_joint_5"), &cmd_vel[4]);
    hardware_interface::JointHandle vel_handle_wrist2(jnt_state_interface.getHandle("j2n6s200_joint_6"), &cmd_vel[5]);
    hardware_interface::JointHandle vel_handle_finger0(jnt_state_interface.getHandle("j2n6s200_joint_finger_1"), &cmd_vel[6]);
    hardware_interface::JointHandle vel_handle_finger1(jnt_state_interface.getHandle("j2n6s200_joint_finger_2"), &cmd_vel[7]);

    jnt_vel_interface.registerHandle(vel_handle_base);
    jnt_vel_interface.registerHandle(vel_handle_shoulder);
    jnt_vel_interface.registerHandle(vel_handle_elbow);
    jnt_vel_interface.registerHandle(vel_handle_wrist0);
    jnt_vel_interface.registerHandle(vel_handle_wrist1);
    jnt_vel_interface.registerHandle(vel_handle_wrist2);
    jnt_vel_interface.registerHandle(vel_handle_finger0);
    jnt_vel_interface.registerHandle(vel_handle_finger1);

    registerInterface(&jnt_vel_interface);

    // connect and register the joint position interface
    // this takes joint positions in as a command.
    hardware_interface::JointHandle pos_handle_base(jnt_state_interface.getHandle("j2n6s200_joint_1"), &cmd_pos[0]);
    hardware_interface::JointHandle pos_handle_shoulder(jnt_state_interface.getHandle("j2n6s200_joint_2"), &cmd_pos[1]);
    hardware_interface::JointHandle pos_handle_elbow(jnt_state_interface.getHandle("j2n6s200_joint_3"), &cmd_pos[2]);
    hardware_interface::JointHandle pos_handle_wrist0(jnt_state_interface.getHandle("j2n6s200_joint_4"), &cmd_pos[3]);
    hardware_interface::JointHandle pos_handle_wrist1(jnt_state_interface.getHandle("j2n6s200_joint_5"), &cmd_pos[4]);
    hardware_interface::JointHandle pos_handle_wrist2(jnt_state_interface.getHandle("j2n6s200_joint_6"), &cmd_pos[5]);
    hardware_interface::JointHandle pos_handle_finger0(jnt_state_interface.getHandle("j2n6s200_joint_finger_1"), &cmd_pos[6]);
    hardware_interface::JointHandle pos_handle_finger1(jnt_state_interface.getHandle("j2n6s200_joint_finger_2"), &cmd_pos[7]);

    jnt_pos_interface.registerHandle(pos_handle_base);
    jnt_pos_interface.registerHandle(pos_handle_shoulder);
    jnt_pos_interface.registerHandle(pos_handle_elbow);
    jnt_pos_interface.registerHandle(pos_handle_wrist0);
    jnt_pos_interface.registerHandle(pos_handle_wrist1);
    jnt_pos_interface.registerHandle(pos_handle_wrist2);
    jnt_pos_interface.registerHandle(pos_handle_finger0);
    jnt_pos_interface.registerHandle(pos_handle_finger1);

    registerInterface(&jnt_pos_interface);


    ROS_INFO("Register Effort Interface...");

    // connect and register the joint position interface
    // this takes joint effort in as a command.
    hardware_interface::JointHandle eff_handle_base(jnt_state_interface.getHandle("j2n6s200_joint_1"), &cmd_eff[0]);
    hardware_interface::JointHandle eff_handle_shoulder(jnt_state_interface.getHandle("j2n6s200_joint_2"), &cmd_eff[1]);
    hardware_interface::JointHandle eff_handle_elbow(jnt_state_interface.getHandle("j2n6s200_joint_3"), &cmd_eff[2]);
    hardware_interface::JointHandle eff_handle_wrist0(jnt_state_interface.getHandle("j2n6s200_joint_4"), &cmd_eff[3]);
    hardware_interface::JointHandle eff_handle_wrist1(jnt_state_interface.getHandle("j2n6s200_joint_5"), &cmd_eff[4]);
    hardware_interface::JointHandle eff_handle_wrist2(jnt_state_interface.getHandle("j2n6s200_joint_6"), &cmd_eff[5]);
    hardware_interface::JointHandle eff_handle_finger0(jnt_state_interface.getHandle("j2n6s200_joint_finger_1"), &cmd_eff[6]);
    hardware_interface::JointHandle eff_handle_finger1(jnt_state_interface.getHandle("j2n6s200_joint_finger_2"), &cmd_eff[7]);

    jnt_eff_interface.registerHandle(eff_handle_base);
    jnt_eff_interface.registerHandle(eff_handle_shoulder);
    jnt_eff_interface.registerHandle(eff_handle_elbow);
    jnt_eff_interface.registerHandle(eff_handle_wrist0);
    jnt_eff_interface.registerHandle(eff_handle_wrist1);
    jnt_eff_interface.registerHandle(eff_handle_wrist2);
    jnt_eff_interface.registerHandle(eff_handle_finger0);
    jnt_eff_interface.registerHandle(eff_handle_finger1);

    registerInterface(&jnt_eff_interface);

    // connect and register the joint mode interface
    // this is needed to determine if velocity or position control is needed.
    hardware_interface::JointModeHandle mode_handle("joint_mode", &joint_mode);
    jm_interface.registerHandle(mode_handle);


    pr_hardware_interfaces::PositionCommandHandle position_command_handle(
        "/hand", &movehand_state, &finger_pos);
    movehand_interface.registerHandle(position_command_handle);
    registerInterface(&movehand_interface);

    registerInterface(&jm_interface);

    // get soft limits from rosparams
    if (model_nh.hasParam("soft_limits/eff"))
    {
        model_nh.getParam("soft_limits/eff", soft_limits);
        ROS_INFO("Set soft_limits for eff to: [%f,%f,%f,%f,%f,%f,%f,%f]",
            soft_limits[0], soft_limits[1], soft_limits[2], soft_limits[3],
            soft_limits[4], soft_limits[5], soft_limits[6], soft_limits[7]);
    }
    else
    {
        ROS_ERROR("No soft limits set for the MICO!");
        throw std::runtime_error("no soft limits set for the MICO!");
    }

    // initialize default positions
    initializeOffsets();

    last_mode = hardware_interface::MODE_VELOCITY;

    // For each joint in the robot, find the joint in Gazebo which is simulating it, then keep
    // a pointer to the Gazebo joint. These are used to read/write joint states to/from Gazebo.
    for (int j = 0; j < joint_names_.size(); j++) {
        gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
        if (!joint)
        {
          ROS_ERROR_STREAM_NAMED("jaco_robot_hw_sim", "This robot has a joint named \"" << joint_names_[j]
            << "\" which is not in the gazebo model.");
          return false;
        }
        sim_joints_.push_back(joint);
    }

    // Get physics engine type
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
    physics_type_ = physics->GetType();
    if (physics_type_.empty()) {
        ROS_WARN_STREAM_NAMED("jaco_robot_hw_sim", "No physics type found.");
    }

    return true;
}

void JacoRobotHWSim::initializeOffsets()
{
    // // TODO might need to add back
    // this->readSim();

    // Next, we wrap the positions so they are within -pi to pi of
    // the hardcoded midpoints, and add that to the offset.
    for (int i = 0; i < num_arm_dof; i++)
    {
        while (this->pos[i] < hardcoded_pos_midpoints[i] - M_PI)
        {
            this->pos[i] += 2.0 * M_PI;
            this->pos_offsets[i] += 2.0 * M_PI;
        }
        while (this->pos[i] > hardcoded_pos_midpoints[i] + M_PI)
        {
            this->pos[i] -= 2.0 * M_PI;
            this->pos_offsets[i] -= 2.0 * M_PI;
        }

        ROS_INFO("Joint %d: %f %f", i, this->pos[i], this->pos_offsets[i] );
    }
}

void JacoRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
    for(unsigned int j = 0; j < sim_joints_.size(); j++) {
        sim_joints_[j]->SetVelocity(0, cmd_vel.at(j));

        // TODO
        // PID controller, must have gains available
        // double error;
        // if (e_stop_active_)
        //   error = -joint_velocity_[j];
        // else
        //   error = joint_velocity_command_[j] - joint_velocity_[j];
        // const double effort_limit = joint_effort_limits_[j];
        // const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
        //                             -effort_limit, effort_limit);
        // sim_joints_[j]->SetForce(0, effort);
    }

}

void JacoRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
    for(unsigned int j = 0; j < sim_joints_.size(); j++) {
        double position = sim_joints_[j]->Position(0);

        pos[j] += angles::shortest_angular_distance(pos[j], position);
        eff[j] = sim_joints_[j]->GetForce((unsigned int)(0));
        vel[j] = sim_joints_[j]->GetVelocity(0);
    }
}

PLUGINLIB_EXPORT_CLASS(JacoRobotHWSim, gazebo_ros_control::RobotHWSim)
