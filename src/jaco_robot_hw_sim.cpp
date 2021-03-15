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
    ROS_INFO("Starting to initialize jaco_hardware");
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

inline double JacoRobotHWSim::degreesToRadians(double degrees)
{
    return (M_PI / 180.0) * degrees;
}

inline double JacoRobotHWSim::radiansToDegrees(double radians)
{
    return (180.0 / M_PI) * radians;
}

inline double JacoRobotHWSim::radiansToFingerTicks(double radians)
{
    return (6800.0 / 80) * radians * 180.0 / M_PI; //this magic number was found in the kinova-ros code, kinova_driver/src/kinova_arm.cpp
}

inline double JacoRobotHWSim::fingerTicksToRadians(double ticks)
{
    return ticks * (80 / 6800.0) * M_PI / 180.0;  //this magic number was found in the kinova-ros code, kinova_driver/src/kinova_arm.cpp
}

void JacoRobotHWSim::sendPositionCommand(const std::vector<double>& command)
{
    // Need to send an "advance trajectory" with a single point and the correct settings
    // Angular position
    AngularInfo joint_pos;
    joint_pos.InitStruct();

    joint_pos.Actuator1 = float(radiansToDegrees(command.at(0) - pos_offsets[0]));
    joint_pos.Actuator2 = float(radiansToDegrees(command.at(1) - pos_offsets[1]));
    joint_pos.Actuator3 = float(radiansToDegrees(command.at(2) - pos_offsets[2]));
    joint_pos.Actuator4 = float(radiansToDegrees(command.at(3) - pos_offsets[3]));
    joint_pos.Actuator5 = float(radiansToDegrees(command.at(4) - pos_offsets[4]));
    joint_pos.Actuator6 = float(radiansToDegrees(command.at(5) - pos_offsets[5]));

    TrajectoryPoint trajectory;
    trajectory.InitStruct(); // initialize structure
    memset(&trajectory, 0, sizeof(trajectory));  // zero out the structure
    // trajectory.Position.Type = ANGULAR_POSITION; // set to angular position
    // trajectory.Position.Actuators = joint_pos; // position is passed in the position struct

    trajectory.Position.Delay = 0.0;
    trajectory.Position.HandMode = POSITION_MODE;
    trajectory.Position.Type = ANGULAR_POSITION;
    trajectory.Position.Fingers.Finger1 = float(radiansToFingerTicks(command.at(6)));
    trajectory.Position.Fingers.Finger2 = float(radiansToFingerTicks(command.at(7)));

    int r = NO_ERROR_KINOVA;
    r = SendAdvanceTrajectory(trajectory);
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not send : Error code %d",r);
    }

}

void JacoRobotHWSim::sendFingerPositionCommand(const std::vector<double>& command)
{

    // ROS_INFO_STREAM("pos finger" << command[6] << " " << command[7]);

    // Need to send an "advance trajectory" with a single point and the correct settings
    // Angular position

    AngularPosition arm_pos;
    GetAngularPosition(arm_pos);

    TrajectoryPoint trajectory;
    trajectory.InitStruct(); // initialize structure
    memset(&trajectory, 0, sizeof(trajectory));  // zero out the structure

    // Set arm velocity to zero
    trajectory.Position.Type = ANGULAR_POSITION; // set to angular velocity
    trajectory.Position.Actuators = arm_pos.Actuators;;

    trajectory.Position.Delay = 0.0;
    trajectory.Position.HandMode = POSITION_MODE;
    trajectory.Position.Type = ANGULAR_POSITION;
    trajectory.LimitationsActive = 0;

    trajectory.Position.Fingers.Finger1 = float(radiansToFingerTicks(command.at(6)));
    trajectory.Position.Fingers.Finger2 = float(radiansToFingerTicks(command.at(7)));

    int r = NO_ERROR_KINOVA;
    r = SendBasicTrajectory(trajectory);
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not send : Error code %d",r);
    }
}

void JacoRobotHWSim::sendVelocityCommand(const std::vector<double>& command)
{
    // Need to send an "advance trajectory" with a single point and the correct settings
    // Angular velocity
    // ROS_INFO_STREAM("vel " << command[0] << " " << command[1] << " "
    //     << command[2] << " " << command[3] << " " << command[4] << " "
    //     << command[5] << " " << command[6] << " " << command[7]);

    AngularInfo joint_vel;
    joint_vel.InitStruct();
    joint_vel.Actuator1 = float(radiansToDegrees(command.at(0)));
    joint_vel.Actuator2 = float(radiansToDegrees(command.at(1)));
    joint_vel.Actuator3 = float(radiansToDegrees(command.at(2)));
    joint_vel.Actuator4 = float(radiansToDegrees(command.at(3)));
    joint_vel.Actuator5 = float(radiansToDegrees(command.at(4)));
    joint_vel.Actuator6 = float(radiansToDegrees(command.at(5)));

    TrajectoryPoint trajectory;
    trajectory.InitStruct();
    memset(&trajectory, 0, sizeof(trajectory));

    trajectory.Position.Type = ANGULAR_VELOCITY;
    trajectory.Position.Actuators = joint_vel;

    trajectory.Position.HandMode = VELOCITY_MODE;
    trajectory.Position.Type = ANGULAR_VELOCITY;
    trajectory.Position.Fingers.Finger1 = float(radiansToFingerTicks(command.at(6)));
    trajectory.Position.Fingers.Finger2 = float(radiansToFingerTicks(command.at(7)));

    int r = NO_ERROR_KINOVA;
    r = SendAdvanceTrajectory(trajectory);
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not send : Error code %d",r);
    }
}

void JacoRobotHWSim::sendTorqueCommand(const std::vector<double>& command)
{
    std::vector<float> joint_eff;
    joint_eff.reserve(command.size());
    for (std::size_t i = 0; i < command.size(); ++i)
    {
        joint_eff.push_back(float(command[i]));
    }
    ROS_INFO_STREAM("eff " << float(joint_eff[5]));

    int r = NO_ERROR_KINOVA;
    r = SendAngularTorqueCommand(&joint_eff[0]);
    if (r != NO_ERROR_KINOVA) {
        ROS_ERROR("Could not send : Error code %d",r);
    }
}

void JacoRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
    sendVelocityCommand(cmd_vel);
}

void JacoRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
    // make sure that pos, vel, and eff are up to date.
    // TODO: If there is too much lag between calling read()
    // and getting the actual values back, we'll need to be
    // reading values constantly and storing them locally, so
    // at least there is a recent value available for the controller.

    AngularPosition arm_pos;
    AngularPosition arm_vel;
    AngularPosition arm_torq;

    // TODO must call to simulated joints
    // Requires 3 seperate calls to the USB
    // GetAngularPosition(arm_pos);
    // GetAngularVelocity(arm_vel);
    // GetAngularForce(arm_torq);

    pos[0] = degreesToRadians(double(arm_pos.Actuators.Actuator1)) + pos_offsets[0];
    pos[1] = degreesToRadians(double(arm_pos.Actuators.Actuator2)) + pos_offsets[1];
    pos[2] = degreesToRadians(double(arm_pos.Actuators.Actuator3)) + pos_offsets[2];
    pos[3] = degreesToRadians(double(arm_pos.Actuators.Actuator4)) + pos_offsets[3];
    pos[4] = degreesToRadians(double(arm_pos.Actuators.Actuator5)) + pos_offsets[4];
    pos[5] = degreesToRadians(double(arm_pos.Actuators.Actuator6)) + pos_offsets[5];
    pos[6] = fingerTicksToRadians(double(arm_pos.Fingers.Finger1));
    pos[7] = fingerTicksToRadians(double(arm_pos.Fingers.Finger2));

    // According to kinova-ros, the reported values are half of the actual.
    vel[0] = degreesToRadians(double(arm_vel.Actuators.Actuator1));
    vel[1] = degreesToRadians(double(arm_vel.Actuators.Actuator2));
    vel[2] = degreesToRadians(double(arm_vel.Actuators.Actuator3));
    vel[3] = degreesToRadians(double(arm_vel.Actuators.Actuator4));
    vel[4] = degreesToRadians(double(arm_vel.Actuators.Actuator5));
    vel[5] = degreesToRadians(double(arm_vel.Actuators.Actuator6));
    vel[6] = fingerTicksToRadians(double(arm_vel.Fingers.Finger1));
    vel[7] = fingerTicksToRadians(double(arm_vel.Fingers.Finger2));

    eff[0] = arm_torq.Actuators.Actuator1;
    eff[1] = arm_torq.Actuators.Actuator2;
    eff[2] = arm_torq.Actuators.Actuator3;
    eff[3] = arm_torq.Actuators.Actuator4;
    eff[4] = arm_torq.Actuators.Actuator5;
    eff[5] = arm_torq.Actuators.Actuator6;
    eff[6] = arm_torq.Fingers.Finger2;
    eff[7] = arm_torq.Fingers.Finger2;
}
