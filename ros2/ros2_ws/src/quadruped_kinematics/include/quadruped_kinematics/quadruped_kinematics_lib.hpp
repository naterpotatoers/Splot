/**
 * quadruped_kinematics_lib.hpp
 */


 #ifndef QUADRUPED_KINEMATICS_LIB_HPP_
 #define QUADRUPED_KINEMATICS_LIB_HPP_


#include "servo_interfaces/msg/quadruped_joint_angles.hpp"
#include "servo_interfaces/msg/quadruped_leg_positions.hpp"


// PHYSICAL PARAMETERS
const double JOINT_1_PLANE_OFFSET = 0.0;
const double LINKAGE_2_LENGTH = 5.0;
const double LINKAGE_3_LENGTH = 7.42;
const double JOINT_1_X_OFFSET = 3.732;
const double JOINT_1_Y_OFFSET = 5.466;
const double JOINT_1_Z_OFFSET = 0.929 - 0.15;


servo_interfaces::msg::QuadrupedLegPositions apply_forward_kinematics(
  const servo_interfaces::msg::QuadrupedJointAngles& joint_angles);


servo_interfaces::msg::QuadrupedJointAngles apply_inverse_kinematics(
  const servo_interfaces::msg::QuadrupedLegPositions& leg_positions);


#endif
