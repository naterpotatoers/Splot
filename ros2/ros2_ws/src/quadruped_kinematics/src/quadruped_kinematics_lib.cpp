/**
 * quadruped_kinematics_lib.cpp
 */


#include "quadruped_kinematics/quadruped_kinematics_lib.hpp"

#include "servo_interfaces/msg/quadruped_joint_angles.hpp"
#include "servo_interfaces/msg/quadruped_leg_positions.hpp"
#include "servo_interfaces/srv/quadruped_forward_kinematics.hpp"
#include "servo_interfaces/srv/quadruped_inverse_kinematics.hpp"
#include "quadruped_kinematics/kinematics_lib.hpp"


using namespace servo_interfaces::msg;
using namespace servo_interfaces::srv;


QuadrupedLegPositions apply_forward_kinematics(
  const QuadrupedJointAngles& joint_angles)
{
  QuadrupedLegPositions leg_positions = QuadrupedLegPositions();
  double x, y, z;

  // Front Right
  ForwardKinematics(
    joint_angles.leg_front_right.joint1_angle,
    joint_angles.leg_front_right.joint2_angle,
    joint_angles.leg_front_right.joint3_angle,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    -1.0,
    x, y, z);
  leg_positions.leg_front_right.x = x + JOINT_1_X_OFFSET;
  leg_positions.leg_front_right.y = y + JOINT_1_Y_OFFSET;
  leg_positions.leg_front_right.z = z + JOINT_1_Z_OFFSET;

  // Front Left
  ForwardKinematics(
    joint_angles.leg_front_left.joint1_angle,
    joint_angles.leg_front_left.joint2_angle,
    joint_angles.leg_front_left.joint3_angle,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    1.0,
    x, y, z);
  leg_positions.leg_front_left.x = x - JOINT_1_X_OFFSET;
  leg_positions.leg_front_left.y = y + JOINT_1_Y_OFFSET;
  leg_positions.leg_front_left.z = z + JOINT_1_Z_OFFSET;

  // Rear Left
  ForwardKinematics(
    joint_angles.leg_rear_left.joint1_angle,
    joint_angles.leg_rear_left.joint2_angle,
    joint_angles.leg_rear_left.joint3_angle,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    1.0,
    x, y, z);
  leg_positions.leg_rear_left.x = x - JOINT_1_X_OFFSET;
  leg_positions.leg_rear_left.y = y - JOINT_1_Y_OFFSET;
  leg_positions.leg_rear_left.z = z + JOINT_1_Z_OFFSET;

  // Rear Right
  ForwardKinematics(
    joint_angles.leg_rear_right.joint1_angle,
    joint_angles.leg_rear_right.joint2_angle,
    joint_angles.leg_rear_right.joint3_angle,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    -1.0,
    x, y, z);
  leg_positions.leg_rear_right.x = x + JOINT_1_X_OFFSET;
  leg_positions.leg_rear_right.y = y - JOINT_1_Y_OFFSET;
  leg_positions.leg_rear_right.z = z + JOINT_1_Z_OFFSET;

  return leg_positions;
}


QuadrupedJointAngles apply_inverse_kinematics(
  const QuadrupedLegPositions& leg_positions)
{
  QuadrupedJointAngles joint_angles = QuadrupedJointAngles();
  int ik_status;
  double j1_angle, j2_angle, j3_angle;

  // Front Right
  ik_status = InverseKinematics(
    leg_positions.leg_front_right.x - JOINT_1_X_OFFSET,
    leg_positions.leg_front_right.y - JOINT_1_Y_OFFSET,
    leg_positions.leg_front_right.z - JOINT_1_Z_OFFSET,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    -1.0, -1.0,
    j1_angle, j2_angle, j3_angle);
  joint_angles.leg_front_right.joint1_angle = j1_angle;
  joint_angles.leg_front_right.joint2_angle = j2_angle;
  joint_angles.leg_front_right.joint3_angle = j3_angle;
  joint_angles.leg_front_right.position_reachable = (ik_status == 0);
  joint_angles.leg_front_right.position_reachable &= (-135.0 <= j1_angle && j1_angle <= 135.0);
  joint_angles.leg_front_right.position_reachable &= (-135.0 <= j2_angle && j2_angle <= 135.0);
  joint_angles.leg_front_right.position_reachable &= (-135.0 <= j3_angle && j3_angle <= 135.0);

  // Front Left
  ik_status = InverseKinematics(
    leg_positions.leg_front_left.x + JOINT_1_X_OFFSET,
    leg_positions.leg_front_left.y - JOINT_1_Y_OFFSET,
    leg_positions.leg_front_left.z - JOINT_1_Z_OFFSET,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    1.0, -1.0,
    j1_angle, j2_angle, j3_angle);
  joint_angles.leg_front_left.joint1_angle = j1_angle;
  joint_angles.leg_front_left.joint2_angle = j2_angle;
  joint_angles.leg_front_left.joint3_angle = j3_angle;
  joint_angles.leg_front_left.position_reachable = (ik_status == 0);
  joint_angles.leg_front_left.position_reachable &= (-135.0 <= j1_angle && j1_angle <= 135.0);
  joint_angles.leg_front_left.position_reachable &= (-135.0 <= j2_angle && j2_angle <= 135.0);
  joint_angles.leg_front_left.position_reachable &= (-135.0 <= j3_angle && j3_angle <= 135.0);

  // Rear Left
  ik_status = InverseKinematics(
    leg_positions.leg_rear_left.x + JOINT_1_X_OFFSET,
    leg_positions.leg_rear_left.y + JOINT_1_Y_OFFSET,
    leg_positions.leg_rear_left.z - JOINT_1_Z_OFFSET,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    1.0, 1.0,
    j1_angle, j2_angle, j3_angle);
  joint_angles.leg_rear_left.joint1_angle = j1_angle;
  joint_angles.leg_rear_left.joint2_angle = j2_angle;
  joint_angles.leg_rear_left.joint3_angle = j3_angle;
  joint_angles.leg_rear_left.position_reachable = (ik_status == 0);
  joint_angles.leg_rear_left.position_reachable &= (-135.0 <= j1_angle && j1_angle <= 135.0);
  joint_angles.leg_rear_left.position_reachable &= (-135.0 <= j2_angle && j2_angle <= 135.0);
  joint_angles.leg_rear_left.position_reachable &= (-135.0 <= j3_angle && j3_angle <= 135.0);

  // Rear Right
  ik_status = InverseKinematics(
    leg_positions.leg_rear_right.x - JOINT_1_X_OFFSET,
    leg_positions.leg_rear_right.y + JOINT_1_Y_OFFSET,
    leg_positions.leg_rear_right.z - JOINT_1_Z_OFFSET,
    JOINT_1_PLANE_OFFSET, LINKAGE_2_LENGTH, LINKAGE_3_LENGTH,
    -1.0, 1.0,
    j1_angle, j2_angle, j3_angle);
  joint_angles.leg_rear_right.joint1_angle = j1_angle;
  joint_angles.leg_rear_right.joint2_angle = j2_angle;
  joint_angles.leg_rear_right.joint3_angle = j3_angle;
  joint_angles.leg_rear_right.position_reachable = (ik_status == 0);
  joint_angles.leg_rear_right.position_reachable &= (-135.0 <= j1_angle && j1_angle <= 135.0);
  joint_angles.leg_rear_right.position_reachable &= (-135.0 <= j2_angle && j2_angle <= 135.0);
  joint_angles.leg_rear_right.position_reachable &= (-135.0 <= j3_angle && j3_angle <= 135.0);

  return joint_angles;
}
