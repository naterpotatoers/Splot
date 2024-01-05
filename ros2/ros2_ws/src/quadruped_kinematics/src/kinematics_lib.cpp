/**
 * kinematics_lib.cpp
*/


#include "quadruped_kinematics/kinematics_lib.hpp"
#include <cmath>

//#include <stdio.h>

#define TRIPED_PI 3.14159265359f
#define TRIPED_PI_over_2 1.57079632679f
#define TRIPED_PI_times_2 6.28318530718f


/**
 * CircleIntersection
 * 
 * If the circles do not intersect, x and y are unmodified. If there is exactly
 * one point of intersection, x and y specify that intersection regardless of
 * configuration.
 * 
 * Input values:
 *   c1_x: double: x component of center of circle 1
 *   c1_y: double: y component of center of circle 1
 *   r1: double: radius of circle 1
 *   c2_x: double: x component of center of circle 2
 *   c2_y: double: y component of center of circle 2
 *   r2: double: radius of circle 2
 *   configuration: double: +1 chooses < x, y > such that the sequence
 *                         [c1, < x, y >, c2] is a counter-clockwise angle.
 *                         -1 chooses < x, y > such that the sequence is a
 *                         clockwise angle.
 * 
 * Returns: int: number of intersections [0, 1, 2]
*/
int CircleIntersection(double c1_x, double c1_y, double r1,
                       double c2_x, double c2_y, double r2,
                       double configuration,
                       double &x, double &y) {
  double c_x_distance = c1_x - c2_x;
  double c_y_distance = c1_y - c2_y;
  double inv_sq_center_distance = 1.0f / (c_x_distance * c_x_distance +
                                          c_y_distance * c_y_distance);
  double r1_sq_plus_r2_sq = r1 * r1 + r2 * r2;
  double r1_sq_minus_r2_sq = r1 * r1 - r2 * r2;
  double factor_1 = 0.5f * r1_sq_minus_r2_sq * inv_sq_center_distance;
  double factor_2_a = 2.0f * r1_sq_plus_r2_sq * inv_sq_center_distance;
  double factor_2_b = r1_sq_minus_r2_sq * inv_sq_center_distance;
  factor_2_b *= factor_2_b;
  double factor_2_discriminant = factor_2_a - factor_2_b - 1.0f;
  if (factor_2_discriminant < 0.0f) {
    return 0;
  }
//  printf("CircleIntersection: factor_1 = %f\n", factor_1);
//  printf("CircleIntersection: < c1_x, c1_y > = < %f , %f >\n", c1_x, c1_y);
//  printf("CircleIntersection: < c2_x, c2_y > = < %f , %f >\n", c2_x, c2_y);
  x = 0.5f * (c1_x + c2_x) + factor_1 * (c2_x - c1_x);
  y = 0.5f * (c1_y + c2_y) + factor_1 * (c2_y - c1_y);
  double factor_2;
  if (factor_2_discriminant > 0.0f) {
    factor_2 = configuration * 0.5f * sqrt(factor_2_discriminant);
    x += factor_2 * (c2_y - c1_y);
    y += factor_2 * (c1_x - c2_x);
//    printf("CircleIntersection: < x, y > = < %f , %f >\n", x, y);
    return 2;
  }
//  printf("CircleIntersection: < x, y > = < %f , %f >\n", x, y);
  return 1;
}


/**
 * ForwardKinematics
 * 
 * Target TCP is specified relative to the intersection of the J1 axis and its
 * normal plane that contains the J2 axis. +x points to the right side of the
 * robot; +y points in the direction the robot is facing; +z points in the
 * direction projecting "upward" from the robot's top/dorsal plane.
 * 
 * Input values:
 *   j1_angle: double: angle in degrees of J1
 *   j2_angle: double: angle in degrees of J2
 *   j3_angle: double: angle in degrees of J3
 *   joint_1_plane_offset: double: parallel distance between J1 axis and
 *                                the plane defined by J2 and J3
 *   linkage_2_length: double: parallel distance between J2 axis and J3 axis
 *   linkage_3_length: double: distance between J3 axis and TCP
 *   j1_config: double: +1: ... -1: ...
 * 
 * Output parameters:
 *   x: double: x component of TCP position
 *   y: double: y component of TCP position
 *   z: double: z component of TCP position
*/
void ForwardKinematics(
    double j1_angle, double j2_angle, double j3_angle,
    double joint_1_plane_offset, double linkage_2_length, double linkage_3_length,
    double j1_config,
    double &x, double &y, double &z) {
  double j2_x, j2_y, j2_z;
  double j3_plane_x, j3_plane_y;
  double tcp_plane_x, tcp_plane_y;

  j2_x = -j1_config * joint_1_plane_offset * cos(j1_angle);
  j2_y = 0.0;
  j2_z = joint_1_plane_offset * sin(j1_angle);

  j3_plane_x = linkage_2_length * sin(j2_angle);
  j3_plane_y = linkage_2_length * -cos(j2_angle);

  tcp_plane_x = j3_plane_x + linkage_3_length * sin(j2_angle + j3_angle);
  tcp_plane_y = j3_plane_y + linkage_3_length * -cos(j2_angle + j3_angle);

  x = j2_x + -j1_config * tcp_plane_y * -sin(j1_angle);
  y = j2_y + tcp_plane_x;
  z = j2_z + tcp_plane_y * cos(j1_angle);
}


/**
 * inverseKinematics
 * 
 * Target TCP is specified relative to the intersection of the J1 axis and its
 * normal plane that contains the J2 axis. +x points to the right side of the
 * robot; +y points in the direction the robot is facing; +z points in the
 * direction projecting "upward" from the robot's top/dorsal plane.
 * 
 * Input values:
 *   x: double: x component of target TCP position
 *   y: double: y component of target TCP position
 *   z: double: z component of target TCP position
 *   joint_1_plane_offset: double: parallel distance between J1 axis and
 *                                the plane defined by J2 and J3
 *   linkage_2_length: double: parallel distance between J2 axis and J3 axis
 *   linkage_3_length: double: distance between J3 axis and TCP
 *   j1_config: double: +1: ... -1: ...
 *   j2_config: double: +1: ... -1: ...
 * 
 * Output parameters:
 *   j1_angle: double: angle in degrees of J1
 *   j2_angle: double: angle in degrees of J2
 *   j3_angle: double: angle in degrees of J3
 * 
 * Returns: int: 0 if successful, -1 if target TCP position is unreachable
*/
int InverseKinematics(
    double x, double y, double z,
    double joint_1_plane_offset, double linkage_2_length, double linkage_3_length,
    double j1_config, double j2_config,
    double &j1_angle, double &j2_angle, double &j3_angle) {
  double j1_x, j1_y; // < x, y, z > projected onto J1 normal plane
  double j2_x, j2_y; // < x, y, z > projected onto J2/J3 normal plane
  double j1_temp_leg_length;
  double x_temp, y_temp;
  int num_intersections;
  // Project target point onto J1 normal plane
  j1_x = x;
  j1_y = z;
  if (joint_1_plane_offset > 0.001) {
    // Calculate j1_angle
    j1_temp_leg_length = sqrt(j1_x * j1_x + j1_y * j1_y -
                              joint_1_plane_offset * joint_1_plane_offset);
//  printf("InverseKinematics: j1_temp_leg_length = %f\n", j1_temp_leg_length);
    num_intersections = CircleIntersection(0.0f, 0.0f, joint_1_plane_offset,
                                           j1_x, j1_y, j1_temp_leg_length,
                                           j1_config, x_temp, y_temp);
//  printf("InverseKinematics: j1 num_intersections = %d\n", num_intersections);
//  printf("InverseKinematics: < x_temp, y_temp > = < %f , %f >\n", x_temp, y_temp);
    if (num_intersections == 0) {
      j1_angle = 0.0f;
      j2_angle = 0.0f;
      j3_angle = 0.0f;
      return -1;
    }
  } else {
    x_temp = 0.0;
    y_temp = 0.0;
  }
//  j1_angle = atan2(y_temp, -j1_config * x_temp);
  j1_angle = atan2(j1_y - y_temp, -j1_config * (j1_x - x_temp)) + TRIPED_PI_over_2;
//  printf("InverseKinematics: j1_angle = %f\n", j1_angle);
//  printf("InverseKinematics: < x_temp, y_temp > = < %f , %f >\n", x_temp, y_temp);
  // Project target point onto J2/J3 normal plane
  j2_x = y;
//  j2_y = -j1_config * (-y_temp * x + x_temp * z) / sqrt(x_temp * x_temp + y_temp * y_temp); //sin(-j1_angle) * x + cos(-j1_angle) * z;
  j2_y = -j1_config * (-sin(j1_angle) * x + -j1_config * cos(j1_angle) * z);
//  printf("InverseKinematics: < j2_x, j2_y > = < %f , %f >\n", j2_x, j2_y);
  // Calculate j2_angle and j3_angle
  num_intersections = CircleIntersection(0.0f, 0.0f, linkage_2_length,
                                         j2_x, j2_y, linkage_3_length,
                                         j2_config, x_temp, y_temp);
//  printf("InverseKinematics: j2&3 num_intersections = %d\n", num_intersections);
//  printf("InverseKinematics: < x_temp, y_temp > = < %f , %f >\n", x_temp, y_temp);
  if (num_intersections == 0) {
    j1_angle = 0.0f;
    j2_angle = 0.0f;
    j3_angle = 0.0f;
    return -1;
  }
  j2_angle = atan2(y_temp, x_temp);
  j3_angle = atan2(j2_y - y_temp, j2_x - x_temp) - j2_angle;
  j2_angle += TRIPED_PI_over_2;
  if (j3_angle >= TRIPED_PI) j3_angle -= TRIPED_PI_times_2;
  if (j3_angle <= -TRIPED_PI) j3_angle += TRIPED_PI_times_2;
//  printf("InverseKinematics: j2_angle = %f\n", j2_angle);
//  printf("InverseKinematics: j3_angle = %f\n", j3_angle);

  return 0;
}
