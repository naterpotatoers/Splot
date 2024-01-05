/**
 * kinematics_lib.hpp
*/


#ifndef KINEMATICS_LIB_HPP_
#define KINEMATICS_LIB_HPP_


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
                       double &x, double &y);


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
  double &x, double &y, double &z);


/**
 * InverseKinematics
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
  double &j1_angle, double &j2_angle, double &j3_angle);


#endif
