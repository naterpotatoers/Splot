#include <chrono>
#include <memory>


#include <rclcpp/rclcpp.hpp>
#include "servo_interfaces/msg/quadruped_joint_angles.hpp"
#include "servo_interfaces/msg/quadruped_leg_positions.hpp"
#include "servo_interfaces/srv/quadruped_forward_kinematics.hpp"
#include "servo_interfaces/srv/quadruped_inverse_kinematics.hpp"
#include "quadruped_kinematics/quadruped_kinematics_lib.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace servo_interfaces::msg;
using namespace servo_interfaces::srv;


class QuadrupedKinematics : public rclcpp::Node
{
  public:
    QuadrupedKinematics();

  private:
    void leg_positions_callback(
      const QuadrupedLegPositions& leg_positions_msg) const;
    void forward_kinematics_callback(
      const std::shared_ptr<QuadrupedForwardKinematics::Request> request,
      std::shared_ptr<QuadrupedForwardKinematics::Response> response) const;
    void inverse_kinematics_callback(
      const std::shared_ptr<QuadrupedInverseKinematics::Request> request,
      std::shared_ptr<QuadrupedInverseKinematics::Response> response) const;

    rclcpp::Subscription<QuadrupedLegPositions>::SharedPtr subscription_;
    rclcpp::Publisher<QuadrupedJointAngles>::SharedPtr publisher_;
    rclcpp::Service<QuadrupedForwardKinematics>::SharedPtr fk_service_;
    rclcpp::Service<QuadrupedInverseKinematics>::SharedPtr ik_service_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadrupedKinematics>());
  rclcpp::shutdown();

  return 0;
}


QuadrupedKinematics::QuadrupedKinematics()
: Node("quadruped_kinematics")
{
  subscription_ = this->create_subscription<QuadrupedLegPositions>(
    "leg_positions", 10, std::bind(&QuadrupedKinematics::leg_positions_callback, this, _1));
  publisher_ = this->create_publisher<QuadrupedJointAngles>("joint_angles", 10);
  fk_service_ = this->create_service<QuadrupedForwardKinematics>(
    "forward_kinematics", std::bind(&QuadrupedKinematics::forward_kinematics_callback, this, _1, _2));
  ik_service_ = this->create_service<QuadrupedInverseKinematics>(
    "inverse_kinematics", std::bind(&QuadrupedKinematics::inverse_kinematics_callback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "quadruped_kinematics running");
}


void QuadrupedKinematics::leg_positions_callback(const QuadrupedLegPositions& leg_positions_msg) const
{
  // RCLCPP_INFO(this->get_logger(), "Leg positions received:");
  // RCLCPP_INFO(this->get_logger(), "\tFront right: < %.2f, %.2f, %.2f >",
  //             leg_positions_msg.leg_front_right.x,
  //             leg_positions_msg.leg_front_right.y,
  //             leg_positions_msg.leg_front_right.z);
  // RCLCPP_INFO(this->get_logger(), "\tFront left: < %.2f, %.2f, %.2f >",
  //             leg_positions_msg.leg_front_left.x,
  //             leg_positions_msg.leg_front_left.y,
  //             leg_positions_msg.leg_front_left.z);
  // RCLCPP_INFO(this->get_logger(), "\tRear left: < %.2f, %.2f, %.2f >",
  //             leg_positions_msg.leg_rear_left.x,
  //             leg_positions_msg.leg_rear_left.y,
  //             leg_positions_msg.leg_rear_left.z);
  // RCLCPP_INFO(this->get_logger(), "\tRear right: < %.2f, %.2f, %.2f >",
  //             leg_positions_msg.leg_rear_right.x,
  //             leg_positions_msg.leg_rear_right.y,
  //             leg_positions_msg.leg_rear_right.z);

  auto joint_angles_msg = apply_inverse_kinematics(leg_positions_msg);
  bool do_publish = true;
  
  // RCLCPP_INFO(this->get_logger(), "Joint angles calculated:");
  // RCLCPP_INFO(this->get_logger(), "\tFront right: < %.2f, %.2f, %.2f >",
  //             joint_angles_msg.leg_front_right.joint1_angle,
  //             joint_angles_msg.leg_front_right.joint2_angle,
  //             joint_angles_msg.leg_front_right.joint3_angle);
  if (!joint_angles_msg.leg_front_right.position_reachable) {
    do_publish = false;
    RCLCPP_INFO(this->get_logger(), "\tFront right target position unreachable");
  }
  // RCLCPP_INFO(this->get_logger(), "\tFront right: < %.2f, %.2f, %.2f >",
  //             joint_angles_msg.leg_front_left.joint1_angle,
  //             joint_angles_msg.leg_front_left.joint2_angle,
  //             joint_angles_msg.leg_front_left.joint3_angle);
  if (!joint_angles_msg.leg_front_left.position_reachable) {
    do_publish = false;
    RCLCPP_INFO(this->get_logger(), "\tFront left target position unreachable");
  }
  // RCLCPP_INFO(this->get_logger(), "\tFront right: < %.2f, %.2f, %.2f >",
  //             joint_angles_msg.leg_rear_left.joint1_angle,
  //             joint_angles_msg.leg_rear_left.joint2_angle,
  //             joint_angles_msg.leg_rear_left.joint3_angle);
  if (!joint_angles_msg.leg_rear_left.position_reachable) {
    do_publish = false;
    RCLCPP_INFO(this->get_logger(), "\tRear left target position unreachable");
  }
  // RCLCPP_INFO(this->get_logger(), "\tFront right: < %.2f, %.2f, %.2f >",
  //             joint_angles_msg.leg_rear_right.joint1_angle,
  //             joint_angles_msg.leg_rear_right.joint2_angle,
  //             joint_angles_msg.leg_rear_right.joint3_angle);
  if (!joint_angles_msg.leg_rear_right.position_reachable) {
    do_publish = false;
    RCLCPP_INFO(this->get_logger(), "\tRear right target position unreachable");
  }

  if (do_publish) {
    this->publisher_->publish(joint_angles_msg);
  }
}


void QuadrupedKinematics::forward_kinematics_callback(
  const std::shared_ptr<QuadrupedForwardKinematics::Request> request,
  std::shared_ptr<QuadrupedForwardKinematics::Response> response) const
{
  response->leg_positions = apply_forward_kinematics(request->joint_angles);
}


void QuadrupedKinematics::inverse_kinematics_callback(
  const std::shared_ptr<QuadrupedInverseKinematics::Request> request,
  std::shared_ptr<QuadrupedInverseKinematics::Response> response) const
{
  response->joint_angles = apply_inverse_kinematics(request->leg_positions);
}
