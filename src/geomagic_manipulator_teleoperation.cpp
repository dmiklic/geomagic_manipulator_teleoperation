
#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <omni_msgs/OmniButtonEvent.h>

using std::string;

//! Clamp value
template <typename T>
T clamp(const T&x, const T& lower, const T& upper)
{
  return std::max(lower, std::min(x, upper));
}

class GeomagicManipulatorTeleoperation
{
public:

  //! Constructor
  GeomagicManipulatorTeleoperation();

  //! Receive path to follow.
  void receivePose(geometry_msgs::PoseStamped pose);
  void receiveButton(omni_msgs::OmniButtonEvent button);

  //! Run the controller.
  void run();
  
private:
  
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_pose_, sub_button_;
  ros::Publisher pub_pose_, pub_gripper_;

  double x_offset_, y_offset_, z_offset_;
  double x_scale_, y_scale_, z_scale_;
  double x_min_, y_min_, z_min_;
  double x_max_, y_max_, z_max_;
  std_msgs::Int32MultiArray gripper_msg_;
  
};

GeomagicManipulatorTeleoperation::GeomagicManipulatorTeleoperation()
  : nh_private_("~")
{
  // Get parameters from the parameter server
  nh_private_.param<double>("x_offset", x_offset_, 1.0);
  nh_private_.param<double>("y_offset", y_offset_, 1.0);
  nh_private_.param<double>("z_offset", z_offset_, 1.0);
  nh_private_.param<double>("x_scale", x_scale_, 1.0);
  nh_private_.param<double>("y_scale", y_scale_, 1.0);
  nh_private_.param<double>("z_scale", z_scale_, 1.0);
  nh_private_.param<double>("x_min", x_min_, 1.0);
  nh_private_.param<double>("y_min", y_min_, 1.0);
  nh_private_.param<double>("z_min", z_min_, 1.0);
  nh_private_.param<double>("x_max", x_max_, 1.0);
  nh_private_.param<double>("y_max", y_max_, 1.0);
  nh_private_.param<double>("z_max", z_max_, 1.0);
  
  sub_pose_ = nh_.subscribe("geomagic_pose", 1, &GeomagicManipulatorTeleoperation::receivePose, this);
  sub_button_ = nh_.subscribe("geomagic_button", 1, &GeomagicManipulatorTeleoperation::receiveButton, this);
  pub_pose_ = nh_.advertise<geometry_msgs::Pose>("robot_pose", 1);
  pub_gripper_ = nh_.advertise<std_msgs::Int32MultiArray>("robot_gripper", 1);

  // Initialize gripper message
  gripper_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
  gripper_msg_.layout.dim[0].size = 6;
  gripper_msg_.layout.dim[0].stride = 1;
  gripper_msg_.layout.dim[0].label = "gripper";
  gripper_msg_.data.clear();
  gripper_msg_.data.insert(gripper_msg_.data.end(), 6, 0.0);
}

void GeomagicManipulatorTeleoperation::receivePose(geometry_msgs::PoseStamped pose)
{
  geometry_msgs::Pose robot_pose(pose.pose);
  robot_pose.position.x = clamp(x_scale_ * (robot_pose.position.x + x_offset_), x_min_, x_max_);
  robot_pose.position.y = clamp(y_scale_ * (robot_pose.position.y + y_offset_), y_min_, y_max_);
  robot_pose.position.z = clamp(z_scale_ * (robot_pose.position.z + z_offset_), z_min_, z_max_);
  pub_pose_.publish(robot_pose);
}

void GeomagicManipulatorTeleoperation::receiveButton(omni_msgs::OmniButtonEvent button)
{
  if (button.grey_button != button.white_button)
  {
    if (button.grey_button > 0 && gripper_msg_.data[1] != 1)
    {
      gripper_msg_.data[1] = 1;
      pub_gripper_.publish(gripper_msg_);
    }
    else if (button.white_button > 0 && gripper_msg_.data[1] != 2)
    {
      gripper_msg_.data[1] = 2;
      pub_gripper_.publish(gripper_msg_);
    }
  }
}

void GeomagicManipulatorTeleoperation::run()
{
  ros::spin();
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "geomagic_manipulator_teleoperation");

  GeomagicManipulatorTeleoperation teleop;
  teleop.run();

  return 0;
}
