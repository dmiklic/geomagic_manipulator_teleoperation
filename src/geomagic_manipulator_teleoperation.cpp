
#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <omni_msgs/OmniButtonEvent.h>

using std::string;

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

  std_msgs::Int32MultiArray gripper_msg_;
  
};

GeomagicManipulatorTeleoperation::GeomagicManipulatorTeleoperation()
  : nh_private_("~")
{
  // Get parameters from the parameter server
  //nh_private_.param<double>("wheelbase", L_, 1.0);
  
  sub_pose_ = nh_.subscribe("geomagic_pose", 1, &GeomagicManipulatorTeleoperation::receivePose, this);
  sub_button_ = nh_.subscribe("geomagic_button", 1, &GeomagicManipulatorTeleoperation::receiveButton, this);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
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
  pub_pose_.publish(pose);
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
