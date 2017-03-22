#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetString.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <cob_follow_trajectory_interface/follow_trajectory_utils.h>

#define DEFAULT_CARTESIAN_TARGET "arm_target2"


class FollowTrajectoryInterface
{
protected:

  ros::NodeHandle nh_;
  std::string action_name_;
  std::string action_ns_;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryActionFeedback feedback_;
  control_msgs::FollowJointTrajectoryActionResult result_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform target_pose_;

  ros::ServiceClient start_tracking_;
  std::string tracking_frame_;    // the frame tracking the target
  ros::ServiceClient stop_tracking_;
  bool tracking_;

  double update_rate_;
  std::string root_frame_, chain_tip_link_, target_frame_,chain_base_link_;

  //MoveIt
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;

  FollowTrajectoryUtils utils_;
  boost::shared_ptr<geometry_msgs::PoseArray> cartesian_path_;

public:

  FollowTrajectoryInterface(std::string name) :
    robot_model_loader("robot_description"),
    as_(nh_, action_ns_, boost::bind(&FollowTrajectoryInterface::goalCallback, this, _1 ),false),
    action_name_(name),
    action_ns_(name)
  {
      /*as_.registerGoalCallback(boost::bind(&FollowTrajectoryInterface::goalCallback, this));
      as_.registerPreemptCallback(boost::bind(&FollowTrajectoryInterface::preemptCallback, this));*/
      as_.start();
      cartesian_path_.reset(new geometry_msgs::PoseArray);

  }

  ~FollowTrajectoryInterface(void)
  {
  }

  void goalCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  void acceptGoal(const trajectory_msgs::JointTrajectory trajectory);

  bool preemptCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  void actionAbort(const bool success, const std::string& message);

  void actionSuccess(const bool success, const std::string& message);

  bool initialize();

  bool posePathBroadcaster(const geometry_msgs::PoseArray cartesian_path);

  bool startTracking();

  bool stopTracking();

};

