#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetString.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>

#include <cob_follow_me/follow_trajectory_utils.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#define DEFAULT_CARTESIAN_TARGET "arm_target2"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class FollowMe
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

  ros::Subscriber wrench_sub;

  ros::ServiceClient start_tracking_;
  ros::ServiceClient get_planning_scene_;
  std::string tracking_frame_;    // the frame tracking the target
  ros::ServiceClient stop_tracking_;
  bool tracking_;

  double update_rate_;
  std::string root_frame_, chain_tip_link_, target_frame_,chain_base_link_;

  FollowTrajectoryUtils utils_;
  boost::shared_ptr<geometry_msgs::PoseArray> cartesian_path_;

public:

  FollowMe(std::string name) :
      as_(nh_, action_ns_, boost::bind(&FollowMe::goalCallback, this, _1 ),false),
      action_name_(name),
      action_ns_(name)
  {
      /*as_.registerGoalCallback(boost::bind(&FollowTrajectoryInterface::goalCallback, this));
      as_.registerPreemptCallback(boost::bind(&FollowTrajectoryInterface::preemptCallback, this));*/
      as_.start();
      cartesian_path_.reset(new geometry_msgs::PoseArray);
      wrench_sub=nh_.subscribe("wrench", 1, &FollowMe::WrenchCallback,this);

  }

  ~FollowMe(void)
  {
  }

  void goalCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  void acceptGoal(const trajectory_msgs::JointTrajectory trajectory);

  bool preemptCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  void actionAbort(const bool success, const std::string& message);

  void actionSuccess(const bool success, const std::string& message);

  void WrenchCallback(const geometry_msgs::WrenchStampedPtr & goal);

  bool initialize();

  bool posePathBroadcaster(const geometry_msgs::PoseArray cartesian_path);

  bool startTracking();

  bool stopTracking();

  bool getTransform(const std::string& target_frame, const std::string& source_frame, Eigen::Affine3d& T);

  bool setTransform(const std::string& target_frame, const std::string& source_frame, Eigen::Vector3d& in, Eigen::Vector3d& out);

};

