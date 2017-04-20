#include <cob_follow_me/cob_follow_me.h>

bool FollowMe::initialize(){

    ros::NodeHandle nh_tracker("frame_tracker");

    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    if (!nh_.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set");
        return false;
    }

    if (!nh_.getParam("root_frame", root_frame_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set");
        return false;
    }

    if (!nh_.getParam("update_rate", update_rate_))
    {
        update_rate_ = 2.0;  // hz
    }

    /// Private Nodehandle
    if (!nh_tracker.getParam("target_frame", target_frame_))
    {
        ROS_WARN("Parameter 'target_frame' not set. Using default 'cartesian_target'");
        target_frame_ = DEFAULT_CARTESIAN_TARGET;
    }
    target_frame_ = DEFAULT_CARTESIAN_TARGET;
    ROS_WARN("Waiting for Services...");
    start_tracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
    stop_tracking_ = nh_.serviceClient<std_srvs::Trigger>("frame_tracker/stop");
    start_tracking_.waitForExistence();
    tracking_ = false;
    tracking_frame_ = chain_tip_link_;

    bool transform_available = false;
    while (!transform_available)
    {
        try
        {
            tf_listener_.lookupTransform(root_frame_, tracking_frame_, ros::Time(), target_pose_);
            transform_available = true;
        }
        catch (tf::TransformException& ex)
        {
            // ROS_WARN("IFT::initialize: Waiting for transform...(%s)",ex.what());
            ros::Duration(0.1).sleep();
        }
    }

    ROS_INFO("Following...");

    return true;
};

bool FollowMe::startTracking()
{
    bool success = false;
    cob_srvs::SetString start;
    start.request.data = target_frame_;
    if (!tracking_)
    {
        success = start_tracking_.call(start);

        if (success)
        {
            success = start.response.success;
            if (success)
            {
                ROS_INFO("Response 'start_tracking': succeded");
                tracking_ = true;
            }
            else
            {
                ROS_ERROR("Response 'start_tracking': failed");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service 'start_tracking'");
        }
    }
    else
    {
        ROS_WARN("Already tracking");
    }

    return success;
};

bool FollowMe::stopTracking()
{
    bool success = false;
    std_srvs::Trigger stop;
    if (tracking_)
    {
        success = stop_tracking_.call(stop);

        if (success)
        {
            ROS_INFO("Service 'stop' succeded!");
            tracking_ = false;
        }
        else
        {
            ROS_ERROR("Failed to call service 'stop_tracking'");
        }
    }
    else
    {
        ROS_WARN("Have not been tracking");
    }

    return success;
}

void FollowMe::WrenchCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{

}

void FollowMe::goalCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    // helper variables

    ros::Rate rate(1);
    bool success = true;

    // initially broadcast target_frame
    tf::Transform identity = tf::Transform();
    identity.setIdentity();
    tf_broadcaster_.sendTransform(tf::StampedTransform(identity, ros::Time::now(), root_frame_, target_frame_));

    // Interpolate path

    acceptGoal(goal->trajectory);

    //ADD INTERPOLATION


    if (cartesian_path_->poses.size()<1)
    {
        actionAbort(false, "Failed to obtain Cartesian trajectory");
        return;
    }
    // Publish Preview
    utils_.previewPath(*cartesian_path_);
    ROS_INFO("PREVIEWING THE TRACJEYTORY");
    sleep(10);
    // Activate Tracking
    if (!startTracking())
    {
        actionAbort(false, "Failed to start tracking");
        return;
    }

    // Execute path
    if (!posePathBroadcaster(*cartesian_path_))
    {
        actionAbort(false, "Failed to execute path for 'move_lin'");
        return;
    }

    // De-Activate Tracking
    if (!stopTracking())
    {
        actionAbort(false, "Failed to stop tracking");
        return;
    }


    sleep(5);
    stopTracking();

    return;

 }

 // Broadcasting interpolated Cartesian path
 bool FollowMe::posePathBroadcaster(const geometry_msgs::PoseArray cartesian_path)
 {
     bool success = true;
     ros::Rate rate(update_rate_);
     tf::Transform transform;

     if (!as_.isActive())
     {
         success = false;
     }

     std::vector<double> joint_values;
     for(int i=0;i<cartesian_path.poses.size();i++){


             // Send/Refresh target Frame
         transform.setOrigin(tf::Vector3(cartesian_path.poses[i].position.x,
                                         cartesian_path.poses[i].position.y,
                                         cartesian_path.poses[i].position.z));

         transform.setRotation(tf::Quaternion(cartesian_path.poses[i].orientation.x,
                                              cartesian_path.poses[i].orientation.y,
                                              cartesian_path.poses[i].orientation.z,
                                              cartesian_path.poses[i].orientation.w));

         tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cartesian_path.header.frame_id, target_frame_));

         ros::spinOnce();
         rate.sleep();
     }

     return success;
 }

void FollowMe::acceptGoal(const trajectory_msgs::JointTrajectory trajectory){

 }

 void FollowMe::actionAbort(const bool success, const std::string& message)
 {
     ROS_ERROR_STREAM("Goal aborted: "  << message);
     result_.result.error_code =-1;
     as_.setAborted(result_.result, message);
     stopTracking();
 }

 void FollowMe::actionSuccess(const bool success, const std::string& message)
 {
     ROS_INFO_STREAM("Goal succeeded: " << message);
     result_.result.error_code = 0; //SUCCSSESS
     result_.status.status = 3; //SUCCSSESS
     as_.setSucceeded(result_.result, message);
 }
