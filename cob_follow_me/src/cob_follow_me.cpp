#include <cob_follow_me/cob_follow_me.h>

bool FollowMe::initialize(){

    ros::NodeHandle nh_tracker("frame_tracker");

    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set. Using default 'arm_ee_link'");
        chain_tip_link_="arm_ee_link";
    }

    if (!nh_.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set. Using default 'arm_base_link'");
        chain_base_link_="arm_base_link";
    }

    if (!nh_.getParam("root_frame", root_frame_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set Using default 'odom_combined' ");
        root_frame_= "odom_combined";
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
    //start_tracking_.waitForExistence();
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

    /// publisher for visualizing current twist direction
    force_direction_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("force_direction", 1);

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

void FollowMe::WrenchCallback(const geometry_msgs::WrenchStampedPtr &goal)
{
    ROS_INFO_STREAM("Force values received" << " : " << goal->wrench.force);

    ROS_INFO_STREAM("Torque values received" << " : " << goal->wrench.torque);

    Vector6d wrench;
    tf::wrenchMsgToEigen(goal->wrench, wrench);
    Eigen::Vector3d force;
    force << wrench(0), wrench(1), wrench(2);
    Eigen::Vector3d torque;
    torque << wrench(3), wrench(4), wrench(5);
    Eigen::Vector3d force_rotated;
    Eigen::Vector3d torque_rotated;

    bool success = setTransform(chain_tip_link_,root_frame_,force,force_rotated);

    //success = setTransform(chain_tip_link_,root_frame_,torque,torque_rotated); //Data alredy comes in iertial frame. not needed to be converted

    KDL::Wrench wrench2;

    wrench2.force.x(force[0]);
    wrench2.force.y(force[1]);
    wrench2.force.z(force[2]);

    visualizeForce(wrench2);

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

bool FollowMe::getTransform(const std::string& target_frame, const std::string& source_frame, Eigen::Affine3d& T)
{
      try
      {
          tf::StampedTransform Ts;
          tf_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
          tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);
          tf::transformTFToEigen(Ts, T);
          //std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << T << std::endl;
      }
      catch (tf::TransformException& ex)
      {
          ROS_WARN("%s",ex.what());
          return false;
      }

      return true;
}

bool FollowMe::setTransform(const std::string& target_frame, const std::string& source_frame, Eigen::Vector3d& in,Eigen::Vector3d& out)
{
    try
    {
        tf::StampedTransform Ts;
        Eigen::Affine3d T;
        tf_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), Ts);
        tf::transformTFToEigen(Ts, T);
        out=T.rotation()*in;
        //std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n" << T << std::endl;
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("%s",ex.what());
        return false;
    }

    return true;
}

void FollowMe::visualizeForce(KDL::Wrench wrench)
{
    visualization_msgs::Marker marker_force;
    marker_force.header.frame_id = root_frame_;
    marker_force.header.stamp = ros::Time::now();
    marker_force.ns = "force_dir";
    marker_force.id = 0;
    marker_force.type = visualization_msgs::Marker::ARROW;
    marker_force.action = visualization_msgs::Marker::ADD;
    marker_force.lifetime = ros::Duration(0.1);
    marker_force.pose.orientation.w = 1.0;

    marker_force.scale.x = 0.02;
    marker_force.scale.y = 0.02;
    marker_force.scale.z = 0.02;

    marker_force.color.r = 1.0f;
    marker_force.color.g = 1.0f;
    marker_force.color.b = 0.0f;
    marker_force.color.a = 1.0;

    marker_force.points.resize(2);
    marker_force.points[0].x = wrench.force.x();
    marker_force.points[0].y = wrench.force.y();
    marker_force.points[0].z = wrench.force.z();

    visualization_msgs::Marker marker_rot;
    marker_rot.header.frame_id =  root_frame_;
    marker_rot.header.stamp = ros::Time::now();
    marker_rot.ns = "torque_rot";
    marker_rot.id = 0;
    marker_rot.type = visualization_msgs::Marker::CYLINDER;
    marker_rot.action = visualization_msgs::Marker::ADD;
    marker_rot.lifetime = ros::Duration(0.1);
    marker_rot.pose.position.x = wrench.torque.x();
    marker_rot.pose.position.y = wrench.torque.y();
    marker_rot.pose.position.z = wrench.torque.z();

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker_force);

    force_direction_pub_.publish(markers);
}
