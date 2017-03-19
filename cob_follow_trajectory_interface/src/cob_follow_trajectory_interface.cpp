#include <cob_follow_trajectory_interface/cob_follow_trajectory_interface.h>

bool FollowTrajectoryInterface::initialize(){

    if (!nh_.getParam("/arm/chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    if (!nh_.getParam("/arm/chain_base_link", root_frame_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set");
        return false;
    }
    if (!nh_.getParam("update_rate", update_rate_))
    {
        update_rate_ = 50.0;  // hz
    }

    /// Private Nodehandle
    if (!nh_.getParam("target_frame", target_frame_))
    {
        ROS_WARN("Parameter 'target_frame' not set. Using default 'cartesian_target'");
        target_frame_ = DEFAULT_CARTESIAN_TARGET;
    }

    ROS_WARN("Waiting for Services...");
    start_tracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
    stop_tracking_ = nh_.serviceClient<std_srvs::Trigger>("frame_tracker/stop");
    start_tracking_.waitForExistence();
    tracking_ = false;

    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    ROS_INFO("Follow Trajectory Interface running");

    return true;
};

bool FollowTrajectoryInterface::startTracking()
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

bool FollowTrajectoryInterface::stopTracking()
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

void FollowTrajectoryInterface::goalCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    // helper variables

    ros::Rate rate(1);
    bool success = true;

    // initially broadcast target_frame
    tf::Transform identity = tf::Transform();
    identity.setIdentity();
    tf_broadcaster_.sendTransform(tf::StampedTransform(identity, ros::Time::now(), chain_tip_link_, target_frame_));

    // Activate Tracking
    if (!startTracking())
    {
        actionAbort(false, "Failed to start tracking");
        return;
    }

    // Execute path
    if (!posePathBroadcaster(goal->trajectory))
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
 bool FollowTrajectoryInterface::posePathBroadcaster(const trajectory_msgs::JointTrajectory trajectory)
 {
     bool success = true;
     ros::Rate rate(update_rate_);
     tf::Transform transform;

     robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

     const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

     const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

     if (!as_.isActive())
     {
         success = false;
     }

     std::vector<double> joint_values;
     for(int i=0;i<trajectory.points.size();i++){

         kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
         joint_values=trajectory.points[i].positions;

         // publish info to the console for the user
         ROS_INFO_STREAM("Actual point"<< trajectory.points[0]);
         // start executing the action
         kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
         const Eigen::Affine3d &end_effector_state = kinematic_state-> getGlobalLinkTransform(chain_tip_link_);

         /* Print end-effector pose. Remember that this is in the model frame */
         ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
         ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
         ROS_INFO_STREAM("Number of points:"<<trajectory.points.size());
         startTracking();

         geometry_msgs::Pose go;
         go.position.x=end_effector_state.translation()[0];
         go.position.y=end_effector_state.translation()[1];
         go.position.z=end_effector_state.translation()[2];
         Eigen::Quaterniond q(end_effector_state.rotation());
         go.orientation.w=q.w();
         go.orientation.x=q.x();
         go.orientation.y=q.y();
         go.orientation.z=q.z();
             // Send/Refresh target Frame
         transform.setOrigin(tf::Vector3(end_effector_state.translation()[0],
                                         end_effector_state.translation()[1],
                                         end_effector_state.translation()[2]));

         transform.setRotation(tf::Quaternion(q.w(),q.x(),q.y(),q.z()));

         ROS_INFO_STREAM("PUblish Translation: " << end_effector_state.translation()[0] << end_effector_state.translation()[1] << end_effector_state.translation()[2]);
         ROS_INFO_STREAM("PUblish Rotation: " << q.w() << q.x() << q.y() << q.z());
         ROS_INFO_STREAM("ROOT_FRAME:"<<root_frame_);
         ROS_INFO_STREAM("TARGET_FRAME:"<<target_frame_);
         tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", target_frame_));

         /*Eigen::Quaterniond q(end_effector_state.rotation());
         * tf::Transform transform;
         transform.setOrigin(tf::Vector3(end_effector_state.translation()[0],
                                          end_effector_state.translation()[1],
                                          end_effector_state.translation()[2]));
         transform.setRotation(tf::Quaternion(q.w(),q.x(),q.y(),q.z()));

         tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "gripper"));

         geometry_msgs::Pose go;
         go.position.x=end_effector_state.translation()[0];
         go.position.y=end_effector_state.translation()[1];
         go.position.z=end_effector_state.translation()[2];
         Eigen::Quaterniond q(end_effector_state.rotation());
         go.orientation.w=q.w();
         go.orientation.x=q.x();
         go.orientation.y=q.y();
         go.orientation.z=q.z();
         */

         ros::spinOnce();
         rate.sleep();
     }

     return success;
 }

 void FollowTrajectoryInterface::actionAbort(const bool success, const std::string& message)
 {
     ROS_ERROR_STREAM("Goal aborted: "  << message);
     result_.result.error_code =-1;
     as_.setAborted(result_.result, message);
     stopTracking();
 }

 void FollowTrajectoryInterface::actionSuccess(const bool success, const std::string& message)
 {
     ROS_INFO_STREAM("Goal succeeded: " << message);
     result_.result.error_code = 0; //SUCCSSESS
     result_.status.status = 3; //SUCCSSESS
     as_.setSucceeded(result_.result, message);
 }
