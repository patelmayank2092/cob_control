#include <cob_follow_trajectory_interface/cob_follow_trajectory_interface.h>

bool FollowTrajectoryInterface::initialize(){

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

    get_planning_scene_ = nh_.serviceClient<cob_srvs::SetString>("/get_planning_scene");
    get_planning_scene_.waitForExistence();
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_ptr_(new robot_model_loader::RobotModelLoader("robot_description","robot_description_semantic"));
    kinematic_model = robot_model_loader2.getModel();
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
 bool FollowTrajectoryInterface::posePathBroadcaster(const geometry_msgs::PoseArray cartesian_path)
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

void FollowTrajectoryInterface::acceptGoal(const trajectory_msgs::JointTrajectory trajectory){


    robot_model_loader::RobotModelLoaderPtr robot_model_loader_ptr_(new robot_model_loader::RobotModelLoader("robot_description","robot_description_semantic"));
     kinematic_model = robot_model_loader_ptr_->getModel();

     ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

     robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
     bool joint_model=kinematic_model->hasJointModelGroup("arm");
     std::vector<std::string> group_names = kinematic_model->getJointModelGroupNames();

     for (int i =0; i< group_names.size();i++)
         ROS_INFO_STREAM("Joint groups:"<<group_names[i]);

     if(!joint_model)
         ROS_INFO_STREAM("JOINT MODEL NOT FOUND");

     const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

     const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

     std::vector<double> joint_values;

     (*cartesian_path_).poses.clear();
     cartesian_path_->poses.resize(trajectory.points.size());

     for(int i=0;i<trajectory.points.size();i++){

         joint_values=trajectory.points[i].positions;

         // publish info to the console for the user
         ROS_INFO_STREAM("Actual point"<< trajectory.points[i]);
         // start executing the action

         /*double dumb = joint_values[0];

         joint_values[0]=joint_values[2];

         joint_values[2]=dumb;*/

         kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
         ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

         kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

         kinematic_state->updateLinkTransforms();

         const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(chain_tip_link_);
         for(std::size_t j = 0; j < joint_names.size(); ++j)
          {
            ROS_INFO("Joint %s: %f %f", joint_names[j].c_str(), joint_values[j], trajectory.points[i].positions[j]);
          }

         Eigen::Quaterniond q(end_effector_state.rotation());

         (*cartesian_path_).poses[i].position.x=end_effector_state.translation()[0];
         (*cartesian_path_).poses[i].position.y=end_effector_state.translation()[1];
         (*cartesian_path_).poses[i].position.z=end_effector_state.translation()[2];

         (*cartesian_path_).poses[i].orientation.x=q.x();
         (*cartesian_path_).poses[i].orientation.y=q.y();
         (*cartesian_path_).poses[i].orientation.z=q.z();
         (*cartesian_path_).poses[i].orientation.w=q.w();
         (*cartesian_path_).header.frame_id=trajectory.header.frame_id; //trajectory.header.frame_id chain_base_link_

         /* DEBUG INFO */
         const Eigen::Affine3d &end_effector_state2 = kinematic_state->getFrameTransform(chain_tip_link_); // return the same as previous functio
         geometry_msgs::Pose ef_in;
         ef_in.position.x=end_effector_state.translation()[0];
         ef_in.position.y=end_effector_state.translation()[1];
         ef_in.position.z=end_effector_state.translation()[2];
         ef_in.orientation.w=q.w();
         ef_in.orientation.x=q.x();
         ef_in.orientation.y=q.y();
         ef_in.orientation.z=q.z();
         geometry_msgs::Pose ef_transformed;

         utils_.transformPose(chain_base_link_,root_frame_,ef_in,ef_transformed);
         ROS_INFO_STREAM("arm_base_link to odom_combined: " << ef_transformed.position);

         utils_.transformPose(trajectory.header.frame_id,root_frame_,ef_in,ef_transformed);
         ROS_INFO_STREAM("trajectory.header.frame_id: " << ef_transformed.position);

         utils_.transformPose(chain_tip_link_,root_frame_,ef_in,ef_transformed);
         ROS_INFO_STREAM("chain_tip_link_ to odom_combined: " << ef_transformed.position);


         ROS_INFO_STREAM("PUblish Translation: " << end_effector_state.translation()[0] << end_effector_state.translation()[1] << end_effector_state.translation()[2]);
         ROS_INFO_STREAM("PUblish Rotation: " << q.w() << q.x() << q.y() << q.z());
         ROS_INFO_STREAM("FRAME MOVEIT"<<trajectory.header.frame_id);
         ROS_INFO_STREAM("ROOT_FRAME:"<<root_frame_);
         ROS_INFO_STREAM("TARGET_FRAME:"<<target_frame_);
         ROS_INFO_STREAM("CHAIN_TIP_LINK:"<<chain_tip_link_);
         ROS_INFO_STREAM("CHAIN_BASE_LINK:"<<chain_base_link_);

     }
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
