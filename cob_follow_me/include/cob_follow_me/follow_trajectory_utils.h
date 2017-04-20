/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_cartesian_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *   Bruno Brito, email: bfb@ipa.fraunhofer.de / brunoffbrito0@gmail.com
 *
 * \date Date of creation: March, 2017
 * \brief
 *   Helper functions  used in the cob_cartesian_controller package.
 *
 ****************************************************************/

#ifndef COB_FOLLOW_TRAJECTORY_UTILS_H
#define COB_FOLLOW_TRAJECTORY_UTILS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

class FollowTrajectoryUtils
{
public:
    FollowTrajectoryUtils()
    {
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("follow_trajectory/preview_path", 1);
    }

    void transformPose(const std::string source_frame, const std::string target_frame, const geometry_msgs::Pose pose_in, geometry_msgs::Pose& pose_out);
    tf::StampedTransform getStampedTransform(const std::string& target_frame, const std::string& source_frame);
    geometry_msgs::Pose getPose(const std::string& target_frame, const std::string& source_frame);

    bool inEpsilonArea(const tf::StampedTransform& stamped_transform, const double epsilon);
    void poseToRPY(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw);

    void previewPath(const geometry_msgs::PoseArray pose_array);

    double roundUpToMultiplier(const double numberToRound, const double multiplier);

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    visualization_msgs::MarkerArray marker_array_;

    ros::Publisher marker_pub_;
};

#endif  // COB_FOLLOW_TRAJECTORY_UTILS_H
