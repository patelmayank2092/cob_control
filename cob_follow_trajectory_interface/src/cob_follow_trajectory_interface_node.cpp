#include <ros/ros.h>
#include <cob_follow_trajectory_interface/cob_follow_trajectory_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm");
    //new_handle.reset(new FollowJointTrajectoryControllerHandle(name, action_ns));
    FollowTrajectoryInterface* cob_follow_trajectory_interface = new FollowTrajectoryInterface("arm","follow_joint_trajectory");

    if (!cob_follow_trajectory_interface->initialize())
    {
        ROS_ERROR("Failed to initialize TwistController");
        return -1;
    }

    ros::spin();
    return 0;
}
