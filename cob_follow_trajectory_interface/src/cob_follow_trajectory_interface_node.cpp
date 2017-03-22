#include <ros/ros.h>
#include <cob_follow_trajectory_interface/cob_follow_trajectory_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_follow_trajectory_interface_node");
    //new_handle.reset(new FollowJointTrajectoryControllerHandle(name, action_ns));
    FollowTrajectoryInterface* cob_follow_trajectory_interface = new FollowTrajectoryInterface("follow_trajectory_interface");

    if (!cob_follow_trajectory_interface->initialize())
    {
        ROS_ERROR("Failed to initialize Follow TRajectory Interface Node");
        return -1;
    }

    ros::spin();
    return 0;
}
