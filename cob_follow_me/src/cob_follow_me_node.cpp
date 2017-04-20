#include <ros/ros.h>
#include <cob_follow_me/cob_follow_me.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_follow_trajectory_interface_node");
    //new_handle.reset(new FollowJointTrajectoryControllerHandle(name, action_ns));
    FollowMe* cob_follow_me = new FollowMe("follow_trajectory_interface");

    if (!cob_follow_me->initialize())
    {
        ROS_ERROR("Failed to initialize Follow TRajectory Interface Node");
        return -1;
    }

    ros::spin();
    return 0;
}
