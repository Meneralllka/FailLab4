#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Main MoveIt! libraries are included

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    // Use an AsyncSpinner instead of ros::spin()
    ros::AsyncSpinner spinner(0);
    spinner.start(); // Necessary for MoveIt! implementation

    // Specify the planning group
    static const std::string PLANNING_GROUP = "snake_group"; // Replace with your group controller's name

    // Create a MoveGroupInterface object
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Retrieve the joint model group for joint control
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()-> getJointModelGroup(PLANNING_GROUP);

    // Define PoseStamped messages for current and target poses
    geometry_msgs::PoseStamped init_pose;
    geometry_msgs::PoseStamped target_pose;

    // Get the current position and orientation of the end effector
    init_pose = move_group.getCurrentPose();

    // Set the loop rate
    ros::Rate loop_rate(50); // 50 Hz

    while (ros::ok())
    {
        target_pose = init_pose;
        target_pose.pose.position.x -= 0.5; // Move 0.7 units along the negative x-axis
        target_pose.pose.position.y -= 0;

        move_group.setApproximateJointValueTarget(target_pose);
        move_group.setStartStateToCurrentState();
        move_group.move();

        target_pose.pose.position.x -= 0;
        target_pose.pose.position.y += 0.5;
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.setStartStateToCurrentState();
        move_group.move();

        target_pose.pose.position.x -= 0.5;
        target_pose.pose.position.y += 0;
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.setStartStateToCurrentState();
        move_group.move();

        target_pose.pose.position.x -= 0;
        target_pose.pose.position.y -= 0.5;
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.setStartStateToCurrentState();
        move_group.move();

        target_pose.pose.position.x += 0.5;
        target_pose.pose.position.y += 0;
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.setStartStateToCurrentState();
        move_group.move();
        break;
        loop_rate.sleep();
    }
    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}
