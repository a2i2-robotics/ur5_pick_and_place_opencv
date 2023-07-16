#include <string>
#include <vector>
#include "ros/ros.h"
#include <ros/console.h>
#include "ur5_realsense_perception_msgs/Filters.h"
#include "ur5_realsense_perception_msgs/Detection.h"
#include "ur5_realsense_perception_msgs/DetectedObjects.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_node");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    
    move_group_interface_arm.setPlanningTime(60);
    move_group_interface_gripper.setPlanningTime(60);
    ROS_INFO("get max plan time %f", move_group_interface_arm.getPlanningTime());

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");

    move_group_interface_arm.move();

    // 2. Get object position 
    // 2.1 Register for "box" filter
    ros::ServiceClient filter_srv_client = n.serviceClient<ur5_realsense_perception_msgs::Filters>("filter_registration");
    ur5_realsense_perception_msgs::Filters srv2;
    std::vector<std::string> filters{"box"};
    srv2.request.filters = filters;
    if(filter_srv_client.call(srv2)) {
	ROS_INFO_STREAM("Filters registered: " << srv2.response.filters[0]);
    } else {
	ROS_INFO_STREAM("Failed to register filters");
    }

    // 2.2 Get "box" position
    ros::ServiceClient position_srv_client = n.serviceClient<ur5_realsense_perception_msgs::DetectedObjects>("object_detection");

    ur5_realsense_perception_msgs::DetectedObjects srv;

    if(position_srv_client.call(srv)) {
	ROS_INFO_STREAM("3d target position base frame: x " << srv.response.objects[0].position.x << " y " << srv.response.objects[0].position.y << " z " << srv.response.objects[0].position.z);
    } else {
	ROS_INFO_STREAM("Failed to call box and target position service");
    }

    // 3. Move robot arm
    ROS_INFO_NAMED("tutorial", "// 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box");
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");

    geometry_msgs::Pose target_pose1;
  
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = srv.response.objects[0].position.x;
    target_pose1.position.y = srv.response.objects[0].position.y;
    target_pose1.position.z = srv.response.objects[0].position.z + 0.3;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Goal position x:%f y:%f z:%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "SUCCESS" : "FAILED");

    move_group_interface_arm.move();

  ros::shutdown();
  return 0;
}
