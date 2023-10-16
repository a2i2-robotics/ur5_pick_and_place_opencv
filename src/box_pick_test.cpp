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

#include "misc.h"

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
    static const std::string PLANNING_GROUP_ARM = "ur5e_arm"; //"ur5_arm", "manipulator";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
    
    move_group_interface_arm.setPlanningTime(60);
    move_group_interface_gripper.setPlanningTime(60);
    ROS_INFO("get max plan time %f \n", move_group_interface_arm.getPlanningTime());


    const std::string end_effector_link = move_group_interface_arm.getEndEffectorLink();
    const std::string end_effector = move_group_interface_arm.getEndEffector();
    printf("End effector link %s \n", end_effector_link.c_str());
    printf("End effector %s \n", end_effector.c_str());

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    //planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;

    // 1. Move to home position
    std::string _home = read_filter("Go to reset(up) position? (Yes/No) ");
    transform(_home.begin(), _home.end(), _home.begin(), ::toupper);
    if (_home == "YES") {
	    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("up"));
	    
	    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");

	    move_group_interface_arm.move();
    }

    // 2. Get object position 
    // 2.1 Register for "box" filter
    // move_group_interface_arm.execute(trajectory);
    //move_group_interface_arm.move();

    ros::shutdown();
    return 0;
}
