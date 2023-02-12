#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "global_planner/planner_core.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include "geometry_msgs/Twist.h"
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <rotate_recovery/rotate_recovery.h>




geometry_msgs::PoseStamped goal;
geometry_msgs::PoseStamped odomPose;
std::vector<geometry_msgs::PoseStamped> global_planner_path;
geometry_msgs::Twist Tiago_vel;

bool makeGlobalPlan  = false;
bool isGoal = false;
bool move = false;



void handle_goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  goal = *msg;
  goal.header.frame_id = "map";
  makeGlobalPlan = true;
  isGoal = true;
  ROS_INFO("Goal passed successfully");

}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  odomPose.pose = msg->pose.pose;
  odomPose.header = msg->header;
  odomPose.header.frame_id = "map";
  
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Rate rate(10.0);


  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  costmap_2d::Costmap2DROS globalCostmap("global_costmap", buffer);
  costmap_2d::Costmap2DROS localCostmap("local_costmap", buffer);

  global_planner::GlobalPlanner globalPlanner("global_planner", globalCostmap.getCostmap(), "map");

  base_local_planner::TrajectoryPlannerROS local_trajectory_planner;
  local_trajectory_planner.initialize("local_planner", &buffer, &localCostmap);



  rotate_recovery::RotateRecovery rr;
  rr.initialize("my_rotate_recovery", &buffer, &globalCostmap, &localCostmap);
  // rr.runBehavior();

  clear_costmap_recovery::ClearCostmapRecovery ccr;
  ccr.initialize("my_clear_costmap_recovery", &buffer, &globalCostmap, &localCostmap);
  // ccr.runBehavior();

  


  ros::Subscriber goalSubscriber = n.subscribe("/move_base_simple/goal", 1000, handle_goal);
  ros::Subscriber odometrySubscriber = n.subscribe("/mobile_base_controller/odom", 1000, odometryCallback);
  ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("nav_vel", 1000);

  while (ros::ok())
  {
    if (makeGlobalPlan)
    {
      globalPlanner.makePlan(odomPose, goal, global_planner_path);
      globalPlanner.publishPlan(global_planner_path);
      local_trajectory_planner.setPlan(global_planner_path);
      makeGlobalPlan = false;
      move = true;
      ROS_INFO("Planning");
      
    }

    if(move){
      if(!local_trajectory_planner.isGoalReached()){
        if(local_trajectory_planner.computeVelocityCommands(Tiago_vel)){
          velocityPublisher.publish(Tiago_vel);
          ROS_INFO("In progress");
        }
        else{
          ROS_INFO("Recovery behavior");
          // localCostmap.resetLayers();
          rr.runBehavior();
        }
      }
      else{
        ROS_INFO("Finish");
        move = false;
      }
    }
  ros::spinOnce();
  rate.sleep();
  }

  return 0;
}