#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath>
#include <std_msgs/Bool.h>

sensor_msgs::LaserScan laser;    
geometry_msgs::PoseWithCovarianceStamped robot_pose;               
std_msgs::Bool check;
std_msgs::Bool objectdetect;

void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = *msg;
    return;
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_pose = *msg;
    return;
}

void detectorCB(const std_msgs::Bool::ConstPtr &msg){
    objectdetect = *msg;
    return;
}


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc,argv,"moverobot");
	ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/amcl_pose", 1000, &amclCB);
  ros::Subscriber laserscanning = nh.subscribe("/scan", 1000, &laserscanCB);
  //
  ros::Subscriber detector_sub = nh.subscribe("/object", 1000, &detectorCB);
  
  //sends bool to lasertest2 that it has arived to its point so it should scan
  ros::Publisher destination_arrived = nh.advertise<std_msgs::Bool>("/arrived", 1000);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a random goal point
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  int randomX = std::rand() % 19 + (-9);
  int randomY = std::rand() % 19 + (-9);
  goal.target_pose.pose.position.x = randomX;
  goal.target_pose.pose.position.y = randomY;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Moving to point");
  ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
  ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
  ac.sendGoalAndWait(goal, ros::Duration(60.0));
  actionlib::SimpleClientGoalState mystate = ac.getState();
  
  ac.waitForResult();
  

  ros::Rate r(1.0);
  
  while(ros::ok()){

    ros::spinOnce();

    mystate = ac.getState();
    ROS_INFO_STREAM("My Curr State: " << mystate.toString());

    if(objectdetect.data = true){
      goal.target_pose.pose.position.x = randomX;
      goal.target_pose.pose.position.y = randomY;
      goal.target_pose.pose.orientation.w = 0.1;
      ac.sendGoal(goal);
    }

    else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    check.data = true;
    destination_arrived.publish(check);
    }
    
    //if nothing is detected or the point is unreachable send next point
    else if(objectdetect.data = false || ac.getState() == actionlib::SimpleClientGoalState::ABORTED || ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
     
      ROS_INFO("Arrived!");
      randomX = std::rand() % 19 + (-9);
      randomY = std::rand() % 19 + (-9);
      goal.target_pose.pose.position.x = randomX;
      goal.target_pose.pose.position.y = randomY;
      ROS_INFO("Sending new point");
      ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
      ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
      ac.sendGoalAndWait(goal, ros::Duration(60.0));

    }

    // else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED || ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
    //     ROS_INFO("FAILED");
    //     randomX = std::rand() % 19 + (-9);
    //     randomY = std::rand() % 19 + (-9);
    //     goal.target_pose.pose.position.x = randomX;
    //     goal.target_pose.pose.position.y = randomY;
        
    //     ROS_INFO("Skipping point, sending new point");
    //     ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
    //     ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
    //     ac.sendGoalAndWait(goal, ros::Duration(60.0));
      
  
    // }

    //turn off laser
    check.data = false;
    destination_arrived.publish(check);
    r.sleep();

  }


  return 0;
}