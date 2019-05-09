#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseWithCovarianceStamped robot_pose;

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_pose = *msg;
    return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gotospecpoint");
  ros::NodeHandle nh;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher rot = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber odom_sub = nh.subscribe("/amcl_pose", 1000, &amclCB);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal rotate;
  geometry_msgs::Twist vel_msg;
  

  int angle = 180;
  float PI = 3.1415926535897;
  float angular_speed = 0.5*2*PI/360;
  float relative_angle = angle*2*PI/360;
  vel_msg.angular.z = angular_speed;
  
  int current_angle = 0;
  double t0 = ros::Time::now().toSec();

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = 2;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Moving to point");
  ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
  ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
  ac.sendGoal(goal);
  ac.waitForResult();

  ros::Rate rate(1);

  while(ros::ok()){
      ros::spinOnce();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, made it to the goal point");

      
      rot.publish(vel_msg);
      double t1 = ros::Time::now().toSec();
      current_angle = angular_speed*(t1-t0);

      if(current_angle > relative_angle){
        vel_msg.angular.z = 0;
        rot.publish(vel_msg);
      }
      
      
      
    }
    else{
      ROS_INFO("The base failed to move forward 1 meter for some reason sending next target");
      
    }
  }


  ROS_INFO("Task Complete");


  return 0;
}