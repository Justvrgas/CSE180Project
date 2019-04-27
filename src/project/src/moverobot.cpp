#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "moverobot");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  int i = 0;
  float x_array[45] = {-7.5,  -4.5,  -7.5,  -4.5,  -6,   1.5};
  float y_array[45] = {-7.5,  -7.5,  -4,5,  -4.5,  -6,  -7.5};

  goal.target_pose.pose.position.x = x_array[i];
  goal.target_pose.pose.position.y = y_array[i];
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Moving to point");
  ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
  ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
  ac.sendGoal(goal);
  
  ac.waitForResult();
  int count = 0;

  ros::Rate r(1.0);

  while(ros::ok()){

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || count == 3){
      
     
      ROS_INFO("Next Goal");
      i++;
      goal.target_pose.pose.position.x = x_array[i];
      goal.target_pose.pose.position.y = y_array[i];

      ROS_INFO("Sending new point");
      ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
      ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
      ac.sendGoal(goal);

    }

    else{

      ROS_INFO("The base failed to move resending goal");
      goal.target_pose.pose.position.x = x_array[i];
      goal.target_pose.pose.position.y = y_array[i];

      ROS_INFO("Sending new point");
      ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
      ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
      ac.sendGoal(goal);
      

    }

    r.sleep();

  }


  ROS_INFO("Task Complete");


  return 0;
}