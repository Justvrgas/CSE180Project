#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>



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
  int count = 0;

  ros::Rate r(1.0);
  
  while(ros::ok()){

    mystate = ac.getState();

    ROS_INFO_STREAM("My Curr State: " << mystate.toString());

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ){
     
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

    else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED || ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        ROS_INFO("FAILED");
        randomX = std::rand() % 19 + (-9);
        randomY = std::rand() % 19 + (-9);
        goal.target_pose.pose.position.x = randomX;
        goal.target_pose.pose.position.y = randomY;
        
        ROS_INFO("Skipping point, sending new point");
        ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
        ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
        ac.sendGoalAndWait(goal, ros::Duration(60.0));
      
  
    }

    r.sleep();

  }


  return 0;
}