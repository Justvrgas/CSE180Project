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
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


sensor_msgs::LaserScan laser;    
geometry_msgs::PoseWithCovarianceStamped robot_pose;               

//contains the laser data and their original index
struct Sample{

  int   original_index;
  float laser_range;
 
};

void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = *msg;
    return;
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_pose = *msg;
    return;
}

//checks if a mailbox or table has been seen
bool ObjectCheck(vector<Sample> data){

  int datasize = data.size(); //length of cluster vector

  if(datasize == 0){
      return false;
  }

  float angle_increment = 0.0065540750511;
  float mailbox = .60;
  float table = .15566;

  //calculation to get size of the array in meters
  float alpha = (datasize-1)*angle_increment;
  float length = data[ datasize - 1 ].laser_range*asinf(alpha/2);
  float objectwidth = 2*length;

  if( abs(objectwidth - mailbox) <= 0.2){
      return true;
  }

  else if(abs(objectwidth - table) <= 0.2){
    return true;
  }

  else{
      return false;
  }


}

//checks if a sample is a mailbox
bool MailCheck(vector<Sample> data){

  int datasize = data.size(); //length of cluster vector

  if(datasize == 0){
      return false;
  }

  float angle_increment = 0.0065540750511;
  float mailbox = .60;


  //calculation to get size of the array in meters
  float alpha = (datasize-1)*angle_increment;
  float length = data[ datasize - 1 ].laser_range*asinf(alpha/2);
  float objectwidth = 2*length;

  if( abs(objectwidth - mailbox) <= 0.2){
      return true;
  }

  else{
      return false;
  }

}


//checks if a sample is a table
bool TableCheck(vector<Sample> data){

  int datasize = data.size(); //length of cluster vector

  if(datasize == 0){
      return false;
  }

  float angle_increment = 0.0065540750511;
  float table = .15566;


  //calculation to get size of the array in meters
  float alpha = (datasize-1)*angle_increment;
  float length = data[ datasize - 1 ].laser_range*asinf(alpha/2);
  float objectwidth = 2*length;

  if( abs(objectwidth - table) <= 0.2){
      return true;
  }

  else{
      return false;
  }



}

//calculates object coordinates
pair<float, float> Object_Points(vector<Sample> data){

  int datasize = data.size();
  float object_angle = datasize/2;
  float angle_increment = 0.0065540750511;
  float theta = 0;
  float mid_laser_dist = data[object_angle].laser_range;
  float quat_x = robot_pose.pose.pose.orientation.x;				//Extracting current Quaternion Values of Robot
	float quat_y = robot_pose.pose.pose.orientation.y;
	float quat_z = robot_pose.pose.pose.orientation.z;
	float quat_w = robot_pose.pose.pose.orientation.w;
  tf::Quaternion qft(quat_x, quat_y, quat_z, quat_w);

  float theta_temp = tf::getYaw(qft);

  if(object_angle > 360 && theta_temp > 0){
    theta = tf::getYaw(qft) - (object_angle - 360) * angle_increment;
  }

  else if(object_angle > 360 && theta_temp < 0){
     theta = tf::getYaw(qft ) - (object_angle - 360) * angle_increment;
  }

  else if(object_angle < 360 && theta_temp >= 0){
     theta = tf::getYaw(qft) + (360 - object_angle) * angle_increment;
  }

  else{
     theta = tf::getYaw(qft) + (360 - object_angle) * angle_increment;
  }

  //calculates the position of the robot by converting polar coordinates to cartesian
  float x = (mid_laser_dist * std::cos(theta)) + robot_pose.pose.pose.position.x;
  float y = (mid_laser_dist * std::sin(theta)) + robot_pose.pose.pose.position.y;

  ROS_INFO_STREAM("X:" << x);
  ROS_INFO_STREAM("Y:" << y);

  return make_pair(x,y); //returns the pair coordinates 

}


int main(int argc, char** argv){
  
  ros::init(argc, argv, "final");
  ros::NodeHandle nh;
  ros::Publisher rotation = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

            //VARIABLES//
  geometry_msgs::Twist rot;
  rot.angular.z = .5;
  move_base_msgs::MoveBaseGoal goal;
  int randomX = std::rand() % 19 + (-9);
  int randomY = std::rand() % 19 + (-9);
  int count = 0;
  vector<Sample> data; //contains lasers that are closely together and also stores their original indeces
  Sample sample;
  vector< pair<float, float> > Object_Position;//a vector that contains the positions of the objects
 	

  //we'll send a random goal point
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();  
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

    //made it to a point now laser scan and check for a mailbox
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ){
     
        for(int i = 0; i < laser.ranges.size(); i++){
      
          //if values are close together insert into array
          if(abs(laser.ranges[i] - laser.ranges[i+1]) <= 0.1 && laser.ranges[i] < 5 && laser.ranges[i+1] < 5){

            sample.original_index = i;
            sample.laser_range = laser.ranges[i];
            data.push_back(sample);

          }

          //cut off the inputed sample vector
          else{

            //check if the sample vector is either a mailbox or a table
            if(ObjectCheck(data)){
              ROS_INFO_STREAM("Object Spotted!");   
              count++;
              rotation.publish(rot);

                data.clear();
                rot.angular.z = 0;
                rotation.publish(rot);

                  if(abs(laser.ranges[i] - laser.ranges[i+1]) <= 0.1 && laser.ranges[i] < 5 && laser.ranges[i+1] < 5){

                    sample.original_index = i;
                    sample.laser_range = laser.ranges[i];
                    data.push_back(sample);

                  }

                  //cut off the inputed sample vector
                  else{

                    if(ObjectCheck(data)){

                        if(MailCheck(data)){
                           ROS_INFO_STREAM("MAILBOX Spotted!");
                        }

                        else if(TableCheck(data)){
                          ROS_INFO_STREAM("TABLE Spotted!");
                        }


                    }

                    

                  }


              // pair<float, float> coord = Object_Points(data); //pair structure coord is the value from the calculations in object_points
              // Object_Position.push_back(coord); //add the new coordinate point into a vector
              // ROS_INFO_STREAM("ESTIMATED MAILBOX COORDINATES: (" << coord.first << ", " << coord.second << ")");  

             
            }




            else if (TableCheck(data)){
              ROS_INFO("TABLE SPOTTED");
              // pair<float, float> coord = Object_Points(data); //pair structure coord is the value from the calculations in object_points
              // Object_Position.push_back(coord); //add the new coordinate point into a vector
              // ROS_INFO_STREAM("ESTIMATED TABLE COORDINATES: (" << coord.first << ", " << coord.second << ")"); 
            }
            
            //clear the sample array and continue checking
            data.clear();

          }  


        }

      //continue on and move to the next point
      ROS_INFO("Arrived!");
      randomX = std::rand() % 19 + (-9);
      randomY = std::rand() % 19 + (-9);
      goal.target_pose.pose.position.x = randomX;
      goal.target_pose.pose.position.y = randomY;
      ROS_INFO("Sending new point");
      ROS_INFO_STREAM("X: " << goal.target_pose.pose.position.x);
      ROS_INFO_STREAM("Y: " << goal.target_pose.pose.position.y);
      ac.sendGoalAndWait(goal, ros::Duration(60.0));
      count = 0;

    }

    //if the robot gets stuck or takes too long to reach the point then skip it and continue on
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