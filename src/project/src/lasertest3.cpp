#include <ros/ros.h>
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

//contains the laser data and their original index
struct Sample{

  int   original_index;
  float laser_range;
 
};

sensor_msgs::LaserScan laser;    
geometry_msgs::PoseWithCovarianceStamped robot_pose;               
std_msgs::Bool beginscan;
std_msgs::Bool detected;

void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = *msg;
    return;
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_pose = *msg;
    return;
}

void arrivedCB(const std_msgs::BoolConstPtr &msg) {
    beginscan = *msg;
    return;
}

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


bool TableCheck(vector<Sample> data){

  int datasize = data.size(); //length of cluster vector

  if(datasize == 0){
      return false;
  }

  float angle_increment = 0.0065540750511;
  float mailbox = .15566;


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


int main(int argc,char ** argv) {

	ros::init(argc,argv,"lasertest2");
	ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/amcl_pose", 1000, &amclCB);
  ros::Subscriber laserscanning = nh.subscribe("/scan", 1000, &laserscanCB);
  //recieves a bool from moverobot that it has arrived from its location
  ros::Subscriber scan = nh.subscribe("/arrived", 1000, &arrivedCB);
  //sends a bool to moverobot to tell it either found a mailbox or it didnt (if it did it will make the robot turn 180 degrees and check again)
  ros::Publisher object_detected = nh.advertise<std_msgs::Bool>("/object", 1000);


  //variables//
  vector<Sample> data; //contains lasers that are closely together and also stores their original indeces
  Sample sample;
  vector< pair<float, float> > Object_Position;//a vector that contains the positions of the objects
  int count;
 	

  ros::Rate r(1.0);

  while (ros::ok()){
    
    ros::spinOnce();

    //moverobot has reached a point being the scan
    if(beginscan.data = true){

      for(int i = 0; i < laser.ranges.size(); i++){
        
        //if values are close together insert into array
        if(abs(laser.ranges[i] - laser.ranges[i+1]) <= 0.2 && laser.ranges[i] < 3 && laser.ranges[i+1] < 3){

          sample.original_index = i;
          sample.laser_range = laser.ranges[i];
          data.push_back(sample);

        }

        //cut off the inputed sample vector
        else{

          //check if the sample vector is either a mailbox or a table
          if(MailCheck(data)){
            ROS_INFO_STREAM("MAILBOX SPOTTED!");  
            count++;

            //if when rotated the mailbox is detected twice then calculate the posituion of the mailbox
            if(count == 2){
              // pair<float, float> coord = Object_Points(data); //pair structure coord is the value from the calculations in object_points
              // Object_Position.push_back(coord); //add the new coordinate point into a vector
              // ROS_INFO_STREAM("ESTIMATED MAILBOX COORDINATES: (" << coord.first << ", " << coord.second << ")");
              break;
            }

            //tell moverobot to turn 180  
            detected.data = true;
            object_detected.publish(detected);      
          }

          else if (TableCheck(data)){
            ROS_INFO("TABLE SPOTTED");
            count++;

            //if when rotated the table is detected twice then calculate the posituion of the table
            if(count == 2){
              // pair<float, float> coord = Object_Points(data); //pair structure coord is the value from the calculations in object_points
              // Object_Position.push_back(coord); //add the new coordinate point into a vector
              // ROS_INFO_STREAM("ESTIMATED TABLE COORDINATES: (" << coord.first << ", " << coord.second << ")");
              break;
            }

            //tell moverobot to turn 180  
            detected.data = true;
            object_detected.publish(detected);     
          }
          
          else{
            //nothing detected continue on
            detected.data = false;
            object_detected.publish(detected);
            //clear the sample array and continue checking
            data.clear();
          }

        }  

      }

    }
      r.sleep();

  }

} 
