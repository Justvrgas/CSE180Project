#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath>

using namespace std;

//contains the laser data and their original index
struct Sample{

  int   original_index;
  float laser_range;
 
};

sensor_msgs::LaserScan laser;    
geometry_msgs::PoseWithCovarianceStamped robot_pose;               

void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = *msg;
    return;
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_pose = *msg;
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

pair<float, float> Object_Points(vector<Sample> data){

  int datasize = data.size();

  float angle_increment = 0.0065540750511;
  float theta = 0;
  float mid_laser_dist = data[datasize/2].laser_range;
  theta = laser.angle_min + data[datasize/2].original_index * angle_increment;

  //calculates the position of the robot by converting polar coordinates to cartesian
  float x = mid_laser_dist * std::cos(theta) + robot_pose.pose.pose.position.x;
  float y = mid_laser_dist * std::sin(theta) + robot_pose.pose.pose.position.y;

  ROS_INFO_STREAM("X:" << x);
  ROS_INFO_STREAM("Y:" << y);

  return make_pair(x,y); //returns the pair coordinates 

}


int main(int argc,char ** argv) {

	ros::init(argc,argv,"lasertest");
	ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/amcl_pose", 1000, &amclCB);
  ros::Subscriber laserscanning = nh.subscribe("/scan", 1000, &laserscanCB);

  
  vector<Sample> data; //contains lasers that are closely together and also stores their original indeces
  Sample sample;
  vector< pair<float, float> > Object_Position;//a vector that contains the positions of the objects
 	
  ros::Rate r(1.0);

  while (ros::ok()){
    
    ros::spinOnce();
    
    for(int i = 0; i < laser.ranges.size(); i++){
      
      //if values are close together insert into array
      if(abs(laser.ranges[i] - laser.ranges[i+1]) <= 0.2){

        sample.original_index = i;
        sample.laser_range = laser.ranges[i];
        data.push_back(sample);

      }

      else{

        //create an if to check if points calculated from the steps below have already been seen to not include them into the array.
        
        if(MailCheck(data)){
          ROS_INFO_STREAM("YO THERES A MAILBOX HERE DAWG!");   

        

          pair<float, float> coord = Object_Points(data); //pair structure coord is the value from the calculations in object_points
          Object_Position.push_back(coord); //add the new coordinate point into a vector
          

          ROS_INFO_STREAM("MAILBOX COORDINATES: (" << coord.first << ", " << coord.second << ")");      
        }

        data.clear();

      }  


    }
  
    r.sleep();

  }

} 
