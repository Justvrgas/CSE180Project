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

void odomCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_pose = *msg;
    return;
}

bool MailCheck(vector<Sample> data){

    int datasize = data.size(); //length of cluster vector

   ROS_INFO_STREAM(datasize);

    if(datasize == 0){
        return false;
    }

    float angle_increment = 0.0065540750511;
    float mailbox = .60;

 
    
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

  ROS_INFO_STREAM("End Index: " << data[datasize - 1].original_index);
  ROS_INFO_STREAM("Middle Index: " << data[datasize/2].original_index);
  ROS_INFO_STREAM("Start Index: " << data[0].original_index);

  float angle_increment = 0.0065540750511;
  float theta = 0;
  float mid_laser_dist = data[datasize/2].laser_range;

  ROS_INFO_STREAM("mid laser dist: " << mid_laser_dist);

  theta = laser.angle_min + data[datasize/2].original_index * angle_increment;
  float x = mid_laser_dist * std::cos(theta) + robot_pose.pose.pose.position.x;
  float y = mid_laser_dist * std::sin(theta) + robot_pose.pose.pose.position.y;

  ROS_INFO_STREAM("X:" << x);
  ROS_INFO_STREAM("Y:" << y);

  return make_pair(x,y);

}


int main(int argc,char ** argv) {

	ros::init(argc,argv,"lasertest");
	ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/amcl_pose", 1000, &odomCB);
  ros::Subscriber laserscanning = nh.subscribe("/scan", 1000, &laserscanCB);

  
  vector<Sample> data;
  Sample sample;
  vector< pair<float, float> > Object_Position;
 	
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
        
        if(MailCheck(data)){
          ROS_INFO_STREAM("YO THERES A MAILBOX HERE DAWG!");   

          pair<float, float> coord = Object_Points(data);
          Object_Position.push_back(coord);
          

          ROS_INFO_STREAM("MAILBOX COORDINATES: (" << coord.first << ", " << coord.second << ")");      
        }
        ROS_INFO_STREAM("This is new size: " << data.size());
        data.clear();

      }       
    }
  
           

    r.sleep();

  }

} 
