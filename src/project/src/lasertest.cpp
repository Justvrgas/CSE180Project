#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

using namespace std;

//testing this
 sensor_msgs::LaserScan laser;


void laserscan(const sensor_msgs::LaserScan::ConstPtr &msg){
	
	laser = *msg; 
	

}

int main(int argc,char ** argv) {

	ros::init(argc,argv,"lasertest");
	ros::NodeHandle nh;
    ros::Subscriber laser = nh.subscribe("/scan", 1000, &laserscan);
	
        
        char scan;

    while (ros::ok()){
        ros::spinOnce();
        
        cin >> scan;

        if(scan == 's'){

            for(int i = 180; i < 540; i++){

                ROS_INFO_STREAM("Laser Angle: " << i << "Value: " << laser.ranges[i]);

            }

            scan = 'x';

        }

    }
    

}    