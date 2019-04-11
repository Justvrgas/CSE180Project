#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>
using namespace std;

//testing this
sensor_msgs::LaserScan laser;                   

void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = *msg;
    return;
}


bool CheckObject(vector<float> values){

    int rsize = values.size(); //length of cluster vector

    if(rsize == 0){
        return false;
    }

    float angle_increment = 0.0065540750511;
    float mailbox = 1;

    ROS_INFO_STREAM(rsize);
    
    float alpha = (rsize-1)*angle_increment;
    float length = values[rsize-1]*asinf(alpha/2);
    float objectwidth = 2*length;

    if( abs(objectwidth - mailbox) <= 0.2){
        return true;
    }
    
    else{
        return false;
    }

}




int main(int argc,char ** argv) {

	ros::init(argc,argv,"lasertest");
	ros::NodeHandle nh;
    ros::Subscriber laserscanning = nh.subscribe("/scan", 1000, &laserscanCB);
	
        
        char scan;
        vector<float> values;
        vector<int> locations;
        ros::Rate r(1.0);
        
    
    while (ros::ok()){
        ros::spinOnce();
        
        
        
        

           for(int i = 0; i < laser.ranges.size(); i++){
               
               if( abs(laser.ranges[i] - laser.ranges[i+1]) <= 0.01 ){
                    
                    values.push_back(laser.ranges[i]);
                     
               }

               else{
                   //calculate what the object is and compare to objects
                  

                   if (CheckObject(values)){
                       ROS_INFO_STREAM("MAILBOX FOUND");




                   }
                   else{
                       values.clear();
                   }

                


               }

           }

           r.sleep();
        

    }


    

}    