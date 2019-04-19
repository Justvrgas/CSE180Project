// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
// #include <nav_msgs/Odometry.h>
// #include <iostream>
// #include <cmath>
// using namespace std;

// //testing this
// sensor_msgs::LaserScan laser;    
// nav_msgs::Odometry curr_robot_pose;               

// void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
//     laser = *msg;
//     return;
// }

// void odomCB(const nav_msgs::Odometry::ConstPtr &msg) {
//     curr_robot_pose = *msg;
//     return;
// }

// bool CheckForMailbox(vector<float> values){

//     int rsize = values.size(); //length of cluster vector

//     if(rsize == 0){
//         return false;
//     }

//     float angle_increment = 0.0065540750511;
//     float mailbox = .80;

//     ROS_INFO_STREAM(rsize);
    
//     float alpha = (rsize-1)*angle_increment;
//     float length = values[rsize-1]*asinf(alpha/2);
//     float objectwidth = 2*length;

//     if( abs(objectwidth - mailbox) <= 0.2){
//         return true;
//     }
//     else{
//         return false;
//     }

// }

// bool CheckForTable(vector<float> values){

//     int rsize = values.size(); //length of cluster vector

//     if(rsize == 0){
//         return false;
//     }

//     float angle_increment = 0.0065540750511;
//     float tableleg = .150;

//     ROS_INFO_STREAM(rsize);
    
//     float alpha = (rsize-1)*angle_increment;
//     float length = values[rsize-1]*asinf(alpha/2);
//     float objectwidth = 2*length;

//     if( abs(objectwidth - tableleg) < 0.1){
//         return true;
//     }
//     else{
//         return false;
//     }

// }


// int CalculateDistance(int index, int laser){

//     int x_mailbox = laser


// }




// int main(int argc,char ** argv) {

// 	ros::init(argc,argv,"lasertest");
// 	ros::NodeHandle nh;

//     ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 1000, &odomCB);
//     ros::Subscriber laserscanning = nh.subscribe("/scan", 1000, &laserscanCB);
	
        
//         char scan;
//         vector<float> values;
//         vector<int> object_locations;
//         ros::Rate r(1.0);
//         int original_index;
//         int laserdist;

//     while (ros::ok()){
//         ros::spinOnce();
        
        
        
        

//            for(int i = 0; i < laser.ranges.size(); i++){
               
//                if( abs(laser.ranges[i] - laser.ranges[i+1]) <= 0.01 ){
                    
//                     values.push_back(laser.ranges[i]);
                     
//                }

//                else{
//                    //calculate what the object is and compare to objects
                  

//                    if (CheckForMailbox(values)){
//                        ROS_INFO_STREAM("MAILBOX FOUND");

//                        for(int i = 0; i < laser.ranges.size(); i++){
//                             if(laser.ranges[i] == values[i]){
//                                     original_index = i;
//                             }   
//                        }

//                        laserdist = laser.ranges[original_index];

//                        CalculateDistance(original_index, laserdist);                    

//                        ROS_INFO_STREAM("MAILBOC COORDINATES:"); 

//                    }

//                    else if(CheckForTable(values)){
//                        ROS_INFO_STREAM("Table FOUND");
//                        ROS_INFO_STREAM("Table COORDINATES:");
//                    }

//                    else{
//                        values.clear();
//                    }

                


//                }

//            }

//            r.sleep();
        

//     }


    

// }    


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath>


struct Sample{

public:

  int   original_index;
  float laser_range;
 
};



//testing this
sensor_msgs::LaserScan laser;    
nav_msgs::PoseWithCovarianceStamped robot_pose;               

void laserscanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = *msg;
    return;
}

void odomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    curr_robot_pose = *msg;
    return;
}

bool CheckForMailbox(vector<Sample> values){

    int rsize = values.size(); //length of cluster vector

    if(rsize == 0){
        return false;
    }

    float angle_increment = 0.0065540750511;
    float mailbox = .80;

    ROS_INFO_STREAM(rsize);
    
    float alpha = (rsize-1)*angle_increment;
    float length = values[rsize-1].laser_range*asinf(alpha/2);
    float objectwidth = 2*length;

    if( abs(objectwidth - mailbox) <= 0.2){
        return true;
    }
    else{
        return false;
    }

}

bool CheckForTable(vector<Sample> values){

    int rsize = values.size(); //length of cluster vector

    if(rsize == 0){
        return false;
    }

    float angle_increment = 0.0065540750511;
    float tableleg = .150;

    ROS_INFO_STREAM(rsize);
    
    float alpha = (rsize-1)*angle_increment;
    float length = values[rsize-1].laser_range*asinf(alpha/2);
    float objectwidth = 2*length;

    if( abs(objectwidth - tableleg) < 0.1){
        return true;
    }
    else{
        return false;
    }

}


int CalculateDistance(vector<Sample> values){

    int rsize = values.size(); //length of cluster vector	
    float angle_increment = 0.0065540750511;
    float theta = 0;
    theta = laser.min_angle + values[rsize/2].original_index * angle_increment;
    
    float x = values[rsize/2].laser_range * std::cos(theta) + robot_pose.pose.position.x;
    float y = values[rsize/2].laser_range * std::sin(theta) + robot_pose.pose.position.y;


}




int main(int argc,char ** argv) {

	ros::init(argc,argv,"lasertest");
	ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/amcl", 1000, &odomCB);
    ros::Subscriber laserscanning = nh.subscribe("/scan", 1000, &laserscanCB);
	
        
        char scan;
        vector<Sample> values;
        vector<int> object_locations;
        ros::Rate r(1.0);
        int original_index;
        int laserdist;

    while (ros::ok()){
        ros::spinOnce();
        
        
        
        

           for(int i = 0; i < laser.ranges.size(); i++){
               
               if( abs(laser.ranges[i] - laser.ranges[i+1]) <= 0.01 ){
                    Sample sample;
                    sample.original_index = i;
                    sample.laser_range = laser.ranges[i];
                    values.push_back(sample);
                     
               }

               else{
                   //calculate what the object is and compare to objects
                  

                   if (CheckForMailbox(values)){
                       ROS_INFO_STREAM("MAILBOX FOUND");

                       float mail_loc = CalculateDistance(values);                    

                       ROS_INFO_STREAM("MAILBOC COORDINATES:"); 

                   }

                   else if(CheckForTable(values)){
                       ROS_INFO_STREAM("Table FOUND");
                       ROS_INFO_STREAM("Table COORDINATES:");
                   }

                   else{
                       values.clear();
                   }

                


               }

           }

           r.sleep();
        

    }


    

} 
