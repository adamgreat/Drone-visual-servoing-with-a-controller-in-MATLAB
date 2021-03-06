#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/Pose2D.h" //for pub err
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <math.h>       /* sin */
#include <std_msgs/Float64.h> //for publishing yaw

#define PI 3.14159265
//Create global variables and declare a function
double quatx;
double quaty;
double quatz;
double quatw;
//double global_yaw;

#define FACTOR  0.6

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

// Subscriber to bottom camera
image_transport::Subscriber sub;

// Subscriber to flight mode
ros::Subscriber mavros_state_sub;

// RC publisher
ros::Publisher pub;

//Matlab/Simulink publisher
ros::Publisher err_pub;

//Yaw publisher
ros::Publisher yaw_pub;

// Time control
ros::Time lastTime;

// Mark info
float MarkX, MarkY; // Mark center
float lastMarkX, lastMarkY; // Last mark center
double lastMarkVelX, lastMarkVelY; // Last mark velocity

//Image center
float ImageX, ImageY;

double Roll, Pitch;

// Flight mode
std::string mode;
bool guided;
bool armed;

void ComPoseCallback(const geometry_msgs::PoseStamped& msg);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Time since last call
        double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();
        lastTime = ros::Time::now();
        
        char tab2[1024];
        strncpy(tab2, mode.c_str(), sizeof(tab2));
        tab2[sizeof(tab2) - 1] = 0;
        //ROS_INFO("Marker = (%f , %f) | LastMarker = (%f , %f) \n timeBetweenMarkers = %fs | lastMarkVelX = (%f , %f)\n Roll = %f | Pitch = %f\n Mode = %s \n", MarkX, MarkY, lastMarkX, lastMarkY, timeBetweenMarkers, lastMarkVelX, lastMarkVelY, Roll, Pitch, tab2);

        aruco::MarkerDetector MDetector;
        vector<aruco::Marker> Markers;
        cv::Point2f MarkCenter;

        // Get the msg image
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Error between Image and Mark
        float ErX = 0.0;
        float ErY = 0.0;

        // Get the Image center
        ImageX = InImage.cols / 2.0f;
        ImageY = InImage.rows / 2.0f;

        // Detect markers
        MDetector.detect(InImage,Markers);

        // Create RC msg
        mavros_msgs::OverrideRCIn msg;
        
        // Create MATLAB/Simulink msg
        geometry_msgs::Pose2D err_msg;


        lastMarkX = MarkX;
        lastMarkY = MarkY;

        // For each marker, draw info ant its coundaries in the image
        for (unsigned int i = 0; i<Markers.size(); i++){
            Markers[i].draw(InImage,cv::Scalar(0,0,255),2);

            // Calculate the error between Image center and Mark center
            MarkCenter = Markers[i].getCenter();
            MarkX = MarkCenter.x;
            MarkY = MarkCenter.y;
            ErX = ImageX - MarkX;
            ErY = ImageY - MarkY;
        }

        // Calculate velocity
        if (timeBetweenMarkers < 1.0){
            lastMarkVelX = (lastMarkX - MarkX)/timeBetweenMarkers;
            lastMarkVelY = (lastMarkY - MarkY)/timeBetweenMarkers;
        } else{
            lastMarkVelX = 0.0;
            lastMarkVelY = 0.0;
        }

        // Calculate Roll and Pitch depending on the mode
        if (mode == "LOITER"){
            Roll = BASERC - ErX * FACTOR;
            Pitch = BASERC - ErY * FACTOR;
        }else if (mode == "ALT_HOLD"){
            Roll = BASERC - (0.5*ErX+0.1*lastMarkVelX);
            Pitch = BASERC - (0.5*ErY+0.1*lastMarkVelY); 
        }else{
            Roll = BASERC;
            Pitch = BASERC;
        }  
         
        // Limit the Roll
        if (Roll > MAXRC)
        {
            Roll = MAXRC;
        } else if (Roll < MINRC)
        {
            Roll = MINRC;
        }

        // Limit the Pitch
        if (Pitch > MAXRC)
        {
            Pitch = MAXRC;
        } else if (Pitch < MINRC)
        {
            Pitch = MINRC;
        }

        msg.channels[0] = Roll;     //Roll
        msg.channels[1] = Pitch;    //Pitch
        msg.channels[2] = BASERC;   //Throttle
        msg.channels[3] = 0;        //Yaw
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;

        pub.publish(msg);
        //
        if(ErX!=0 || ErY!=0)
        {
            err_msg.x=ErX;
            err_msg.y=ErY;
            err_pub.publish(err_msg);
            ROS_INFO("err: [%f, %f]", err_msg.x, err_msg.y);
        }
        cv::imshow("view", InImage);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode = msg->mode;
    guided = msg->guided==128;
    armed = msg->armed==128;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
    mavros_state_sub = nh.subscribe("/mavros/state", 1, mavrosStateCb);
    ros::Subscriber ComPose_sub = nh.subscribe("/mavros/local_position/pose", 1000, ComPoseCallback);
    pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    err_pub = nh.advertise<geometry_msgs::Pose2D>("/matlab/err",1);
    yaw_pub = nh.advertise<std_msgs::Float64>("/matlab/yaw",1);
    ros::spin();


}

void ComPoseCallback(const geometry_msgs::PoseStamped& msg)            
{
    // Create MATLAB/Simulink msg for yaw
    std_msgs::Float64 yaw_msg;

    //ROS_INFO("Seq: [%d]", msg.header.seq);
    //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg.pose.position.x,msg.pose.position.y, msg.pose.position.z);
    //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);

   float linearposx=msg.pose.position.x;
   float linearposy=msg.pose.position.y;
   double quatx= msg.pose.orientation.x;
   double quaty= msg.pose.orientation.y;
   double quatz= msg.pose.orientation.z;
   double quatw= msg.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //global_yaw=yaw;
    ROS_INFO("Roll: [%f],Pitch: [%f],Yaw: [%f]",roll,pitch,yaw);
    yaw_msg.data=yaw;
    yaw_pub.publish(yaw_msg);
    return ;
}
