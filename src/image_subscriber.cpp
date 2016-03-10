#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>
#include <mavros/OverrideRCIn.h>
#include <math.h>
#include <mavros/State.h>

/*
    TO DO : Choose a FACTOR of your choice. Note that:
            * Increasing this value will make the drone move faster, but imprecisely.
            * Decreasing the value will make the drone move more precisely, but slower.
*/
#define FACTOR //TO DO

/*
    TO DO : adapt these constants to your copter
*/
#define KP //TO DO
#define KD //TO DO


/*
    TO DO : Choose the correct RC values (minimum, base and maximum)
*/
#define MINRC   //TO DO
#define BASERC  //TO DO
#define MAXRC   //TO DO


// Subscriber to bottom camera
image_transport::Subscriber sub;

// Subscriber to flight mode
ros::Subscriber mavros_state_sub;

// RC publisher
ros::Publisher pub;

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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    // Time since last call
    double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();
    lastTime = ros::Time::now();
    
    char tab2[1024];
    strncpy(tab2, mode.c_str(), sizeof(tab2));
    tab2[sizeof(tab2) - 1] = 0;
    ROS_INFO("Marker = (%f , %f) | LastMarker = (%f , %f) \n timeBetweenMarkers = %fs | lastMarkVelX = (%f , %f)\n Roll = %f | Pitch = %f\n Mode = %s \n", MarkX, MarkY, lastMarkX, lastMarkY, timeBetweenMarkers, lastMarkVelX, lastMarkVelY, Roll, Pitch, tab2);

    aruco::MarkerDetector MDetector;
    vector<aruco::Marker> Markers;
    cv::Point2f MarkCenter;

    /*
        TO DO : Extract the image from the message (msg) and store it in InImage
    */
    cv::Mat InImage /* = image_from_the_msg */;

    // Detect the markers in InImage and store them in Markers
    MDetector.detect(InImage,Markers);

    // Create RC msg
    mavros::OverrideRCIn msg;

    // Error between Image and Mark
    float ErX = 0.0;
    float ErY = 0.0;

    // For each marker, draw info ant its coundaries in the image
    for (unsigned int i = 0; i<Markers.size(); i++){
        Markers[i].draw(InImage,cv::Scalar(0,0,255),2);

        /*
            TO DO : Calculate the error between the center of the image and the center of the mark


            ERROR_X_AXIS = IMAGE_CENTER_X_AXIS - MARK_CENTER_X_AXIS
            ERROR_Y_AXIS = IMAGE_CENTER_Y_AXIS - MARK_CENTER_Y_AXIS

        */
    }

    lastMarkX = MarkX;
    lastMarkY = MarkY;



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
        Roll = BASERC - (KP*ErX+KD*lastMarkVelX);
        Pitch = BASERC - (KP*ErY+KD*lastMarkVelY); 
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


    // Publish the msg
    pub.publish(msg);

    // Shows the image on screen
    cv::imshow("view", InImage);
    cv::waitKey(30);

}


void mavrosStateCb(const mavros::StateConstPtr &msg)
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
    pub = nh.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 10);;
    ros::spin();
}
