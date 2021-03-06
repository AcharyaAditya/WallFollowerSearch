#include "../include/node_wallFollowing.h"
#include "std_msgs/Float32.h"

#define SUBSCRIBER_BUFFER_SIZE 1000
#define PUBLISHER_BUFFER_SIZE 1000
#define WALL_DISTANCE 0.7
#define MAX_SPEED 0.2
#define P 2
#define D 1
#define ANGLE_COEF 0.3
#define DIRECTION -1

#define PUBLISHER_TOPIC "/cmd_vel"

#define PUBLISHER_TOPIC_DISTANCE "/myDistance"

#define PUBLISHER_TOPIC_ERROR "/myError"

#define SUBSCRIBER_TOPIC "/scan"

#define SUBSCRIBER_HEIGHT_TOPIC "/sonar_height"

#define SUBSCRIBER_POSE_TOPIC "/slam_out_pose"

int main(int argc, char **argv){

    //Initialize node
    ros::init(argc, argv, "wallFollowing");
    ros::NodeHandle n;

    //create publisher
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
    ros::Publisher meroDistance = n.advertise<std_msgs::Float32>(PUBLISHER_TOPIC_DISTANCE, PUBLISHER_BUFFER_SIZE);
    ros::Publisher calculatedError = n.advertise<std_msgs::Float32>(PUBLISHER_TOPIC_ERROR, PUBLISHER_BUFFER_SIZE);

    //create object, which stores data from sensors and has methods for publishing and subscribing
    NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(meroDistance, calculatedError, pubMessage, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);
    ros::Subscriber forPose = n.subscribe(SUBSCRIBER_POSE_TOPIC, 50, &NodeWallFollowing::poseCallback, nodeWallFollowing);
    ros::Subscriber forHeight = n.subscribe(SUBSCRIBER_HEIGHT_TOPIC, 50, &NodeWallFollowing::sonarHeightCallback, nodeWallFollowing);
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);
    
    ros::spin();

    return 0;

}