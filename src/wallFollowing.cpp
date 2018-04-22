#include "../include/node_wallFollowing.h"

#define SUBSCRIBER_BUFFER_SIZE 1
#define PUBLISHER_BUFFER_SIZE 1000
#define WALL_DISTANCE 1.5
#define MAX_SPEED 0.5
#define P 1
#define D 0.5
#define ANGLE_COEF 1
#define DIRECTION 1

#define PUBLISHER_TOPIC "/cmd_vel"

#define SUBSCRIBER_TOPIC "/scan"

#define SUBSCRIBER_HEIGHT_TOPIC "/sonar_height"

#define SUBSCRIBER_POSE_TOPIC "/slam_out_pose"

int main(int argc, char **argv){

    //Initialize node
    ros::init(argc, argv, "wallFollowing");
    ros::NodeHandle n;

    //create publisher
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

    //create object, which stores data from sensors and has methods for publishing and subscribing
    NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(pubMessage, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);
    ros::Subscriber forPose = n.subscribe(SUBSCRIBER_POSE_TOPIC, 50, &NodeWallFollowing::poseCallback, nodeWallFollowing);
    ros::Subscriber forHeight = n.subscribe(SUBSCRIBER_HEIGHT_TOPIC, 50, &NodeWallFollowing::sonarHeightCallback, nodeWallFollowing);
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);

    ros::spin();

    return 0;

}