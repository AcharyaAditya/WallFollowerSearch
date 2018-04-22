#include "../include/node_wallFollowing.h"
#include <math.h>
#define PI 3.141592

NodeWallFollowing::NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an)
{
    wallDistance = wallDist;
    maxSpeed = maxSp;
    direction = dir;
    P = pr;
    D = di;
    angleCoef = an;
    e = 0;
    angleMin = 0;
    pubMessage = pub;
}

NodeWallFollowing::~NodeWallFollowing() {}

//Publisher
void NodeWallFollowing::publishMessage()
{
    double currentTime;
    //preparing message
    geometry_msgs::Twist msg;
    msg.angular.z = direction * (P * e + D * diffE) + angleCoef * (angleMin - PI * direction / 2);

    //added for takeoff in simulation
    if (sonarHeight < 0.5)
    {
        msg.linear.z = 1;
    }

    //Closure check
    currentTime = ros::Time::now().toSec();
    if (currentTime > (distChangeTime + 10))
    {
        if (((positionX) > (referencePosX - 1) && positionX < (referencePosX + 1)) && ((positionY) > (referencePosY - 1) && positionY < (referencePosY + 1)))
        {
        }
    }

    if (distFront < wallDistance)
    {
        msg.linear.x = 0;
    }
    else if (distFront > wallDistance * 2)
    {
        msg.linear.x = 0.5 * maxSpeed;
    }
    else if (fabs(angleMin) > 1.75)
    {
        msg.linear.x = 0.4 * maxSpeed;
    }
    else
    {
        msg.linear.x = maxSpeed;
    }

    //publish message
    pubMessage.publish(msg);
}

//Subscrber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int size = msg->ranges.size();
    // double minDistance = msg->ranges[0];
    //variables with index of highest and lowest values in array
    int minIndex = size * (direction + 1) / 4;
    int maxIndex = size * (direction + 3) / 4;
    // int minIndex = 0;
    // int maxIndex = size;

    for (int i = minIndex; i < maxIndex; i++)
    {
        if ((msg->ranges[i] < msg->ranges[minIndex]) && (msg->ranges[i] > 0.01))
        {
            minIndex = i;
        }
        // if(msg->ranges[i] < minDistance){
        //     minIndex = i;
        //     minDistance = msg->ranges[i];
        // }
    }

    //calculation of angles from indexes and storing data to class variables
    angleMin = (minIndex - size / 2) * msg->angle_increment;
    double distMin;
    distMin = msg->ranges[minIndex];
    distFront = msg->ranges[size / 2];
    diffE = (distMin - wallDistance);

    publishMessage();
}

void NodeWallFollowing::sonarHeightCallback(const sensor_msgs::Range::ConstPtr &sonarMsg)
{

    sonarHeight = sonarMsg->range;
}

void NodeWallFollowing::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &poseMsg)
{

    positionX = poseMsg->pose.position.x;
    positionY = poseMsg->pose.position.y;
}
