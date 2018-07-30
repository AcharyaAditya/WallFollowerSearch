#include "../include/node_wallFollowing.h"
#include "std_msgs/Float32.h"
#include <math.h>
#define PI 3.141592

double NodeWallFollowing::myErr = 0;
double NodeWallFollowing::dist180 = 0;
bool NodeWallFollowing::largeChange = false;
double NodeWallFollowing::previousTime = 0;
bool NodeWallFollowing::turnComp = true;

NodeWallFollowing::NodeWallFollowing(ros::Publisher meroDistance, ros::Publisher calcErr, ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an)
{
    wallDistance = wallDist;
    maxSpeed = maxSp;
    direction = dir;
    P = pr;
    D = di;
    angleCoef = an;
    angleMin = 0;
    pubMessage = pub;
    meroDistancePub = meroDistance;
    calculatedError = calcErr;
}

NodeWallFollowing::~NodeWallFollowing() {}

//Publisher
void NodeWallFollowing::publishMessage()
{
    // double previousTime = ros::Time::now().toSec();
    // double currentTime = 0;

    //preparing message
    geometry_msgs::Twist msg;

    //for tests
    std_msgs::Float32 distancemsg;
    std_msgs::Float32 errormsg;

    //added for takeoff in simulation
    // if (sonarHeight < 0.5)
    // {
    //     msg.linear.z = 1;
    // }

    // if (largeChange)
    // {
    //     while (currentTime < (previousTime + 2.0))
    //     {
    //         if (sonarHeight < 0.5)
    //         {
    //             msg.linear.z = 1;
    //         }
    //         ROS_INFO_STREAM("---------------------4 sec sidha janchu---------------------------");
    //         msg.linear.x = 0.4;

    //         pubMessage.publish(msg);
    //         currentTime = ros::Time::now().toSec();
    //     }
    //     while (currentTime < (previousTime + 4.0))
    //     {
    //         if (sonarHeight < 0.5)
    //         {
    //             msg.linear.z = 1;
    //         }
    //         ROS_INFO_STREAM("---------------------gumdai chu aba---------------------------");
    //         msg.angular.z = 1; //+ve for anti clockwise
    //         pubMessage.publish(msg);
    //         currentTime = ros::Time::now().toSec();
    //     }
    // }
    // else
    // {
    if (sonarHeight < 0.5)
    {
        msg.linear.z = 1;
    }
    msg.angular.z = -1 * direction * (P * myErr + D * diffE) + angleCoef * (((PI * direction) / 2) - angleMin);
    // msg.linear.x = maxSpeed;

    //until 90 degree turn is complete if statement for that

    //otherwise follow the normal pattern

    if (turnComp)
    {
        if (distFront < wallDistance)
        {
            msg.linear.x = 0;
            turnComp = false;
        }
        else if (distFront < wallDistance * 2)
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
    }
    else
    {
        ROS_INFO_STREAM("Not Complete Not Complete");  /// HAS NOT BEEN TESTED wall dist 5 rakhera, should work in theory
        msg.linear.x = 0;
        if (distFront > (4 * wallDistance))
        {
            turnComp = true;
        }
    }
    pubMessage.publish(msg);
    // }

    //publish message
    errormsg.data = dist180;
    distancemsg.data = angleMin * 180 / PI;
    meroDistancePub.publish(distancemsg);
    calculatedError.publish(errormsg);
}

//Subscrber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    if (largeChange)
    {
        wallDistance = 3;
    }
    else
    {
        wallDistance = 1;
    }
    int size = msg->ranges.size();

    //variables with index of highest and lowest values in array
    // int minIndex = (size * (direction + 1) / 4);
    // int maxIndex = size * (direction + 3) / 4;
    int minIndex = 0;
    int maxIndex = 540;
    if (!turnComp)
    {
        int minIndex = 180;
        int maxIndex = 540;
    }

    int closestIndex = -1;
    double minVal = 999;
    overDistCount = 0;
    for (int i = minIndex; i < maxIndex; i++)
    {
        if (msg->ranges[i] > wallDistance + 2) /////YESMA MILAUNA PARCHA SURE change kasari milaune?
        {
            overDistCount++;
        }
        if ((msg->ranges[i] <= minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            minVal = msg->ranges[i];
            closestIndex = i;
        }
    }

    if (overDistCount > 5390)
    {
        largeChange = true;
        ROS_INFO_STREAM("---------------------Left ma bhwang cha---------------------------");
        // double previousTime = ros::Time::now().toSec();
    }
    else
    {
        ROS_INFO_STREAM("****************THIK THIK THIK THIK*********");
        // double previousTime = ros::Time::now().toSec();
        largeChange = false;
    }

    // ROS_INFO_STREAM("Value800 ---"<<msg->ranges[800]);
    ROS_INFO_STREAM("Index ---" << closestIndex);
    ROS_INFO_STREAM("Value ---" << msg->ranges[closestIndex]);
    // ROS_INFO_STREAM("----------------------------------");

    //calculation of angles from indexes and storing data to class variables
    angleMin = (closestIndex - (size / 2)) * msg->angle_increment;
    float distMin = 0;
    distMin = msg->ranges[closestIndex];
    distFront = msg->ranges[(size / 2)];

    ROS_INFO_STREAM("Front Distance ---" << distFront);
    diffE = (distMin - wallDistance) - myErr;
    myErr = distMin - wallDistance;
    dist180 = msg->ranges[180];
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
