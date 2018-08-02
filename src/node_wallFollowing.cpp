#include "../include/node_wallFollowing.h"
#include "std_msgs/Float32.h"
#include <math.h>
#define PI 3.141592

double NodeWallFollowing::myErr = 0;
double NodeWallFollowing::dist180 = 0;
bool NodeWallFollowing::largeChange = false;
double NodeWallFollowing::previousTime = 0;
bool NodeWallFollowing::turnComp = true;
int NodeWallFollowing::minIndex = 0;
int NodeWallFollowing::maxIndex = 540;
bool NodeWallFollowing::rightClear = true;
double NodeWallFollowing::rightDistance = 0;
// bool NodeWallFollowing::rightRecorded = false;

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
    //preparing message
    geometry_msgs::Twist msg;
    //for tests
    std_msgs::Float32 distancemsg;
    std_msgs::Float32 errormsg;

    //added for takeoff in simulation
    if (sonarHeight < 0.5)
    {
        msg.linear.z = 1;
    }

    msg.angular.z = -1 * direction * (P * myErr + D * diffE) + angleCoef * (((PI * direction) / 2) - angleMin);

    //until 90 degree turn is complete if statement for that
    //otherwise follow the normal pattern
    if (turnComp)
    {

        if ((distFront < (wallDistance)) || !rightClear)
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
            int minIndex = 180;
            int maxIndex = 540;
            msg.linear.x = 0.4 * maxSpeed;
        }
        else
        {
            msg.linear.x = maxSpeed;
        }
    }
    else
    {
        ROS_INFO_STREAM("Not Complete Not Complete"); /// HAS NOT BEEN TESTED wall dist 5 rakhera, should work in theory
        msg.linear.x = 0;
        if (distFront > (0.80 * rightDistance)) //90% of max distance to the right instead of this hard coded value Change this potentially??
        {
            turnComp = true;
        }
    }

    //publish message
    pubMessage.publish(msg);

    //define and publish messages to plot
    errormsg.data = dist180;
    distancemsg.data = angleMin * 180 / PI;
    meroDistancePub.publish(distancemsg);
    calculatedError.publish(errormsg);
}

//Subscrber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    int size = msg->ranges.size();
    int closestIndex = -1;
    double minVal = 999;
    float distMin = 0;
    double maxValueRight = 0;

    //variables with index of highest and lowest values in array
    // int minIndex = (size * (direction + 1) / 4);
    // int maxIndex = size * (direction + 3) / 4;
    if (!turnComp)
    {
        // wallDistance = 1/2;
        minIndex = 450;
        maxIndex = 540;
    }
    else
    {
        //////////////////////////////////////////////////////////////
        for (int j = 540; j < 1080; j++)
        {
            if ((msg->ranges[j] >= maxValueRight) && (msg->ranges[j] >= msg->range_min) && (msg->ranges[j] <= msg->range_max))
            {
                maxValueRight = msg->ranges[j];
            }
        }
        if (msg->ranges[900] > wallDistance)
        {
            rightDistance = msg->ranges[900];
        }
        else
        {
            rightDistance = maxValueRight;
        }
        //////////////////////////////////////////////////////////////
        // wallDistance = 1;
        minIndex = 0;
        maxIndex = 540;
    }

    if (msg->ranges[720] < wallDistance)
    {
        rightClear = false;
    }
    else
    {
        rightClear = true;
    }

    for (int i = minIndex; i < maxIndex; i++)
    {
        if ((msg->ranges[i] <= minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            minVal = msg->ranges[i];
            closestIndex = i;
        }
    }

    // ROS_INFO_STREAM("Value800 ---"<<msg->ranges[800]);
    ROS_INFO_STREAM("****************THIK THIK THIK THIK*********");
    ROS_INFO_STREAM("Index ---" << closestIndex);
    ROS_INFO_STREAM("Value ---" << msg->ranges[closestIndex]);

    //calculation of angles from indexes and storing data to class variables
    angleMin = (closestIndex - (size / 2)) * msg->angle_increment;
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
