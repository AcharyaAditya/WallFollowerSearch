#include "../include/node_wallFollowing.h"
#include "std_msgs/Float32.h"
#include <math.h>
#define PI 3.141592

double NodeWallFollowing::myErr = 0;
double NodeWallFollowing::dist180 = 0;
int NodeWallFollowing::minIndex = 0;
int NodeWallFollowing::maxIndex = 540;
int NodeWallFollowing::overDistCount = 0;

bool NodeWallFollowing::rightClear = true;
bool NodeWallFollowing::rightTurnRequired = false;
double NodeWallFollowing::rightDistance = 0;
double NodeWallFollowing::leftDistance = 0;

bool NodeWallFollowing::leftTurnRequired = false;

float NodeWallFollowing::distFront = 0;

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

    msg.angular.z = -1 * direction * (P * myErr + D * diffE) + angleCoef * (((PI * direction) / 2) - angleMin); //+ve for anti clockwise

    /* until 90 degree turn is complete if statement 
    for that otherwise follow the normal pattern */
    if (rightTurnRequired)
    {
        ROS_INFO_STREAM("****************RIGHT TURN REQUIRED*********");
        msg.linear.x = 0; //when turning right, linear velocit is zero. We get to this situation only if it clears the first if condition in the else part of this
        if (distFront > (0.80 * rightDistance)) //80% of max distance to the right instead of this hard coded value Change this potentially??
        {
            rightTurnRequired = false;
        }
    }
    else if (leftTurnRequired)
    {
        //left turn complete check and set leftTurnRequired to false
        ROS_INFO_STREAM("****************LEFT TURN REQUIRED*********");
        msg.linear.x = maxSpeed * 0.5;
        if (overDistCount < 250)
        {
            leftTurnRequired = false;
        }
    }
    else
    {
        if ((distFront < (wallDistance)) || !rightClear)
        {
            msg.linear.x = 0;
            rightTurnRequired = true; //this is the only condition that sets right turn required as true
            leftTurnRequired = false; // no left turns when we want right, aslo right turn overrides everything
        }
        else if (overDistCount > 250)
        {
            leftTurnRequired = true; //this is the only condition that sets left turn required as true
            rightTurnRequired = false; // no right turns when we want left, aslo we get here only of right side is clear
        }
        else if (distFront < wallDistance * 2)
        {
            msg.linear.x = 0.5 * maxSpeed; //reducing speed so that turn can be made smoother
        }
        else if (fabs(angleMin) < (-1.75))
        {
            msg.linear.x = 0.4 * maxSpeed; // reducing spped to keep tracking things behind me on the left side
        }
        else
        {
            msg.linear.x = maxSpeed; // top speed when everything is clear
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
    overDistCount = 0;

    //variables with index of highest and lowest values in array
    // int minIndex = (size * (direction + 1) / 4);
    // int maxIndex = size * (direction + 3) / 4;
    if (rightTurnRequired)
    {
        //only look at the forward 22.5 degrees until the turn is completed
        minIndex = 450;
        maxIndex = 540;
    }
    else if (leftTurnRequired)
    {
        //only look at the backward 22.5 degrees until the turn is completed
        minIndex = 0;
        maxIndex = 540;
        leftDistance = msg->ranges[180];
    }
    else
    {
        //This loops gives the max range in the right
        for (int j = 540; j < 1080; j++)
        {
            if ((msg->ranges[j] >= maxValueRight) && (msg->ranges[j] >= msg->range_min) && (msg->ranges[j] <= msg->range_max))
            {
                maxValueRight = msg->ranges[j];
            }
        }

        //Setting up right distance to check for turn completions
        if (msg->ranges[900] > wallDistance)
        {
            /*
            This check is for bigger rooms where the turn
            completion check needs to be reduced to an acceptable limit
            */
            if (msg->ranges[900] < 5)
            {
                /*if the room is small enough then we just take 
                the value ar the 90 degree mark on the right 
                as the reference value
                */
                rightDistance = msg->ranges[900];
            }
            else
            {
                rightDistance = 5;
            }
        }
        else
        {
            /*if the enclosed area is smaller than the assigned wall 
            distance value it is required to turn until the opening is 
            seen hence we look for the highest value area on the right
            */
            rightDistance = maxValueRight;
        }
        //////////////////////////////////////////////////////////////

        //look at the entire left half after the turn has been completed
        minIndex = 0;
        maxIndex = 540;
    }

    /*
    This check is to determine if there is enough room to turn, 
    and hence the values on the right needs to be checked
    A right turn which is more than 90 degrees will use this condition 
    */
    if (msg->ranges[720] < wallDistance)
    {
        rightClear = false;
    }
    else
    {
        rightClear = true;
    }

    /*This loop gives the distance to to closest obstacle and
    its index in the range array so that the angle can be extracted
    */
    for (int i = minIndex; i < maxIndex; i++)
    {
        if ((msg->ranges[i] <= minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            minVal = msg->ranges[i];
            closestIndex = i;
        }
    }

    // to determine that there is a large change and there is a gap or a dent on the left, trigger for left turns
    for (int i = 180; i < 540; i++)
    {
        if (msg->ranges[i] > (0.9))
        {
            overDistCount++;
        }
    }
    ROS_INFO_STREAM("Over Dist count ======================" << overDistCount);
    //turn until right front is visible
    /*
    Check if it is stuck
    */

    /*
    Fix Left turn
    */

    /*
    fix slight right turn
    */

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
