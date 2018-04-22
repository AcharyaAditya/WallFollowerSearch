#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/PoseStamped.h"

class NodeWallFollowing
{
  public:
    /* Constructor:
   * pub   Publisher, which can send commands to robot.
   * wallDist Desired distance from the wall.
   * maxSp Maximum speed, that robot can go.
   * dir   1 for wall on the left side of the robot (-1 for the right side).
   * pr    P constant for PD controller.
   * di    D constant for PD controller.
   * an    Angle coefficient for P controller.
   */
    NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an);
    ~NodeWallFollowing();

    /* This method is generating commands for robot by
   * using data in variables (e, diffE, angleMin).
   */
    void publishMessage();

    /* This method finds minimum distance in data from
   * sensor and process these data into variables
   * (e, diffE, angleMin).
   * msg  Message, which came from robot and contains data from
   * laser scan.
   */
    void messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void sonarHeightCallback (const sensor_msgs::Range::ConstPtr &sonarMsg);

    void poseCallback (const geometry_msgs::PoseStamped::ConstPtr &poseMsg);

    //variables
    double wallDistance; // r_wall             desired distance to wall 
    double e;            // e_tn               difference between desired distance from the wall and actual distance
    double diffE;        // e_tn - e_tn-1      Derative element for PD controller.
    double maxSpeed;     // Maximum speed of the robot.
    double P;            // k_P                 constant for PD controller.
    double D;            // k_D                 constant for the PD controller
    double angleCoef;    // k_P2                coefficient for P controller
    int direction;       // d                   1 for wall on the right and -1 for wall on the left
    double angleMin;     // chi                 Angle, measured at the shortest distance
    double distFront;    // Distance measured at the front of the robot
    ros::Publisher pubMessage;

    double sonarHeight; //sonar height from gazebo
    double positionX;   //pose x
    double positionY;   //pose y
    double referencePosX=0; //for closure
    double referencePosY=0; //for closure
    int closureCounter=0;   //for closure
    double distChangeTime=0;  //for closure
};