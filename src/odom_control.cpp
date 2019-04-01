#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<math.h>
#include<angles/angles.h>
#include "proportional_controller/Goal.h"

/**
Proportional Controller for moving the robot
**/

class ProportionalController
{
public:
    ProportionalController()
    {
        //publisher object
        pub_ =n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
        //subscriber object
        poseSub_ = n_.subscribe("/odom",1000,&ProportionalController::updateCurrentPose,this);
        //subscriber object
        destSub_ = n_.subscribe("/destination",1000,&ProportionalController::populateDestination,this);
        
        //set initial values for flags and relevant variables
        execute = false; //this flag should be false untill user wants the robot to move to a goal

    }

    void updateCurrentPose(const nav_msgs::Odometry &msg) { 
        x = msg.pose.pose.position.x;
        y = msg.pose.pose.position.y;
        tf::poseMsgToTF(msg.pose.pose,pose);
        double yaw_angle = tf::getYaw(pose.getRotation());
        //*****for debugging*****
/*      double yaw_angle_deg= radToDeg(yaw_angle);   
        if(yaw_angle_deg<0){
            yaw_angle_deg=360+yaw_angle;
            //yaw_angle = angles::from_degrees(yaw_angle_deg);
        }
        ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"yaw_Deg: " << yaw_angle_deg);
        ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<" x: " << x<<", y: "<<y);
*/
        t=yaw_angle;
        if(execute){
            moveToGoal();
        }
        
    }

    void populateDestination(const proportional_controller::Goal &msg){
        xGoal = msg.x;
        yGoal = msg.y;
        tGoal =msg.theta;
        boundaryCheck();
        execute = msg.move;
        ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"(" << xGoal << ", " <<yGoal<<"); Status: "<<execute);  
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber poseSub_;
    ros::Subscriber destSub_;

    //Constants
    const static double PI = 3.14159265359;
    const static  double TOLERANCE = 0.1;
    const static double Kp = 1;
    //Defining the acceptable region for movement - to avoid hitting the walls
    const static  double xMin = -2;
    const static  double yMin = -2;
    const static  double xMax = 11;
    const static  double yMax = 11;

    //Max velocities
    const static double V_MAX_LINEAR = 0.25;
    const static double V_MAX_ANGULAR = 1.0;
    const static double V_MIN_ANGULAR = -1.0;

    //Variables to hold current position
    double x;
    double y;
    double t; //current orientation of the robot

    tf::Pose pose;
    //Variables to hold goal position
    double xGoal;
    double yGoal;
    double tGoal; //If user wants to define a certain orientation at the goal position

    //Variable to hold output cmd_vel
    geometry_msgs::Twist vel_msg;

    //Flags
    bool execute; //Main control

    //Functions

    double radToDeg(double angle){
        return angle*(180/PI);
    }
    double normalizeAngle(double angle){
        angle = fmod(angle +180,360);
        if(angle<0)
            angle +=360;
        return angle;
    }
    double calDistance(double x1, double y1, double x2, double y2){
        return sqrt(pow((x2-x1),2)+pow((y2-y1),2)); //distance to goal
    }

    double calAngle(double x1, double y1, double x2, double y2){
        return atan2((y2-y1),(x2-x1))-t; // angle to goal
    }

    void stop(){
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;        
            vel_msg.angular.z = 0;
            pub_.publish(vel_msg);
    }

    double boundaryCheck(){
        if(xGoal <= xMin) {xGoal = xMin;}
        if(yGoal <= yMin) { yGoal = yMin;}
        if(xGoal >= xMax) {xGoal = xMax;}
        if(yGoal >= yMax) { yGoal = yMax;}
    }

    void safteyManeuver(){
        if(x <= xMin) {xGoal = xMin;}
        else if(x >= xMax) {xGoal = xMax;}

        if(y <= yMin) { yGoal = yMin;}
        else if(y >= yMax) { yGoal = yMax;}

        execute = true;
    }

    double moveToGoal(){
        if(calDistance(x,y,xGoal,yGoal) > TOLERANCE && execute==true ){
            vel_msg.linear.x = calDistance(x,y,xGoal,yGoal);
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;        
            vel_msg.angular.z = Kp * calAngle(x,y,xGoal,yGoal);

            // Check for maximum velocity conditions
            if(vel_msg.linear.x>V_MAX_LINEAR){
               vel_msg.linear.x = V_MAX_LINEAR; 
            }
            if(vel_msg.angular.z>V_MAX_ANGULAR){
               vel_msg.angular.z = V_MAX_ANGULAR; 
            }
            else if (vel_msg.angular.z<V_MIN_ANGULAR){
               vel_msg.angular.z = V_MIN_ANGULAR;   
            }

            //boundaryCheck();
            pub_.publish(vel_msg);
        }
        else if(calDistance(x,y,xGoal,yGoal) <= TOLERANCE ){
            execute = false;
            stop();
            ROS_INFO_STREAM("Destination Reached!");
        }
           }



}; //End of Class

int main(int argc, char **argv) {

    //init ROS and register as a node
    ros::init(argc,argv, "Odom_Nav_Controller");

    ProportionalController navController;

    ros::spin();

    return 0;


}
