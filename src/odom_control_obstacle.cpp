#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Pose.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<math.h>
#include<angles/angles.h>
#include <sensor_msgs/LaserScan.h>
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
        //Create a subscriber object
        ObstacleSub_ = n_.subscribe("/scan", 1000, &ProportionalController::obstacleDetection, this);        
        //set initial values for flags and relevant variables
        execute = false; //this flag should be false untill user wants the robot to move to a goal
        obstacle = 0;
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

//***LaserScan data for TurtleBot3***
    // 360 degrees scan - 0 straight ahead counter clockwise to 360 Degrees
    // 360 elements in the laser scan array
    // range_min = 0.1199m  -  range_max = 3.5m
    void obstacleDetection(const sensor_msgs::LaserScan &msg) {        
        double range_ahead = msg.ranges[0];
        double range_left = msg.ranges[15];
        double range_right = msg.ranges[345];

        int goalDirection = goalQuadrant(x,y,xGoal,yGoal);

        double distanceToUserGoal = calDistance(x,y,xGoal,yGoal);

        ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"l: " << range_left<<" f: " << range_ahead<<" r: " << range_right);
        if(range_ahead <= 1 || range_left<=1.4 || range_right<=1.4)
        {
            if(execute && obstacle == 0 && distanceToUserGoal>1 ){ 
            //When an obstacle is first detected, stop the movement and save the 
            //goal information.
                stop();
                execute = false;
                userGoalTemp.x = xGoal;
                userGoalTemp.y = yGoal;
                userGoalTemp.theta = tGoal;
                ROS_INFO_STREAM("Stop");
                obstacle=1;
            }
            else if(execute && obstacle == 5 ){ 
            //When an obstacle is first detected, stop the movement and save the 
            //goal information.
                stop();
                execute = false;
                ROS_INFO_STREAM("again obstacle");
            }
            if(obstacle==1 || obstacle == 5){
                if(goalDirection=1){
                    turnRight();
                    obstacle = 2;
                }
                else if(goalDirection=2){
                    turnLeft();
                    obstacle = 2;
                }
                else if(goalDirection=3){
                    turnLeft();
                    obstacle = 2;
                }
                else if(goalDirection=4){
                    turnRight();
                    obstacle = 2;
                }
            }


        }
        if(range_ahead > 1 && range_left > 1.1 && range_right > 1.1 && obstacle> 0 )
        {
            if(obstacle ==2){
                stop();
                struct poseInfo interim;
                interim = distanceToCartesian(1.0,t,x,y);
                xGoal = interim.x;
                yGoal = interim.y;
                obstacle = 3;
                ROS_INFO_STREAM("New interim goal set");          
            }
            
            if(obstacle ==3){
                execute = true;
                obstacle = 5;
            }
            if(obstacle ==10 && !execute){
                xGoal = userGoalTemp.x;
                yGoal = userGoalTemp.y;
                execute = true;
                obstacle = 0;
            }
                 
        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber poseSub_;
    ros::Subscriber destSub_;
    ros::Subscriber ObstacleSub_;

//Struct to hold pose informations required for navigation
    struct poseInfo{
        double x;
        double y;
        double theta;
    };

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
    //Variable to hold goal position temporarily in case of an obstacle
     poseInfo userGoalTemp;

    //Variable to hold output cmd_vel
    geometry_msgs::Twist vel_msg;

    //Flags
    bool execute; //Main control
    int obstacle;

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

/*    h: distance in meters
    x,y : current co-ordinates
    angle: current yaw angle*/
    struct poseInfo distanceToCartesian(double h, double angle,double x,double y){
        struct poseInfo coordinates;
        double p = h * sin(angle);
        double b = h * cos(angle);
        coordinates.x = x+b;
        coordinates.y = y+p;
        coordinates.theta = 0;
        return coordinates;

    }

    int goalQuadrant(double x1, double y1, double x2, double y2){
        double xQ = (x2-x1);
        double yQ = (y2-y1);
        if(xQ>0 && yQ>0){
            return 1;
        }
        if(xQ<0 && yQ>0){
            return 2;
        }
        if(xQ<0 && yQ<0){
            return 3;
        }
        if(xQ>0 && yQ<0){
            return 4;
        }
//******for straight motion******
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
    void turnLeft(){
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;        
            vel_msg.angular.z = 0.4;
            pub_.publish(vel_msg);
    }
    void turnRight(){
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;        
            vel_msg.angular.z = -0.4;
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
            if(obstacle == 5){
                obstacle = 10;
            }
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
