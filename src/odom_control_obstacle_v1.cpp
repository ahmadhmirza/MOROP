#include<ros/ros.h>
#include <algorithm> 
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<math.h>
#include<angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include "proportional_controller/Goal.h"
#include "proportional_controller/yawControlService.h"

/**
Proportional Controller for moving the robot
**/

class ProportionalController
{
public:
    ProportionalController()
    {
        //publisher object for publishing velocity commands
        pub_ =n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
        //subscriber object to odometry topic
        poseSub_ = n_.subscribe("/odom",1000,&ProportionalController::updateCurrentPose,this);
        //subscriber object for receiving goal information from user
        destSub_ = n_.subscribe("/destination",1000,&ProportionalController::populateDestination,this);        
        //Create a subscriber object
        ObstacleSub_ = n_.subscribe("/scan", 1000, &ProportionalController::obstacleDetection, this); 
        //set initial values for flags and relevant variables
        execute = false; //this flag should be false untill user wants the robot to move to a goal
        executeTurn = false;
        obstacleStatus = 0;
    }

    void updateCurrentPose(const nav_msgs::Odometry &msg) { 
        currentPose.x = msg.pose.pose.position.x;
        currentPose.y = msg.pose.pose.position.y;
        tf::poseMsgToTF(msg.pose.pose,pose);
        double yaw_angle = tf::getYaw(pose.getRotation());
        currentPose.theta=yaw_angle;
		//ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"yaw_Deg: " << yaw_angle);
        if(execute){
            moveToGoal();
        }
        
    }

    void populateDestination(const proportional_controller::Goal &msg){
        goalPose.x = msg.x;
        goalPose.y = msg.y;
        goalPose.theta =degToRad(msg.theta);
        execute = msg.move;
        backupGoalPose = goalPose;
        ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"(" << goalPose.x << ", " <<goalPose.y<<"); Status: "<<execute);  

    }

     void obstacleDetection(const sensor_msgs::LaserScan &msg) {        
        double range_ahead = msg.ranges[0];
        double range_left = msg.ranges[15];
        double range_right = msg.ranges[345];
        
        bool turnStatus = false;
        int turnDirection;
        //ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"l: " << range_left<<" f: " << range_ahead<<" r: " << range_right);

        if( calDistance(currentPose.x,currentPose.y,backupGoalPose.x,backupGoalPose.y) > 1 ) {

        if(execute && range_ahead <= 1 || range_left<=1.4 || range_right<=1.4){
            ROS_INFO_STREAM("Obstacle detected");
            obstacleStatus = 1;
            //tempGoalPose = goalPose;
            //obstacleDetected = true;
            stop();
            execute = false;
            executeTurn = false;
            turnDirection = calculateTurnDirection(msg);
            ROS_INFO_STREAM("turn direction recieved{-2, -1, 0 , 1, 2}: "<< turnDirection );
        }
        }

        if(obstacleStatus == 1) {  //If an obstacle is detected then get the turn angle from the service
            if(turnDirection == -2){
                goalPose = distanceToCartesian(1,currentPose.theta + degToRad(75),currentPose.x, currentPose.y);
                goalPose.theta = currentPose.theta + degToRad(75);
            }
            else if(turnDirection == -1){
                goalPose = distanceToCartesian(1,currentPose.theta + degToRad(45),currentPose.x, currentPose.y);
                goalPose.theta = currentPose.theta + degToRad(45);
            }
            else if(turnDirection == 0){
                goalPose = distanceToCartesian(1,currentPose.theta,currentPose.x, currentPose.y);
                goalPose.theta = currentPose.theta;
            }
            else if(turnDirection == 1){
                goalPose = distanceToCartesian(1,currentPose.theta - degToRad(45),currentPose.x, currentPose.y);
                goalPose.theta = currentPose.theta - degToRad(45);
            }
            else if(turnDirection == 2){
                goalPose = distanceToCartesian(1,currentPose.theta - degToRad(75),currentPose.x, currentPose.y);
                goalPose.theta = currentPose.theta - degToRad(75);
            }
                obstacleStatus = 2; // turn angle recieved
        }

        if(obstacleStatus == 2){
            if(turnToGoal(goalPose) == true){
                obstacleStatus = 3; //turn complete time to move to interim waypoint
            }
            else if(turnToGoal(goalPose) == false){
                 ROS_INFO_STREAM("Turning to face interim waypoint");
            }
        }

        if (obstacleStatus == 3){
            execute = true;
        }

        if(obstacleStatus == 4){
            goalPose = backupGoalPose;
            execute = true;
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
	poseInfo currentPose;
    tf::Pose pose;
    //Variables to hold goal position
    poseInfo goalPose;
    poseInfo backupGoalPose;
    poseInfo tempGoalPose;
    //Variables for obstacle avoidance
    poseInfo obstacleClearancePose;
    //Variable to hold output cmd_vel
    geometry_msgs::Twist vel_msg;
    //Flags
    bool execute; //Main control
    bool executeTurn;
    int obstacleStatus;
//***************Functions******************
    double radToDeg(double angle){
        return angle*(180/PI);
    }
    double degToRad(double angle){
        return angle*(PI/180);
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
        return atan2((y2-y1),(x2-x1))-currentPose.theta; // angle to goal
    }


/*  h: distance in meters
    angle: current yaw angle
    x,y : current co-ordinates
    returns co-ordinates corresponding to the given distance
*/
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
    }

/*
    double x[] : array argument
    int size : of the array
    returns average of all the elements in the array
*/
	double calAverage(double x[],int size) {
		double avg;
  		int i, sum = 0;          

   		for (i = 0; i < size; i++) {
      		sum = sum+x[i];
   		}
   		avg = double(sum) / size;

		return avg;
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
        if(goalPose.x <= xMin) {goalPose.x = xMin;}
        if(goalPose.y <= yMin) { goalPose.y = yMin;}
        if(goalPose.x >= xMax) {goalPose.x = xMax;}
        if(goalPose.y >= yMax) { goalPose.y = yMax;}
    }

    void safteyManeuver(){  //Not implemented yet
        if(currentPose.x <= xMin) {goalPose.x = xMin;}
        else if(currentPose.x >= xMax) {goalPose.x = xMax;}

        if(currentPose.y <= yMin) { goalPose.y = yMin;}
        else if(currentPose.y >= yMax) { goalPose.y = yMax;}

        execute = true;
    }

    double moveToGoal(){
        if(calDistance(currentPose.x,currentPose.y,goalPose.x,goalPose.y) > TOLERANCE && execute==true ){
            vel_msg.linear.x = calDistance(currentPose.x,currentPose.y,goalPose.x,goalPose.y);
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;        
            vel_msg.angular.z = Kp * calAngle(currentPose.x,currentPose.y,goalPose.x,goalPose.y);

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

        else if(calDistance(currentPose.x,currentPose.y,goalPose.x,goalPose.y) <= TOLERANCE){
                stop();
                ROS_INFO_STREAM("Destination Reached!");
                //executeTurn = true;
                if(obstacleStatus == 3){
                    obstacleStatus =4;
                }
                else{
                    turnToGoal(goalPose);
                }
                //ROS_INFO_STREAM("setting final angle!");    
        }
                  
    } //End of moveToGoal()


    bool turnToGoal(poseInfo x){
        //if (executeTurn == true){ //setting yaw
                if(execute && x.theta - currentPose.theta >0.08 || x.theta - currentPose.theta < -0.08 ){
                    turnRight();
                    ROS_INFO_STREAM("angle difference: "<< x.theta - currentPose.theta);
                    return false;
                }
                else if(execute && x.theta - currentPose.theta <0.08 || x.theta - currentPose.theta > -0.08 ){
                    execute = false;
                    stop();
                    ROS_INFO_STREAM("turn complete facing target!");
                    return true;
                }                           
            //}

    }

    //*************Functions for obstacle detection and avoidance*************************************
    
    int calculateTurnDirection(const sensor_msgs::LaserScan &msg){

        ros::ServiceClient client = n_.serviceClient<proportional_controller::yawControlService>("turn_service");
        proportional_controller::yawControlService srv;
		int turnDirection;	
			    for(int i=0; i<360;i++){
      			    srv.request.range[i] = msg.ranges[i];
    		    }

                if (client.call(srv))
                {   
            	    turnDirection = srv.response.turnControl;
                }
                else
                {
                    ROS_ERROR("Failed to call service");
                }
        return turnDirection;

    }

}; //End of Class

int main(int argc, char **argv) {

    //init ROS and register as a node
    ros::init(argc,argv, "Odom_Nav_Controller");

    ProportionalController navController;

    ros::spin();

    return 0;


}
