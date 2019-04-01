#include "ros/ros.h"
#include "proportional_controller/yawControlService.h"

const static double PI = 3.141592654;

double calAverage(double x[],int size) {
		double avg;
  		int i, sum = 0;          

   		for (i = 0; i < size; i++) {
      		sum = sum+x[i];
   		}
   		avg = double(sum) / size;

		return avg;
	}

bool navService(proportional_controller::yawControlService::Request  &req,
         proportional_controller::yawControlService::Response &res)
{ 
    double ranges[360];
    for(int i=0; i<360;i++){
      ranges[i] = req.range[i];
    }
    double sensorData[30];
  	double avgSensorData_straight;

		double avgSensorData[5];

		double avgSensorData_left_1;
		double avgSensorData_left_2;
		double avgSensorData_right_1;
		double avgSensorData_right_2;
    int j;
		int turnDirection=0; //-2,-1,0,1,2

			//prepare laserscan data to calculate weighted averages
		for(int i =0; i<14; i++ ){
			j= i+346;
			sensorData[i]=ranges[j];	
		}
		for(int i =14; i<30; i++ ){
			j= i-14;;
			sensorData[i]=ranges[j];	
		}
    avgSensorData[2]=calAverage(sensorData,30); //avgSensorData_straight

			for(int i =0; i<30; i++){
				j=i+16;
				sensorData[i]=ranges[j];
			}

			avgSensorData[1] = calAverage(sensorData,30);  //avgSensorData_left_1

			for(int i =0; i<30; i++){
				j=i+46;
				sensorData[i]=ranges[j];
			}
			avgSensorData[0] = calAverage(sensorData,30); //avgSensorData_left_2

			for(int i =0; i<30; i++){
				j=i+315;
				sensorData[i]=ranges[j];
			}
			avgSensorData[3] = calAverage(sensorData,30);//avgSensorData_right_1
			

			for(int i =0; i<30; i++){
				j=i+285;
				sensorData[i]=ranges[j];
			}
			avgSensorData[4] = calAverage(sensorData,30); //avgSensorData_right_2

			ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"l_2: " << avgSensorData[0]<<" l_1: " << avgSensorData[1]<<" st: " 
			<< avgSensorData[2]<<" r_1: "<< avgSensorData[3]<<" r_2: "<< avgSensorData[4]);
			
/*			double* i1; 
   			i1 = std::min_element(avgSensorData + 0, avgSensorData + 5); 
			ROS_INFO_STREAM(std::setprecision(2)<<std::fixed<<"minimum is:  " << *i1);*/


/*			for(int i =0; i<5; i++){

				if(avgSensorData[i] < temp){
					temp = avgSensorData[i];
					index= i;
				}
			}*/
			double rightAverage = (avgSensorData[3]+avgSensorData[4])/2;
			double leftAverage =  (avgSensorData[0]+avgSensorData[1])/2;
			if( leftAverage > rightAverage ) {
				if(avgSensorData[3] < avgSensorData[4]){
					ROS_INFO_STREAM("Should turn right 1");
					turnDirection =1;
				}
				else if(avgSensorData[4] < avgSensorData[3]){
					ROS_INFO_STREAM("Should turn right 2");
					turnDirection =2;
				}
				else if(avgSensorData[4] == avgSensorData[3]){
					ROS_INFO_STREAM("Should turn right 1");
					turnDirection =1;
				}
			}
			else if( leftAverage < rightAverage ) {
				if(avgSensorData[1] < avgSensorData[2]){
					ROS_INFO_STREAM("Should turn left 1");
					turnDirection =-1;
			}
				else if(avgSensorData[2] < avgSensorData[1]){
					ROS_INFO_STREAM("Should turn left 2");
					turnDirection =-2;
				}
				else if(avgSensorData[1] == avgSensorData[2]){
					ROS_INFO_STREAM("Should turn right 1");
					turnDirection =-11;
				}
			}


  res.turnControl = turnDirection;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turn_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("turn_service", navService);
  ROS_INFO("...turn Server Ready...");
  ros::spin();

  return 0;
}