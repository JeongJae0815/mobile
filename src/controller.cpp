#include <p3dx.h>
#include <iostream>
#include <stdio.h>
#include <boost/format.hpp>
bool bExit;
char key, buffer;

void exitCallback(const std_msgs::Bool msg){
	bExit = msg.data;
	ROS_INFO("exit signal detected");
	
}

void keyCallback(const std_msgs::Char msg){
	if(msg.data != SPEED_1 && msg.data != SPEED_2 && msg.data != SPEED_3 && msg.data != SPEED_4 && msg.data != SPEED_5)
		buffer = msg.data;

	key = msg.data;
	ROS_INFO("Input Received: [%c]", msg.data);
	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "P3DX_Controller");
	ros::NodeHandle nh, nKey, nExit;
	ros::Publisher pub = nh.advertise<std_msgs::String>("/mobile/write", 10);
	ros::Subscriber subKey = nKey.subscribe("keyInput", 100, keyCallback);
	ros::Subscriber subExit = nExit.subscribe("exitSignal", 10, exitCallback);
	std_msgs::String msg;
	float speed = BASE_SPEED;
	// Make the robot stop (robot perhaps has a speed already)
	
	msg.data="PIE";
	pub.publish(msg);
	ros::spinOnce();
	std::string data;
	data=(boost::format("PV%d,%dE") % (100) %(0)).str();

	int linear_velocity=100;
	int angular_velocity=20;
	bExit = false;
	key = STOP_SPACE;
	while(ros::ok() && !bExit){
		switch(key){
		case FORWARD_W:
			data=(boost::format("PV%d,%dE") % (linear_velocity) %(0)).str();
			msg.data=data;
			pub.publish(msg);
			break;
			
		case LEFT_A:
			data=(boost::format("PV%d,%dE") % (linear_velocity) %(angular_velocity)).str();
			msg.data=data;
			pub.publish(msg);
			break;
			
		case BACKWARD_S:
			data=(boost::format("PV-%d,%dE") % (linear_velocity) %(0)).str();
			msg.data=data;
			pub.publish(msg);
			break;
			
		case RIGHT_D:
			data=(boost::format("PV%d,-%dE") % (linear_velocity) %(angular_velocity)).str();
			msg.data=data;
			pub.publish(msg);
			break;
			
		case COUNTERCLKWISE_Q:
			data=(boost::format("PV%d,%dE") % (0) %(angular_velocity)).str();
			msg.data=data;
			pub.publish(msg);
			break;
			
		case CLKWISE_E:
			data=(boost::format("PV%d,-%dE") % (0) %(angular_velocity)).str();
			msg.data=data;
			pub.publish(msg);
			break;
			
		case SPEED_1:
			linear_velocity=100;
			angular_velocity=10;
			key = buffer;
			break;

		case SPEED_2:
			linear_velocity=200;
			angular_velocity=20;
			key = buffer;
			break;

		case SPEED_3:
			linear_velocity=300;
			angular_velocity=30;

			key = buffer;
			break;

		case SPEED_4:
			linear_velocity=400;
			angular_velocity=40;

			key = buffer;
			break;

		case SPEED_5:
			linear_velocity=500;
			angular_velocity=50;

			key = buffer;
			break;

		case STOP_SPACE:
			msg.data="PV0,0E";
			pub.publish(msg);
			break;
		}
		
		ros::spinOnce();
		ros::Rate(100).sleep();
	}

  //system("rosnode kill -a");
	return 0;

}
