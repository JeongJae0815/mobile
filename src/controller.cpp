#include <p3dx.h>

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
	
	msg.data="base speed";
	pub.publish(msg);
	ros::spinOnce();
	
	bExit = false;
	key = STOP_SPACE;

	while(ros::ok() && !bExit){
		switch(key){
		case FORWARD_W:
			msg.data="forward";
			pub.publish(msg);
			break;
			
		case LEFT_A:
			msg.data="left";
			pub.publish(msg);
			break;
			
		case BACKWARD_S:
			msg.data="backward";
			pub.publish(msg);
			break;
			
		case RIGHT_D:
			msg.data="right";
			pub.publish(msg);
			break;
			
		case COUNTERCLKWISE_Q:
			msg.data="count_clock_q";
			pub.publish(msg);
			break;
			
		case CLKWISE_E:
			msg.data="count_clock_e";
			pub.publish(msg);
			break;
			
		case SPEED_1:
			//speed = BASE_SPEED * 0.5;
			key = buffer;
			break;

		case SPEED_2:
			//speed = BASE_SPEED;
			key = buffer;
			break;

		case SPEED_3:
	  	        //speed = BASE_SPEED * 1.5;
			key = buffer;
			break;

		case SPEED_4:
	  	        //speed = BASE_SPEED * 2;
			key = buffer;
			break;

		case SPEED_5:
		        //speed = BASE_SPEED * 2.5;
			key = buffer;
			break;

		case STOP_SPACE:
			msg.data="stop space";
			pub.publish(msg);
			break;
		}
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}

  //system("rosnode kill -a");
	return 0;

}
