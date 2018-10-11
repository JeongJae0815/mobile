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
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);
	ros::Subscriber subKey = nKey.subscribe("keyInput", 100, keyCallback);
  ros::Subscriber subExit = nExit.subscribe("exitSignal", 10, exitCallback);
	geometry_msgs::Twist msg;
	float speed = BASE_SPEED;
	// Make the robot stop (robot perhaps has a speed already)
	msg.linear.x = 0;
	msg.angular.z = 0;
	pub.publish(msg);
	ros::spinOnce();
	
	bExit = false;
	key = STOP_SPACE;

	while(ros::ok() && !bExit){
		switch(key){
		case FORWARD_W:
			msg.linear.x = speed;
			msg.angular.z = 0;
			pub.publish(msg);
			break;
			
		case LEFT_A:
      msg.linear.x = speed;
      msg.angular.z = PI / 6;
			pub.publish(msg);
			break;
			
		case BACKWARD_S:
			msg.linear.x = -1 * speed;
			msg.angular.z = 0;
			pub.publish(msg);
			break;
			
		case RIGHT_D:
      msg.linear.x = speed;
      msg.angular.z = -1 * PI / 6;
			pub.publish(msg);
			break;
			
		case COUNTERCLKWISE_Q:
			msg.linear.x = 0;
      msg.angular.z = PI / 12;
      //msg.angular.z = PI / 5;
			pub.publish(msg);
			break;
			
		case CLKWISE_E:
			msg.linear.x = 0;
      msg.angular.z = -1 * PI / 12;
      //msg.angular.z = -1 * PI / 5;
			pub.publish(msg);
			break;
			
		case SPEED_1:
			speed = BASE_SPEED * 0.5;
			key = buffer;
			break;

		case SPEED_2:
			speed = BASE_SPEED;
			key = buffer;
			break;

		case SPEED_3:
      speed = BASE_SPEED * 1.5;
			key = buffer;
			break;

		case SPEED_4:
      speed = BASE_SPEED * 2;
			key = buffer;
			break;

		case SPEED_5:
      speed = BASE_SPEED * 2.5;
			key = buffer;
			break;

		case STOP_SPACE:
		default:
			msg.linear.x = 0;
			msg.angular.z = 0;
			pub.publish(msg);
			break;
		}
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}

  //system("rosnode kill -a");
	return 0;

}
