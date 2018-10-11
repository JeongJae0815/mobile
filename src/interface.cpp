#include <p3dx.h>

void initTermios(int echo){
    tcgetattr(0, &old); /* grab old terminal i/o settings */
    new1 = old; /* make new settings same as old settings */
    new1.c_lflag &= ~ICANON; /* disable buffered i/o */
    new1.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
    tcsetattr(0, TCSANOW, &new1); /* use these new terminal i/o settings now */

}

/* Restore old terminal i/o settings */
void resetTermios(){
    tcsetattr(0, TCSANOW, &old);

}

/* quit the program cleanly and close ros */
void quit(){
	resetTermios(); /* reset the terminal to old settings */
	ros::shutdown(); /*shutdown ros */
	exit(1); /* exit the system */

}

/* Handles top level control as usual */
int main(int argc, char **argv){
  /* initialize the ros node */
	ros::init(argc, argv, "P3DX_User_Interface");
	ros::NodeHandle nKey, nExit;
	ros::Publisher pubExit = nExit.advertise<std_msgs::Bool>("exitSignal", 10);
	ros::Publisher pubKey = nKey.advertise<std_msgs::Char>("keyInput", 100);

	std_msgs::Bool msgExit;
	std_msgs::Char msgChar, select;
	char buffer = STOP_SPACE;

	msgExit.data = false;

  /* change the terminal input settings */ 
	initTermios(0);

  /* greet user and display selection options */
	std::cout << "P3DX Controller Interface" << std::endl
		<< "[w] go forward" << std::endl
		<< "[s] go backward" << std::endl
		<< "[a] turn left" << std::endl
		<< "[d] turn right" << std::endl
		<< "[q] spin counterclockwise" << std::endl
		<< "[e] spin clockwise" << std::endl
		<< "[space] stop" << std::endl;

	int speed = 2;

  /* loop to handle user input */
	while(ros::ok() && !msgExit.data){
		//select = std::cin.get();	/* use standard input to select program to run */
		select.data = std::cin.get();

		if(select.data == SPEED_1 && speed != 1){
			pubKey.publish(select);
			speed = 1;

		}else if(select.data == SPEED_2 && speed != 2){
			pubKey.publish(select);
			speed = 2;

		}else if(select.data == SPEED_3 && speed != 3){
			pubKey.publish(select);
			speed = 3;

		}else if(select.data == SPEED_4 && speed != 4){
			pubKey.publish(select);
			speed = 4;

		}else if(select.data == SPEED_5 && speed != 5){
			pubKey.publish(select);
			speed = 5;

		}else{
			if(select.data != SPEED_1 && select.data != SPEED_2 && select.data != SPEED_3 && select.data != SPEED_4 && select.data != SPEED_5)
				buffer = select.data;
				
			msgChar.data = buffer;
	
			if(msgChar.data == ESCAPE){
				msgExit.data = true;
				pubExit.publish(msgExit);
        //quit();
	
			}else
				pubKey.publish(msgChar);
		}

		ros::spinOnce();

	}

	return 1;
}
