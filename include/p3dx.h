#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <std_msgs/Char.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#define FORWARD_W 0x77
#define LEFT_A 0x61
#define BACKWARD_S 0x73
#define RIGHT_D 0x64
#define COUNTERCLKWISE_Q 0x71
#define CLKWISE_E 0x65
#define SPEED_1 0x31
#define SPEED_2 0x32
#define SPEED_3 0x33
#define SPEED_4 0x34
#define SPEED_5 0x35
#define STOP_SPACE 0x20
#define ESCAPE 0x1B

#define BASE_SPEED 0.1
#define PI 3.14159265359

static struct termios new1, old;
