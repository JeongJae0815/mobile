/*******
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#define MAX_DIST 4294967295
#define ODO_DIST 200
serial::Serial ser;
struct Odometry{
    double right_wheel;
    double left_wheel;
    char odo_flag;
};
int cal_diff(unsigned long current, unsigned long past){
    long diff=current-past;
    if(abs(diff)>=1000){
        if(diff>0){
            return MAX_DIST-diff;
        }
        else{
            return diff-MAX_DIST;
        }
    }
    else{
        return diff;
    }
}
void write_callback(const std_msgs::String::ConstPtr& msg){
//    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}
int write_buffer(const std_msgs::String msg){
    static unsigned char r_pkt[20];
    static int r_pkt_idx=0;
    int i;
    unsigned long len=msg.data.size();   
    unsigned char battery_level;
    int dist=0;
    int diff_dist=0;
    static int tmp_dist=0;
    static int odo_tmp=0;
    //ROS_INFO("length = %lu",len);
    if (msg.data[len-1]==(char)'E'){
        for (i=0;i<len;i++){
            r_pkt[r_pkt_idx]=msg.data[i];
            r_pkt_idx++;
        }
        //ROS_INFO("r_pkt_idx=%d,length= %lu",r_pkt_idx, len);
        unsigned long travel_distance_right_wheel,travel_distance_left_wheel;
        unsigned long temp;
        if(r_pkt_idx==12){
            travel_distance_right_wheel=0;
            for(i=0;i<4;i++){
                temp=r_pkt[i+1];
                travel_distance_right_wheel |=temp<<((3-i)*8);
            } 
            travel_distance_left_wheel=0;
            for(i=0;i<4;i++){
                temp=r_pkt[i+5];
                travel_distance_left_wheel |=temp<<((3-i)*8);
            } 
            battery_level=r_pkt[10];
            static long tmp_right=travel_distance_right_wheel;
            static long tmp_left=travel_distance_left_wheel;
            static long tmp=0;
            int diff_right=0;
            int diff_left=0;
            diff_right=cal_diff(travel_distance_right_wheel,tmp_right);
            tmp_right=travel_distance_right_wheel

            ROS_INFO("dist_r : %8ld, dist_l : %8ld, Battery : %3d",MAX_DIST,diff_right,battery_level);
            r_pkt_idx=0;
            dist=(current_distance_left_wheel+current_distance_right_wheel)/2;
            //ROS_INFO("%d",dist);
            if(tmp_dist){
               diff_dist=dist-tmp_dist; 
               tmp_dist=dist;
               //ROS_INFO("diff= %d",diff_dist);
               odo_tmp+=diff_dist;
               if (abs(odo_tmp)>ODO_DIST){
                   odo_tmp=1;
                  // ROS_INFO("odotmp = %d",odo_tmp);
                   return odo_tmp;
               }
            }
            else tmp_dist=dist;
            
        }
        else{
            r_pkt_idx=0;
        }
    }
    else{
        for(i=0;i<len;i++){
            r_pkt[r_pkt_idx]=msg.data[i];
            r_pkt_idx++;
        }
    }
    return 0; 
} 
int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    
    ros::Subscriber write_sub = nh.subscribe("/mobile/write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/mobile/read", 1000);
    ros::Publisher odo_flag_pub = nh.advertise<geometry_msgs::PointStamped>("/odo_flag", 1000);
    ros::Publisher odo_dist_pub = nh.advertise<geometry_msgs::PointStamped>("/odo_dist", 1000);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    ros::Rate loop_rate(20);
    while(ros::ok()){
        int odo_flag;
        ros::spinOnce();

        if(ser.available()){
            std_msgs::String result;
            geometry_msgs::PointStamped odo_flag_result;
            result.data = ser.read(ser.available());
            odo_flag=write_buffer(result);
            if(odo_flag) odo_flag_pub.publish(odo_flag_result); 
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}
