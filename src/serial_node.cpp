*******
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

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
//    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}
void write_buffer(const std_msgs::String msg){
    static unsigned char r_pkt[20];
    static int r_pkt_idx=0;
    int i;
    unsigned long len=msg.data.size();   
    unsigned char battery_level;
    if (msg.data[len-1]==(char)'E'){
       // ROS_INFO("%lu",msg.data.size()); 
        
        for (i=0;i<len;i++){
            r_pkt[r_pkt_idx]=msg.data[i];
            r_pkt_idx++;
        }
        long travel_distance_right_wheel,travel_distance_left_wheel;
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
            ROS_INFO("dist_r : %8d, dist_l : %8d, Battery : %3d", travel_distance_right_wheel,travel_distance_left_wheel,battery_level); 
        }
    }
} 
int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("/mobile/write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/mobile/read", 1000);

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

        ros::spinOnce();

        if(ser.available()){
//            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
//            ROS_INFO_STREAM("Read: " << result.data);
            //ROS_INFO("%lu",result.data.size());
              write_buffer(result);
//            ROS_INFO("%lu",result.data[1,2,3,4]);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}
