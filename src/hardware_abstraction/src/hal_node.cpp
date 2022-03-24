#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "usb_can.h"
#include <signal.h>

ros::Subscriber right_hip_cmd_sub,right_leg1_cmd_sub,right_leg2_cmd_sub,right_wheel_cmd_sub; 
ros::Subscriber left_hip_cmd_sub,left_leg1_cmd_sub,left_leg2_cmd_sub,left_wheel_cmd_sub;

ros::Publisher left_wheel_vel_pub,right_wheel_vel_pub;
Usb_can usb_can_left("/dev/ttyACM0",115200,serial::bytesize_t(8),serial::stopbits_one,serial::parity_odd,serial::Timeout::simpleTimeout(1e8));
Usb_can usb_can_right("/dev/ttyACM1",115200,serial::bytesize_t(8),serial::stopbits_one,serial::parity_odd,serial::Timeout::simpleTimeout(1e8));

/*
callback functions 
reveive the ros topics
send command msg data to CAN bus
*/ 
void right_hip_cb(std_msgs::Float64::ConstPtr msg){
    // uint8_t id[2];
    // id[0]=0x00;
    // id[1]=0x00;
    // uint8_t data[3];
    // data[0] = 0xCC;
    // data[1] = 0xEE;
    // data[2] = 0xFF;
    // if(usb_can.isOpen()){
    //     usb_can_right.write(id,3,data);
    //     std::cout<<"send over"<<std::endl;
    // }
    // else{
    //     std::cout<<"open error"<<std::endl;
    // }
}
void right_leg1_cb(std_msgs::Float64::ConstPtr msg){

}
void right_leg2_cb(std_msgs::Float64::ConstPtr msg){

}
void right_wheel_cb(std_msgs::Float64::ConstPtr msg){

}
void left_hip_cb(std_msgs::Float64::ConstPtr msg){

}
void left_leg1_cb(std_msgs::Float64::ConstPtr msg){

}
void left_leg2_cb(std_msgs::Float64::ConstPtr msg){

}
void left_wheel_cb(std_msgs::Float64::ConstPtr msg){

}
/*
catch ctrl+C signal
*/
void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    //close usb_can
    usb_can_left.close();
    usb_can_right.close();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hal_node");
    ros::NodeHandle n;

    left_wheel_vel_pub = n.advertise<std_msgs::Float64>("topic_name",1);
    right_wheel_vel_pub = n.advertise<std_msgs::Float64>("topic_name",1);

    right_hip_cmd_sub = n.subscribe("topic_name",1,right_hip_cb);
    right_leg1_cmd_sub = n.subscribe("topic_name",1,right_leg1_cb);
    right_leg2_cmd_sub = n.subscribe("topic_name",1,right_leg2_cb);
    right_wheel_cmd_sub = n.subscribe("topic_name",1,right_wheel_cb);

    left_hip_cmd_sub = n.subscribe("topic_name",1,left_hip_cb);
    left_leg1_cmd_sub = n.subscribe("topic_name",1,left_leg1_cb);
    left_leg2_cmd_sub = n.subscribe("topic_name",1,left_leg2_cb);
    left_wheel_cmd_sub = n.subscribe("topic_name",1,left_wheel_cb);

    usb_can_left.open();
    usb_can_right.open();
    signal(SIGINT, mySigintHandler);

    while(ros::ok())
    {
        // TODO read can data and send the data to topics
        size_t frame_num = usb_can_left.available();
        if(frame_num > 0){
            std::cout << "--new frame num: " << frame_num << std::endl;
            // each frame
            for(int i=0;i<frame_num;i++){
                std::cout << "  No. " << i << std::endl;
                can_frame* frame_ptr = usb_can_left.read();
                std::cout<<frame_ptr->getStr()<<std::endl;
                delete frame_ptr;
            }
        }
        // left_wheel_vel_pub.publish();
        //read can data and send the data to topics
        frame_num = usb_can_right.available();
        if(frame_num > 0){
            std::cout << "--new frame num: " << frame_num << std::endl;
            // each frame
            for(int i=0;i<frame_num;i++){
                std::cout << "  No. " << i << std::endl;
                can_frame* frame_ptr = usb_can_right.read();
                std::cout<<frame_ptr->getStr()<<std::endl;
                delete frame_ptr;
            }
        }
        // right_wheel_vel_pub.publish();
        ros::spinOnce();
    }
}
