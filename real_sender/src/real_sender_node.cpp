#include    <ros/ros.h>
#include    <ros/console.h>
#include    <serial.h>
#include    "serializer/serializer.hpp"
#include    <iostream>
#include    <string>
#include  <unistd.h>
#include    <consai_msgs/robot_commands.h>

class Sender
{
public:
    Sender()
        : mBaudrate_(57600),mPort_("/dev/ttyUSB0"){
    }
    ~Sender(){}

    void setID(const int id){
        mID_ = id;
        mSerial_ = new serial::Serial(mPort_,mBaudrate_,serial::Timeout::simpleTimeout(1000));
    }
    void callback(const consai_msgs::robot_commandsConstPtr& msg){

        RootsSerializer serializer;

        float   vel_norm  = hypot(msg->vel_surge, msg->vel_sway),
            vel_theta = atan2(msg->vel_sway, msg->vel_surge) + M_PI/2,
            omega     = msg->omega,
            kick_power= (msg->kick_speed_x > 0.0) ? 15 : 0,
            dribble_power   = (msg->dribble_power > 0.0) ? 15 : 0;
        RobotCommand::KickType  kick_type = (msg->kick_speed_z > 0.0) ? RobotCommand::CHIP : RobotCommand::STRAIGHT;

        RobotCommand cmd(mID_, vel_norm, vel_theta, omega, dribble_power, kick_power, kick_type);

        std::string data;
        serializer.serialize(cmd, &data);

        mSerial_->write(data);
    }
    
private:
    const int mBaudrate_;
    const std::string mPort_;

    int mID_;
    serial::Serial *mSerial_;
};

int main(int argc, char **argv) {
    int robot_num = 12;
    ros::init(argc, argv, "real_sender");
    ros::NodeHandle nh;
    ros::Rate r(60);

    Sender senders[robot_num];
    ros::Subscriber subscribers[robot_num];

    for(int i=0;i<12;i++){
        std::stringstream topicStream;
        topicStream << "/robot_" << i << "/robot_commands";
        std::string topicName = topicStream.str();
        senders[i].setID(i);
        subscribers[i] = nh.subscribe(topicName.c_str(),100,&Sender::callback, &senders[i]);
    }

    while (ros::ok()) {
//         ROS_INFO("Loop");

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
