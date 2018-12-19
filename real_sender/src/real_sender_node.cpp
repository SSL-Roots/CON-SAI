#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include <consai_msgs/robot_commands.h>

#include "serial.h"
#include "serializer.hpp"

class Sender
{
public:
    Sender()
        : mBaudrate_(57600),mPort_("/dev/ttyUSB0"),mSerialOpend(false){

        try{
            mSerial_ = new serial::Serial(mPort_,mBaudrate_,serial::Timeout::simpleTimeout(1000));
            mSerialOpend = true;
        } catch (std::exception &error){
            ROS_ERROR(" %s", error.what());
        }
    }

    ~Sender(){}

    bool portOpend(){
        return mSerialOpend;
    }

    void setID(const int id){
        mID_ = id;
    }

    void callback(const consai_msgs::robot_commandsConstPtr& msg){

        RootsSerializer serializer;

        float   vel_norm  = hypot(msg->vel_surge, msg->vel_sway),
            vel_theta = atan2(msg->vel_sway, msg->vel_surge) + M_PI/2,
            omega     = msg->omega,
            kick_power= msg->kick_speed_x,
            dribble_power   = msg->dribble_power;
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
    bool mSerialOpend;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "real_sender");
    ros::NodeHandle nh;
    ros::Rate r(60);

    int ID_MAX = 12;
    ros::param::get("id_max", ID_MAX);

    Sender senders[ID_MAX];
    ros::Subscriber subscribers[ID_MAX];

    for(int i=0;i<ID_MAX;i++){
        if (!senders[i].portOpend()){
            ros::shutdown();
        }

        std::stringstream topicStream;
        topicStream << "robot_" << i << "/robot_commands";
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
