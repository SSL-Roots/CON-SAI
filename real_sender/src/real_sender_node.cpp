#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <string>
#include <cmath>
#include <unistd.h>

#include <consai_msgs/robot_commands.h>
#include <frootspi_msg/FrootsCommand.h>

#include "serial.h"
#include "serializer.hpp"

class Sender
{
public:
    Sender(bool noSerial)
        : mBaudrate_(57600),mPort_("/dev/ttyUSB0"),mSerialOpend(false),
        mIDLinked(false){

        mNoSerial = noSerial;
        if(noSerial == false){
            try{
                mSerial_ = new serial::Serial(mPort_,mBaudrate_,serial::Timeout::simpleTimeout(1000));
                mSerialOpend = true;
            } catch (std::exception &error){
                ROS_ERROR(" %s", error.what());
            }
        }
    }

    ~Sender(){}

    bool portOpend(){
        return mSerialOpend;
    }

    void setID(ros::NodeHandle nh, const int id){
        mID_ = id;
        mIDLinked = true;
        mPublishTopicName = "robot_" + std::to_string(id) + "/froots_command";
        mPubFrootsCommand = nh.advertise<frootspi_msg::FrootsCommand>(
                mPublishTopicName.c_str(), 1);
    }

    void callback(const consai_msgs::robot_commandsConstPtr& msg){

        if(mNoSerial == false){
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

        // for FrootsPi
        if(mIDLinked){
            publishFrootsCommand(msg);
        }
    }

    void publishFrootsCommand(const consai_msgs::robot_commandsConstPtr& msg){
        frootspi_msg::FrootsCommand frootsCommand;

        // 強制的にキック用のコンデンサを充電する
        frootsCommand.charge_flag = true;

        if(msg->kick_speed_x){
            frootsCommand.kick_flag = true;
            frootsCommand.kick_power = msg->kick_speed_x;
        }

        if(msg->dribble_power){
            frootsCommand.dribble_power = msg->dribble_power;
        }

        double norm = std::sqrt(
                std::pow(msg->vel_surge, 2) + std::pow(msg->vel_sway, 2));
        double theta = std::atan2(msg->vel_sway, msg->vel_surge);

        frootsCommand.vel_norm = norm;
        frootsCommand.vel_theta = theta;
        frootsCommand.vel_omega = msg->omega;

        mPubFrootsCommand.publish(frootsCommand);
    }

    
private:
    const int mBaudrate_;
    const std::string mPort_;

    bool mNoSerial;
    int mID_;
    serial::Serial *mSerial_;
    bool mSerialOpend;
    bool mIDLinked;
    ros::Publisher mPubFrootsCommand;
    std::string mPublishTopicName;
};


int main(int argc, char **argv) {
    int robot_num = 12;
    ros::init(argc, argv, "real_sender");
    ros::NodeHandle nh;
    ros::Rate r(60);

    bool noSerial = false;
    ros::param::get("~no_serial", noSerial);

    if(noSerial){
        ROS_INFO("NO SERIAL");
    }else{
        ROS_INFO("SERIALLLLLL");
    }

    // Sender senders[robot_num];
    Sender* senders[robot_num];
    ros::Subscriber subscribers[robot_num];

    for(int i=0;i<12;i++){
        senders[i] = new Sender(noSerial);

        // シリアル通信を使う場合は、ポートのチェックをする　
        if (noSerial == false && !senders[i]->portOpend()){
            ros::shutdown();
        }

        senders[i]->setID(nh, i);

        std::string parentName;
        parentName = "robot_" + std::to_string(i);
        std::string topicName = parentName + "/robot_commands";

        subscribers[i] = nh.subscribe(topicName.c_str(),100,&Sender::callback, senders[i]);
    }

    while (ros::ok()) {
//         ROS_INFO("Loop");

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
