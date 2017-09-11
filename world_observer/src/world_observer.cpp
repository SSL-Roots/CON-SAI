#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <consai_msgs/VisionObservations.h>
#include <consai_msgs/VisionRobotPackets.h>
#include <consai_msgs/VisionPacket.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <sstream>
#include <cmath>

#include <world_observer/estimator.hpp>
#include <world_observer/enemy_estimator.hpp>
#include <world_observer/ball_estimator.hpp>



class Observer
{
    public:
        Observer(ros::NodeHandle& nh, std::string poses_source) :
            sub_vision(nh.subscribe(poses_source, 1000, &Observer::visionCallBack, this)),
            observation_refreshed(false){}

        void update() {
            nav_msgs::Odometry odom;

            if (observation_refreshed) {
                odom = estimator->estimate(accel_, last_observation.poses);
                observation_refreshed = false;
            } else {
                odom = estimator->estimate();
                ROS_DEBUG("No Observation received!");
            }

            publish(odom);
        }


    protected:
        Estimator* estimator;
        ros::Subscriber sub_vision;
        geometry_msgs::PoseArray last_observation;
        geometry_msgs::Accel accel_;
        bool observation_refreshed;

        virtual void publish(nav_msgs::Odometry odom) = 0;
        virtual void visionCallBackProcess(consai_msgs::VisionObservations msg) = 0;

        void visionCallBack(const consai_msgs::VisionObservations::ConstPtr& msg)
        {
            visionCallBackProcess(*msg);
            //    last_observation  = *msg;
            observation_refreshed = true;
        }


        // フィールドの右半分/左半分だけを使って練習するときに使用する。
        bool doSkip(const geometry_msgs::Pose& pose){
            bool skipFlag = false;
            
            // double posX = pose.position.x;
            // double posY = pose.position.y;
            //
            // Skip left side positions
            // if (posX < 0){
            //     skipFlag = true;
            // }
            //
            // Skip lower side positions
            // if (posY < 0){
            //     skipFlag = true;
            // }
            
            return skipFlag;
        }

};




class FriendObserver :public Observer
{
    public:
        FriendObserver(ros::NodeHandle& nh, std::string poses_source, int robot_id) :
            Observer(nh, poses_source),
            pub_odom(nh_odom.advertise<nav_msgs::Odometry>("odom", 1000)),
            pub_vel_norm(nh_odom.advertise<std_msgs::Float64>("vel_norm", 1000)),
            sub_accel(nh_odom.subscribe<geometry_msgs::Accel>("accel_world", 1000, &FriendObserver::accelCallBack, this)),
            _robot_id(robot_id)
    {
        estimator = new EnemyEstimator(0.016);
    }


    protected:
        int _robot_id;
        ros::NodeHandle nh_odom;
        ros::Publisher pub_odom;
        ros::Publisher pub_vel_norm;
        tf::TransformBroadcaster odom_broadcaster;
        ros::Subscriber sub_accel;

        void publish(nav_msgs::Odometry odom)
        {
            ros::Time current_time = ros::Time::now();

            odom.header.stamp = current_time;

            /* make transform */
            geometry_msgs::TransformStamped odom_trans;
            std::stringstream odom_name, base_link_name;

            odom_name   << "friend_" << _robot_id << "/odom";
            base_link_name << "friend_" << _robot_id << "/base_link";

            odom_trans.header.frame_id = odom_name.str();
            odom_trans.child_frame_id = base_link_name.str();
            odom_trans.header.stamp = current_time;

            odom_trans.transform.translation.x = odom.pose.pose.position.x;
            odom_trans.transform.translation.y = odom.pose.pose.position.y;
            odom_trans.transform.rotation = odom.pose.pose.orientation;

            odom_broadcaster.sendTransform(odom_trans);
            pub_odom.publish(odom);


            double velX = odom.twist.twist.linear.x;
            double velY = odom.twist.twist.linear.y;

            std_msgs::Float64 vel_norm;
            vel_norm.data = std::sqrt(velX*velX + velY*velY);
            pub_vel_norm.publish(vel_norm);
        }

        void visionCallBackProcess(consai_msgs::VisionObservations msg)
        {
            geometry_msgs::PoseArray pose_array;

            vector<consai_msgs::VisionRobotPackets>::iterator it;

            it = msg.friends.begin();

            for (it = msg.friends.begin() ; it != msg.friends.end(); ++it) {
                // search desired robot id
                if (it->robot_id == _robot_id) {
                    for (vector<consai_msgs::VisionPacket>::iterator it_pack = it->packets.begin(); it_pack != it->packets.end(); ++it_pack) {
                        if (doSkip(it_pack->pose) ){
                            continue;
                        }
                        pose_array.header=msg.header;
                        pose_array.poses.push_back(it_pack->pose);
                    }
                }
            }

            last_observation  = pose_array;
        }

        void accelCallBack(const geometry_msgs::AccelConstPtr& msg)
        {
            accel_ = *msg;
        }
};



class EnemyObserver :public Observer
{
    public:
        EnemyObserver(ros::NodeHandle& nh, std::string poses_source, int robot_id) :
            Observer(nh, poses_source),
            pub_odom(nh_odom.advertise<nav_msgs::Odometry>("odom", 1000)),
            pub_vel_norm(nh_odom.advertise<std_msgs::Float64>("vel_norm", 1000)),
            _robot_id(robot_id)
    {
        estimator = new EnemyEstimator(0.016);
    }

    protected:
        int _robot_id;
        ros::NodeHandle nh_odom;
        ros::Publisher pub_odom;
        ros::Publisher pub_vel_norm;
        tf::TransformBroadcaster odom_broadcaster;

        void publish(nav_msgs::Odometry odom)
        {
            ros::Time current_time = ros::Time::now();

            odom.header.stamp = current_time;

            /* make transform */
            geometry_msgs::TransformStamped odom_trans;
            std::stringstream odom_name, base_link_name;

            odom_name   << "enemy_" << _robot_id << "/odom";
            base_link_name << "enemy_" << _robot_id << "/base_link";

            odom_trans.header.frame_id = odom_name.str();
            odom_trans.child_frame_id = base_link_name.str();
            odom_trans.header.stamp = current_time;

            odom_trans.transform.translation.x = odom.pose.pose.position.x;
            odom_trans.transform.translation.y = odom.pose.pose.position.y;
            odom_trans.transform.rotation = odom.pose.pose.orientation;

            odom_broadcaster.sendTransform(odom_trans);
            pub_odom.publish(odom);


            double velX = odom.twist.twist.linear.x;
            double velY = odom.twist.twist.linear.y;

            std_msgs::Float64 vel_norm;
            vel_norm.data = std::sqrt(velX*velX + velY*velY);
            pub_vel_norm.publish(vel_norm);
        }

        void visionCallBackProcess(consai_msgs::VisionObservations msg)
        {
            geometry_msgs::PoseArray pose_array;

            vector<consai_msgs::VisionRobotPackets>::iterator it;

            it = msg.enemies.begin();

            for (it = msg.enemies.begin() ; it != msg.enemies.end(); ++it) {
                // search desired robot id
                if (it->robot_id == _robot_id) {
                    for (vector<consai_msgs::VisionPacket>::iterator it_pack = it->packets.begin(); it_pack != it->packets.end(); ++it_pack) {
                        if (doSkip(it_pack->pose) ){
                            continue;
                        }
                        pose_array.header=msg.header;
                        pose_array.poses.push_back(it_pack->pose);
                    }
                }
            }

            last_observation  = pose_array;
        }
};



class BallObserver :public Observer
{
    public:
        BallObserver(ros::NodeHandle& nh, std::string poses_source) :
            Observer(nh, poses_source),
            pub_odom(nh.advertise<nav_msgs::Odometry>("estimation", 1000)),
            pub_vel_norm(nh.advertise<std_msgs::Float64>("vel_norm", 1000))
    {
        estimator = new BallEstimator(0.016);
    }


    protected:
        ros::Publisher pub_odom;
        ros::Publisher pub_vel_norm;
        tf::TransformBroadcaster odom_broadcaster;

        void publish(nav_msgs::Odometry odom)
        {
            ros::Time current_time = ros::Time::now();

            odom.header.stamp = current_time;

            /* make transform */
            geometry_msgs::TransformStamped odom_trans;

            odom_trans.header.frame_id = "map";
            odom_trans.child_frame_id = "ball";
            odom_trans.header.stamp = current_time;

            odom_trans.transform.translation.x = odom.pose.pose.position.x;
            odom_trans.transform.translation.y = odom.pose.pose.position.y;

            odom_trans.transform.rotation.x = 0.0;
            odom_trans.transform.rotation.y = 0.0;
            odom_trans.transform.rotation.z = 0.0;
            odom_trans.transform.rotation.w = 1.0;

            odom_broadcaster.sendTransform(odom_trans);
            pub_odom.publish(odom);


            double velX = odom.twist.twist.linear.x;
            double velY = odom.twist.twist.linear.y;

            std_msgs::Float64 vel_norm;
            vel_norm.data = std::sqrt(velX*velX + velY*velY);
            pub_vel_norm.publish(vel_norm);
        }

        void visionCallBackProcess(consai_msgs::VisionObservations msg)
        {
            geometry_msgs::PoseArray  pose_array;

            for (vector<consai_msgs::VisionPacket>::iterator it_pack = msg.ball.begin(); it_pack != msg.ball.end(); ++it_pack) {
                if (doSkip(it_pack->pose) ){
                    continue;
                }
                pose_array.header=msg.header;
                pose_array.poses.push_back(it_pack->pose);
            }

            last_observation = pose_array;
        }
};


int main(int argc, char **argv)
{
    const std::string node_name = "observer";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    std::string poses_source;
    nh.param<std::string>("poses_source", poses_source, "/vision_observations");

    std::string observe_target;
    if (!nh.getParam("observe_target", observe_target)) {
        ROS_ERROR("observe target is not set");
        return 0;
    }

    int observe_target_id;
    if (observe_target != "Ball") {
        if (!nh.getParam("observe_target_id", observe_target_id)) {
            ROS_ERROR("observe target id is not set");
            return 0;
        }
    }

    Observer*  obs;
    if (observe_target == "Ball") {
        obs = new BallObserver(nh, poses_source);
    } else if (observe_target == "Friend") {
        obs = new FriendObserver(nh, poses_source, observe_target_id);
    } else {
        obs = new EnemyObserver(nh, poses_source, observe_target_id);
    }

    ros::Rate loop_rate(60);


    while (ros::ok()) {
        obs->update();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
