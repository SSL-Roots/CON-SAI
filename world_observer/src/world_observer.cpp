#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <consai_msgs/VisionPacket.h>
#include <consai_msgs/VisionData.h>
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
            mPosesSource(poses_source),
            sub_vision(nh.subscribe(poses_source, 10, &Observer::visionCallBack, this)),
            observation_refreshed(false){}

        void update() {
            nav_msgs::Odometry odom;

            if (observation_refreshed) {
                odom = estimator->estimate(accel_, last_pose_array.poses);
                observation_refreshed = false;
            } else {
                odom = estimator->estimate();
                ROS_DEBUG("No Observation received! %s", mPosesSource.c_str());
            }

            publish(odom);
        }


    protected:
        Estimator* estimator;
        ros::Subscriber sub_vision;
        geometry_msgs::PoseArray last_pose_array;
        geometry_msgs::Accel accel_;
        bool observation_refreshed;
        std::string mPosesSource;

        virtual void publish(nav_msgs::Odometry odom) = 0;

        void visionCallBack(const consai_msgs::VisionPacket::ConstPtr& msg)
        {
            visionCallBackProcess(*msg);
            observation_refreshed = true;
        }

        void visionCallBackProcess(const consai_msgs::VisionPacket msg)
        {
            geometry_msgs::PoseArray pose_array;

            pose_array.header = msg.header;

            for(consai_msgs::VisionData data : msg.data){
                if(doSkip(data.pose)){
                    continue;
                }
                pose_array.poses.push_back(data.pose);
            }
            last_pose_array = pose_array;
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
            pub_odom(nh_odom.advertise<nav_msgs::Odometry>("odom", 10)),
            pub_vel_norm(nh_odom.advertise<std_msgs::Float64>("vel_norm", 10)),
            sub_accel(nh_odom.subscribe<geometry_msgs::Accel>("accel_world", 10, &FriendObserver::accelCallBack, this)),
            _robot_id(robot_id)
    {
        estimator = new EnemyEstimator(0.0166);
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
            pub_odom(nh_odom.advertise<nav_msgs::Odometry>("odom", 10)),
            pub_vel_norm(nh_odom.advertise<std_msgs::Float64>("vel_norm", 10)),
            _robot_id(robot_id)
    {
        estimator = new EnemyEstimator(0.0166);
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
};



class BallObserver :public Observer
{
    public:
        BallObserver(ros::NodeHandle& nh, std::string poses_source) :
            Observer(nh, poses_source),
            pub_odom(nh.advertise<nav_msgs::Odometry>("estimation", 10)),
            pub_vel_norm(nh.advertise<std_msgs::Float64>("vel_norm", 10))
    {
        estimator = new BallEstimator(0.0166);
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
};


int main(int argc, char **argv)
{
    const std::string node_name = "observer";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // 起動時のnamespaceを取得
    std::string ai_name = "/";
    ros::param::get("ai_name", ai_name);

    std::string observe_target;
    if (!ros::param::get("~observe_target", observe_target)) {
        ROS_ERROR("observe target is not set");
        return 0;
    }

    int observe_target_id;
    if (observe_target != "Ball") {
        if (!ros::param::get("~observe_target_id", observe_target_id)) {
            ROS_ERROR("observe target id is not set");
            return 0;
        }
    }

    Observer*  obs;
    if (observe_target == "Ball") {
        ros::NodeHandle nh_("~");
        obs = new BallObserver(nh_, ai_name + "ball_vision_packet");
    } else if (observe_target == "Friend") {
        obs = new FriendObserver(nh, "vision_packet", observe_target_id);
    } else {
        obs = new EnemyObserver(nh, "vision_packet", observe_target_id);
    }

    ros::Rate loop_rate(60);


    while (ros::ok()) {
        obs->update();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
