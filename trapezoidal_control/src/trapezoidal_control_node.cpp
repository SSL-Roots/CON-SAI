#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <trapezoidal_control/parameterConfig.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include "trapezoidal_controller.h"

class Controller{
    public:
        Controller();
        void update();
        
        geometry_msgs::Twist getCommandVelocity();
        geometry_msgs::Point getAvoidingPoint();
        geometry_msgs::Twist getResiduals();

        void callbackTargetPose(const geometry_msgs::PoseStampedConstPtr& msg);
        void callbackTargetVel(const geometry_msgs::TwistStampedConstPtr& msg);
        void callbackRealPose(const nav_msgs::OdometryConstPtr& msg);
        void callbackReconfigure(trapezoidal_control::parameterConfig &config, uint32_t level);
        void callbackAvoidingPoint(const geometry_msgs::Point& msg);

    private:
        geometry_msgs::Twist mCommandVel;
        geometry_msgs::Twist mResiduals;
        geometry_msgs::Pose mTargetPose;
        geometry_msgs::Twist mTargetVel;
        geometry_msgs::Pose mRealPose;
        geometry_msgs::Twist mRealVel;
        geometry_msgs::Point mAvoidingPoint;

        bool mIsVelocityControl;
        TrapezoidalController *mControllerX, *mControllerY, *mControllerYaw;

        void poseControl();
        void velocityControl();

        double yawFromQuaternion(geometry_msgs::Quaternion geoQ);
};

Controller::Controller(){
    mIsVelocityControl = false;

    mControllerX = new TrapezoidalController(0.02, 0.04, 2.0, 0.020, false);
    mControllerY = new TrapezoidalController(0.02, 0.04, 2.0, 0.020, false);
    mControllerYaw = new TrapezoidalController(0.02, 0.04, 2.0, 0.020, true);
}

void Controller::update(){
    if(mIsVelocityControl){
        velocityControl();
    }else{
        poseControl();
    }
}

geometry_msgs::Twist Controller::getCommandVelocity(){
    return mCommandVel;
}

geometry_msgs::Point Controller::getAvoidingPoint(){
    return mAvoidingPoint;
}

geometry_msgs::Twist Controller::getResiduals(){
    return mResiduals;
}

void Controller::callbackTargetPose(const geometry_msgs::PoseStampedConstPtr& msg){
    mTargetPose = msg->pose;
    mIsVelocityControl = false;
}

void Controller::callbackTargetVel(const geometry_msgs::TwistStampedConstPtr& msg){
    mTargetVel = msg->twist;
    mIsVelocityControl = true;
}

void Controller::callbackRealPose(const nav_msgs::OdometryConstPtr& msg){
    mRealPose = msg->pose.pose;
    mRealVel = msg->twist.twist;
}

void Controller::callbackReconfigure(trapezoidal_control::parameterConfig &config, uint32_t level){
    mControllerX->setParameters(config.accSpeed, config.decSpeed, 
            config.maxSpeed, config.hysteresis);
    mControllerY->setParameters(config.accSpeed, config.decSpeed, 
            config.maxSpeed, config.hysteresis);
    mControllerYaw->setParameters(config.accRotation, config.decRotation, 
            config.maxRotation, config.hysRotation);
}


void Controller::callbackAvoidingPoint(const geometry_msgs::Point& msg){
    mAvoidingPoint = msg;
}

void Controller::poseControl(){
    double realYaw = yawFromQuaternion(mRealPose.orientation);
    double targetYaw = yawFromQuaternion(mTargetPose.orientation);

    mControllerX->update(mRealPose.position.x, mRealVel.linear.x, mAvoidingPoint.x);
    mControllerY->update(mRealPose.position.y, mRealVel.linear.y, mAvoidingPoint.y);
    mControllerYaw->update(realYaw, mRealVel.angular.z, targetYaw);

    mResiduals.linear.x = mControllerX->getResidual();
    mResiduals.linear.y = mControllerY->getResidual();
    mResiduals.angular.z = mControllerYaw->getResidual();

    geometry_msgs::Vector3 linearVel;
    linearVel.x = mControllerX->getVelocity();
    linearVel.y = mControllerY->getVelocity();

    // 移動方向をロボット座標系に変換
    mCommandVel.linear.x = cos(-realYaw)*linearVel.x - sin(-realYaw)*linearVel.y;
    mCommandVel.linear.y = sin(-realYaw)*linearVel.x + cos(-realYaw)*linearVel.y;

    mCommandVel.angular.z = mControllerYaw->getVelocity();
}

void Controller::velocityControl(){
    mControllerX->update(mTargetVel.linear.x);
    mControllerY->update(mTargetVel.linear.y);
    mControllerYaw->update(mTargetVel.angular.z);

    mCommandVel.linear.x= mControllerX->getVelocity();
    mCommandVel.linear.y = mControllerY->getVelocity();
    mCommandVel.angular.z = mControllerYaw->getVelocity();
}

double Controller::yawFromQuaternion(geometry_msgs::Quaternion geoQ){
    tf::Quaternion q(geoQ.x, geoQ.y, geoQ.z, geoQ.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(std::isnan(yaw)){
        yaw = 0.0;
    }
    return yaw;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trapezoidal_control");
    ros::NodeHandle nh;
    ros::Rate r(60);

    Controller controller;
    ros::Subscriber subTargetPose = nh.subscribe("move_base_simple/goal",10,
            &Controller::callbackTargetPose, &controller);
    ros::Subscriber subTargetVel = nh.subscribe("move_base_simple/target_velocity",
            10, &Controller::callbackTargetVel, &controller);
    ros::Subscriber subRealPose = nh.subscribe("odom", 10,
            &Controller::callbackRealPose, &controller);
    ros::Subscriber subAvoidingPoint = nh.subscribe("avoiding_point", 10,
            &Controller::callbackAvoidingPoint, &controller);

    dynamic_reconfigure::Server<trapezoidal_control::parameterConfig> reconfigure_server;
    dynamic_reconfigure::Server<trapezoidal_control::parameterConfig>::CallbackType f;

    f = boost::bind(&Controller::callbackReconfigure, &controller, _1, _2);
    reconfigure_server.setCallback(f);

    ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher res_publisher = nh.advertise<geometry_msgs::Twist>("residuals", 10);

    while (ros::ok()){
        controller.update();
        geometry_msgs::Twist cmdVel = controller.getCommandVelocity();
        geometry_msgs::Twist residuals = controller.getResiduals();

        publisher.publish(cmdVel);
        res_publisher.publish(residuals);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
