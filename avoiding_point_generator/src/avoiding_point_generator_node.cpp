#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <avoiding_point_generator/parameterConfig.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm> //std::copy
#include <iterator> //std::back_inserter

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16MultiArray.h>
#include <consai_msgs/AIStatus.h>

#include "transformation.h"


class Controller{
    public:
        Controller();
        void update();
        
        geometry_msgs::Point getAvoidPoint();

        void callbackTargetPose(const geometry_msgs::PoseStampedConstPtr& msg);
        void callbackRealPose(const nav_msgs::OdometryConstPtr& msg);
        void callbackReconfigure(avoiding_point_generator::parameterConfig &config, uint32_t level);
        void callbackEnemyPose(const ros::MessageEvent<nav_msgs::Odometry const>& event);
        void callbackFriendPose(const ros::MessageEvent<nav_msgs::Odometry const>& event);
        void callbackEnemyIDs(const std_msgs::UInt16MultiArrayConstPtr& msg);
        void callbackFriendIDs(const std_msgs::UInt16MultiArrayConstPtr& msg);
        void callbackAIStatus(const consai_msgs::AIStatus& msg);
        void callbackBallPoint(const nav_msgs::OdometryConstPtr& msg);

    private:
        const double mPeriod_;
        geometry_msgs::Pose mTargetPose;
        geometry_msgs::Pose mRealPose;
        geometry_msgs::Twist mRealVel;
        geometry_msgs::Pose mEnemyPoses[12];
        geometry_msgs::Pose mFriendPoses[12];
        std::vector<int> mEnemyIDs;
        std::vector<int> mFriendIDs;
        geometry_msgs::Point mAvoidPoint;
        consai_msgs::AIStatus mAIStatus;
        geometry_msgs::Point mBallPoint;

        double mDetectRange;
        double mAvoidRange;
        double mStartDetectionPos;
        double mAvoidHysteresis;
        bool mAvoidPointIsUpperImag;

        struct OAParameter{
            double trAvoidReal;
            double trAvoidUpperImag;
            double trAvoidLowerImag;
            double trGoalPosReal;
            OAParameter(){
                trAvoidReal = 0;
                trAvoidUpperImag = 0;
                trAvoidLowerImag = 0;
                trGoalPosReal = 0;
            }
        };


        geometry_msgs::Point calcuAvoidingPoint(const geometry_msgs::Point &targetPos);
        OAParameter updateObstacleDetection(const Transformation &trans, 
                double posX, double posY, const OAParameter &parameter);
        OAParameter updateAvoidingPoints(const Transformation &trans,
                double posX, double posY, const OAParameter &parameter);
};

Controller::Controller()
    :mPeriod_(0.016){

    mDetectRange = 0.5;
    mAvoidRange = 0.5;
    mStartDetectionPos = 0.0;
    mAvoidHysteresis = 0.18;
    mAvoidPointIsUpperImag = true;
}

void Controller::update(){
}


geometry_msgs::Point Controller::getAvoidPoint(){
    return mAvoidPoint;
}

void Controller::callbackTargetPose(const geometry_msgs::PoseStampedConstPtr& msg){
    mTargetPose = msg->pose;
}

void Controller::callbackRealPose(const nav_msgs::OdometryConstPtr& msg){
    mRealPose = msg->pose.pose;
    mRealVel = msg->twist.twist;
}

void Controller::callbackReconfigure(avoiding_point_generator::parameterConfig &config, uint32_t level){
    mDetectRange = config.detectRange;
    mAvoidRange = config.avoidRange;
    mStartDetectionPos = config.startDetectionPos;
    mAvoidHysteresis = config.avoidHysteresis;
}

void Controller::callbackEnemyPose(const ros::MessageEvent<nav_msgs::Odometry const>& event){
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");

    // TODO: ここをもっと綺麗にしたい
    // idを抽出
    int size = topic.size();

    std::string idStr;
    if(size == 13){
        idStr = topic.substr(7,1);
    }else{
        idStr = topic.substr(7,2);
    }

    int id = std::atoi(idStr.c_str());
    const nav_msgs::OdometryConstPtr& msg = event.getMessage();
    mEnemyPoses[id] = msg->pose.pose;
}

void Controller::callbackFriendPose(const ros::MessageEvent<nav_msgs::Odometry const>& event){
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");

    // TODO: ここももっと綺麗にしたい
    // idを抽出
    int size = topic.size();

    std::string idStr;
    if(size == 13){
        idStr = topic.substr(7,1);
    }else{
        idStr = topic.substr(7,2);
    }

    int id = std::atoi(idStr.c_str());
    const nav_msgs::OdometryConstPtr& msg = event.getMessage();
    mFriendPoses[id] = msg->pose.pose;
}


void Controller::callbackEnemyIDs(const std_msgs::UInt16MultiArrayConstPtr& msg){
    mEnemyIDs.clear();
    std::copy(msg->data.begin(), msg->data.end(), std::back_inserter(mEnemyIDs));
}

void Controller::callbackFriendIDs(const std_msgs::UInt16MultiArrayConstPtr& msg){
    mFriendIDs.clear();
    std::copy(msg->data.begin(), msg->data.end(), std::back_inserter(mFriendIDs));
}

void Controller::callbackAIStatus(const consai_msgs::AIStatus& msg){
    mAIStatus = msg;
}

void Controller::callbackBallPoint(const nav_msgs::OdometryConstPtr& msg){
    mBallPoint = msg->pose.pose.position;
}


geometry_msgs::Point Controller::calcuAvoidingPoint(const geometry_msgs::Point &targetPos){
    // 障害物の横に回避ポイントを作成する
    // 障害物が横並びの時は、大幅に避けるポイントを作成する
    

    // 自分の位置から目標位置までの座標系を作成
    Complex startPos(mRealPose.position.x,mRealPose.position.y);
    Complex goalPos(targetPos.x, targetPos.y);
    double angleToGoal = Tool::getAngle(goalPos - startPos);

    Transformation trans(startPos, angleToGoal);
    Complex trGoalPos = trans.transform(goalPos);



    // 自分に最も近い障害物と、その左右回避位置を生成
    OAParameter parameter;
    parameter.trAvoidReal = trGoalPos.real();
    parameter.trGoalPosReal = trGoalPos.real();

    // double trAvoidReal = trGoalPos.real();
    // double trAvoidUpperImag=0, trAvoidLowerImag=0;
    geometry_msgs::Pose obstacle;
    std::vector<int>::iterator it;
    int id;

    // 敵ロボット
    for(it=mEnemyIDs.begin(); it!=mEnemyIDs.end(); ++it){
        id = *it;
        obstacle = mEnemyPoses[id];

        parameter = updateObstacleDetection(trans,
                obstacle.position.x, obstacle.position.y, parameter);
    }
    // 見方ロボット
    for(it=mFriendIDs.begin(); it!=mFriendIDs.end(); ++it){
        id = *it;
        obstacle = mFriendPoses[id];

        parameter = updateObstacleDetection(trans,
                obstacle.position.x, obstacle.position.y, parameter);
    }
    // ボール回避は任意
    if(mAIStatus.avoidBall){
        parameter = updateObstacleDetection(trans,
            mBallPoint.x, mBallPoint.y, parameter);
    }



    // 横並びロボット回避位置を生成
    // 障害物がなければ計算省略
    if(parameter.trAvoidUpperImag == 0 && parameter.trAvoidLowerImag == 0){
        // 横並び計算漏れを防ぐため計算をループさせる(ループ回数はテキトー)
        for(int loop=0; loop < 7; loop++){

            // 敵ロボット
            for(it=mEnemyIDs.begin(); it!=mEnemyIDs.end(); ++it){
                id = *it;
                obstacle = mEnemyPoses[id];

                parameter = updateAvoidingPoints(trans,
                        obstacle.position.x, obstacle.position.y, parameter);
            }

            // 見方ロボット
            for(it=mFriendIDs.begin(); it!=mFriendIDs.end(); ++it){
                id = *it;
                obstacle = mFriendPoses[id];
                parameter = updateAvoidingPoints(trans,
                        obstacle.position.x, obstacle.position.y, parameter);
            }

            // ボール回避は任意
            if(mAIStatus.avoidBall){
                parameter = updateAvoidingPoints(trans,
                    mBallPoint.x, mBallPoint.y, parameter);
            }
        }
    }

    // 変換座標系で、左右どちらから避けるかを決定する
    Complex trAvoidPos = trGoalPos;
    double upperSize = fabs(parameter.trAvoidUpperImag);
    double lowerSize = fabs(parameter.trAvoidLowerImag);

    // 左右の回避位置生成にヒステリシスをもたせる
    if(mAvoidPointIsUpperImag == true){
        if(lowerSize < upperSize - mAvoidHysteresis){
            mAvoidPointIsUpperImag = false;
        }
    }else{
        if(upperSize < lowerSize - mAvoidHysteresis){
            mAvoidPointIsUpperImag = true;
        }
    }
    if(mAvoidPointIsUpperImag == true){
        trAvoidPos = Complex(parameter.trAvoidReal, parameter.trAvoidUpperImag);
    }else{
        trAvoidPos = Complex(parameter.trAvoidReal, parameter.trAvoidLowerImag);
    }
    
    // 座標系を元に戻して終了
    Complex avoidPos = trans.invertedTransform(trAvoidPos);

    geometry_msgs::Point output;
    output.x = avoidPos.real();
    output.y = avoidPos.imag();
    output.z = 0;

    return output;
}
        
Controller::OAParameter Controller::updateObstacleDetection(const Transformation &trans,
        double posX, double posY, const OAParameter &parameter){
    OAParameter output = parameter;

    Complex obstaclePos(posX, posY);

    Complex trObstPos = trans.transform(obstaclePos);

    // 自分と目標位置の直線上にいるかチェック
    if(trObstPos.real() > mStartDetectionPos 
            && trObstPos.real() < parameter.trGoalPosReal
            && fabs(trObstPos.imag()) < mDetectRange){

        // 避けれる上下位置を計算
        double upperImag = trObstPos.imag() + mAvoidRange;
        double lowerImag = trObstPos.imag() - mAvoidRange;

        // より近いロボットの距離と回避位置を記憶
        if(trObstPos.real() < parameter.trAvoidReal){
            output.trAvoidReal = trObstPos.real();
            output.trAvoidUpperImag = upperImag;
            output.trAvoidLowerImag = lowerImag;
        }
    }

    return output;
}

Controller::OAParameter Controller::updateAvoidingPoints(const Transformation &trans,
        double posX, double posY, const OAParameter &parameter){

    OAParameter output = parameter;

    Complex obstaclePos(posX, posY);

    Complex trObstPos = trans.transform(obstaclePos);

    // 自分と目標位置の直線上にいるかチェック
    if(trObstPos.real() > mStartDetectionPos 
            && trObstPos.real() < parameter.trGoalPosReal){
        double upperImag = trObstPos.imag() + mAvoidRange;
        double lowerImag = trObstPos.imag() - mAvoidRange;

        if(upperImag > parameter.trAvoidLowerImag 
                && upperImag < parameter.trAvoidUpperImag 
                && lowerImag < parameter.trAvoidLowerImag){

            output.trAvoidLowerImag = lowerImag;
        }else if(lowerImag > parameter.trAvoidLowerImag 
                && lowerImag < parameter.trAvoidUpperImag
                && upperImag > parameter.trAvoidUpperImag){

            output.trAvoidUpperImag = upperImag;
        }
    }

    return output;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "avoiding_point_generator");
    ros::NodeHandle nh;
    ros::Rate r(60);

    Controller controller;
    // ros::Subscriber subTargetPose = nh.subscribe("move_base_simple/goal",100,
    //         &Controller::callbackTargetPose, &controller);
    // ros::Subscriber subTargetVel = nh.subscribe("move_base_simple/target_velocity",
    //         100, &Controller::callbackTargetVel, &controller);
    // ros::Subscriber subRealPose = nh.subscribe("odom", 100,
    //         &Controller::callbackRealPose, &controller);
    // ros::Subscriber subAIStatus = nh.subscribe("ai_status", 100,
    //         &Controller::callbackAIStatus, &controller);
    //

    // Dynamic Reconfigureの設定
    dynamic_reconfigure::Server<avoiding_point_generator::parameterConfig> reconfigure_server;
    dynamic_reconfigure::Server<avoiding_point_generator::parameterConfig>::CallbackType f;
    
    f = boost::bind(&Controller::callbackReconfigure, &controller, _1, _2);
    reconfigure_server.setCallback(f);
    //
    // // 起動時のnamespaceを取得
    // std::string ai_name = "/";
    // nh.getParam("ai_name", ai_name);
    //
    // std::vector<ros::Subscriber> subs;
    // for(int i=0; i< 12; i++){
    //     std::stringstream ss;
    //     ss << i;
    //     std::string topicName;
    //
    //     topicName = ai_name + "enemy_" + ss.str() + "/odom";
    //     subs.push_back(nh.subscribe(topicName.c_str(),100,
    //                 &Controller::callbackEnemyPose, &controller));
    //
    //     topicName = ai_name + "robot_" + ss.str() + "/odom";
    //     subs.push_back(nh.subscribe(topicName.c_str(),100,
    //                 &Controller::callbackFriendPose, &controller));
    // }
    //
    // ros::Subscriber subEnemyIDs = nh.subscribe(ai_name + "existing_enemies_id",100,
    //         &Controller::callbackEnemyIDs, &controller);
    // ros::Subscriber subFriendIDs = nh.subscribe(ai_name + "existing_friends_id",100,
    //         &Controller::callbackFriendIDs, &controller);
    // ros::Subscriber subBall = nh.subscribe(ai_name + "ball_observer/estimation",100,
    //         &Controller::callbackBallPoint, &controller);
    //
    // ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    // ros::Publisher pub_avoidPoint = nh.advertise<geometry_msgs::Point>("avoid_point",1000);

    while (ros::ok()){
        // controller.update();
        // pub_avoidPoint.publish(controller.getAvoidPoint());

        ROS_INFO("avoiding_point_generator");

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
