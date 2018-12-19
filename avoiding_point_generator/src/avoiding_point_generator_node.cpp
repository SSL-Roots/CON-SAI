#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <avoiding_point_generator/parameterConfig.h>
#include <vector>
#include <string>
#include <algorithm> //std::copy
#include <iterator> //std::back_inserter

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16MultiArray.h>
#include <consai_msgs/AIStatus.h>

#include "transformation.h"


// ロボット位置情報と衝突回避コンフィグを保管するグローバル空間
namespace GlobalInfo {
    std::vector<geometry_msgs::Point> targetPoints;
    std::vector<geometry_msgs::Point> friendPoints;
    std::vector<geometry_msgs::Point> enemyPoints;
    std::vector<int> friendIDs;
    std::vector<int> enemyIDs;
    geometry_msgs::Point ballPoint;
    geometry_msgs::Point friendUpperDefencePoint;
    geometry_msgs::Point friendLowerDefencePoint;
    geometry_msgs::Point enemyUpperDefencePoint;
    geometry_msgs::Point enemyLowerDefencePoint;

    // Obstacle Avoidance Config
    struct ObstacleAvoidanceConfig{
        double detectRange = 0.5;
        double avoidRange = 0.5;
        double startDetectionPos = 0.0;
        double avoidHysteresis = 0.18;
        double detectDefenceRange = 0.6;
        double avoidDefenceRange = 0.6;
    };

    ObstacleAvoidanceConfig OAConfig;

    void initPoints(const int id_max){
        targetPoints.resize(id_max);
        friendPoints.resize(id_max);
        enemyPoints.resize(id_max);
    }

    void callbackTargetPoint(const geometry_msgs::PoseStampedConstPtr &msg, const int id){
        targetPoints[id] = msg->pose.position;
    }

    void callbackFriendPoint(const nav_msgs::OdometryConstPtr &msg, const int id){
        friendPoints[id] = msg->pose.pose.position;
    }

    void callbackEnemyPoint(const nav_msgs::OdometryConstPtr &msg, const int id){
        enemyPoints[id] = msg->pose.pose.position;
    }

    void callbackFriendIDs(const std_msgs::UInt16MultiArrayConstPtr& msg){
        friendIDs.clear();
        std::copy(msg->data.begin(), msg->data.end(), std::back_inserter(friendIDs));
    }

    void callbackEnemyIDs(const std_msgs::UInt16MultiArrayConstPtr& msg){
        enemyIDs.clear();
        std::copy(msg->data.begin(), msg->data.end(), std::back_inserter(enemyIDs));
    }

    void callbackBallPoint(const nav_msgs::OdometryConstPtr &msg){
        ballPoint = msg->pose.pose.position;
    }
        
    void callbackReconfigure(avoiding_point_generator::parameterConfig &config, uint32_t level){
        OAConfig.detectRange = config.detectRange;
        OAConfig.avoidRange = config.avoidRange;
        OAConfig.startDetectionPos = config.startDetectionPos;
        OAConfig.avoidHysteresis = config.avoidHysteresis;
    }

    void initDefencePoint(){
        GlobalInfo::friendUpperDefencePoint.x = -4.0;
        GlobalInfo::friendUpperDefencePoint.y = 0.5;
        GlobalInfo::friendLowerDefencePoint.x = -4.0;
        GlobalInfo::friendLowerDefencePoint.y = -0.5;

        GlobalInfo::enemyUpperDefencePoint.x = 4.0;
        GlobalInfo::enemyUpperDefencePoint.y = 0.5;
        GlobalInfo::enemyLowerDefencePoint.x = 4.0;
        GlobalInfo::enemyLowerDefencePoint.y = -0.5;
    }
}


class ObstacleAvoidingPointGenerator{
    public:
        ObstacleAvoidingPointGenerator(){
        }

        void updateAvoidingPoint();

        geometry_msgs::Point getAvoidingPoint(){
            return mAvoidingPoint;
        }

        void setTargetPoint(const geometry_msgs::Point &point){
            mTargetPoint = point;
        }

        void setRobotPoint(const geometry_msgs::Point &point){
            mRobotPoint = point;
        }

        void callbackAIStatus(const consai_msgs::AIStatus &msg){
            mAIStatus = msg;
        }


    private:
        geometry_msgs::Point mTargetPoint;
        geometry_msgs::Point mRobotPoint;
        geometry_msgs::Point mAvoidingPoint;
        consai_msgs::AIStatus mAIStatus;
        bool mAvoidPointIsUpperImag;

        struct OAParameter{
            double trAvoidReal;
            double trAvoidUpperImag;
            double trAvoidLowerImag;
            double trGoalPosReal;
            bool avoiding;
            OAParameter(){
                trAvoidReal = 0;
                trAvoidUpperImag = 0;
                trAvoidLowerImag = 0;
                trGoalPosReal = 0;
                avoiding = false;
            }
        };

        OAParameter detectObstacle(const Transformation &trans, 
                const geometry_msgs::Point &point, const OAParameter &parameter,
                double detectRange = GlobalInfo::OAConfig.detectRange,
                double avoidRange = GlobalInfo::OAConfig.avoidRange);
        OAParameter detectObstacleOverlap(const Transformation &trans,
                const geometry_msgs::Point &point, const OAParameter &parameter,
                double avoidRange = GlobalInfo::OAConfig.avoidRange);
};


void ObstacleAvoidingPointGenerator::updateAvoidingPoint(){
    // 障害物の横に回避ポイントを作成する
    // 障害物が横並びの時は、大幅に避けるポイントを作成する
    

    // 自分の位置から目標位置までの座標系を作成
    Complex startPos = Tool::toComplex(mRobotPoint);
    Complex goalPos = Tool::toComplex(mTargetPoint);
    double angleToGoal = Tool::getAngle(goalPos - startPos);

    Transformation trans(startPos, angleToGoal);
    Complex trGoalPos = trans.transform(goalPos);

    // 自分に最も近い障害物の発見し、その回避位置を生成
    OAParameter parameter;
    parameter.trAvoidReal = trGoalPos.real();
    parameter.trGoalPosReal = trGoalPos.real();

    // 敵ロボット
    for(int id : GlobalInfo::enemyIDs){
        geometry_msgs::Point obstacle = GlobalInfo::enemyPoints[id];
        parameter = detectObstacle(trans, obstacle, parameter);
    }
    // 味方ロボット
    for(int id : GlobalInfo::friendIDs){
        geometry_msgs::Point obstacle = GlobalInfo::friendPoints[id];
        parameter = detectObstacle(trans, obstacle, parameter);
    }
    // ボール回避は任意
    if(mAIStatus.avoidBall){
        parameter = detectObstacle(trans, GlobalInfo::ballPoint, parameter);
    }
    
    // ディフェンスエリア回避は任意
    if(mAIStatus.avoidDefenceArea){
        parameter = detectObstacle(trans, GlobalInfo::friendUpperDefencePoint,
                parameter, GlobalInfo::OAConfig.detectDefenceRange,
                GlobalInfo::OAConfig.avoidDefenceRange);
        parameter = detectObstacle(trans, GlobalInfo::friendLowerDefencePoint,
                parameter, GlobalInfo::OAConfig.detectDefenceRange,
                GlobalInfo::OAConfig.avoidDefenceRange);
        parameter = detectObstacle(trans, GlobalInfo::enemyUpperDefencePoint,
                parameter, GlobalInfo::OAConfig.detectDefenceRange,
                GlobalInfo::OAConfig.avoidDefenceRange);
        parameter = detectObstacle(trans, GlobalInfo::enemyLowerDefencePoint,
                parameter, GlobalInfo::OAConfig.detectDefenceRange,
                GlobalInfo::OAConfig.avoidDefenceRange);
    }

    // 横並びロボット回避位置を生成
    // 障害物がなければ計算省略
    if(parameter.avoiding){
        // 横並び計算漏れを防ぐため計算をループさせる(ループ回数はテキトー)
        for(int loop=0; loop < 7; loop++){
            double old_upperImag = parameter.trAvoidUpperImag;
            double old_lowerImag = parameter.trAvoidLowerImag;

            // 敵ロボット
            for(int id : GlobalInfo::enemyIDs){
                geometry_msgs::Point obstacle = GlobalInfo::enemyPoints[id];
                parameter = detectObstacleOverlap(trans, obstacle, parameter);
            }
            // 見方ロボット
            for(int id : GlobalInfo::friendIDs){
                geometry_msgs::Point obstacle = GlobalInfo::friendPoints[id];
                parameter = detectObstacleOverlap(trans, obstacle, parameter);
            }
            // ボール回避は任意
            if(mAIStatus.avoidBall){
                parameter = detectObstacleOverlap(trans, GlobalInfo::ballPoint, parameter);
            }

            // ディフェンスエリア回避は任意
            if(mAIStatus.avoidDefenceArea){
                parameter = detectObstacleOverlap(trans, GlobalInfo::friendUpperDefencePoint,
                        parameter, GlobalInfo::OAConfig.avoidDefenceRange);
                parameter = detectObstacleOverlap(trans, GlobalInfo::friendLowerDefencePoint,
                        parameter, GlobalInfo::OAConfig.avoidDefenceRange);
                parameter = detectObstacleOverlap(trans, GlobalInfo::enemyUpperDefencePoint,
                        parameter, GlobalInfo::OAConfig.avoidDefenceRange);
                parameter = detectObstacleOverlap(trans, GlobalInfo::enemyLowerDefencePoint,
                        parameter, GlobalInfo::OAConfig.avoidDefenceRange);
            }

            // 回避位置がほとんど更新されなかったらループを抜ける
            if(Tool::isEqual(old_upperImag, parameter.trAvoidUpperImag) &&
                    Tool::isEqual(old_lowerImag, parameter.trAvoidLowerImag)){
                break;
            }
        }

        // 近い方のImagでAvoidingPointを生成する
        Complex trAvoidPos = trGoalPos;
        double upperSize = fabs(parameter.trAvoidUpperImag);
        double lowerSize = fabs(parameter.trAvoidLowerImag);
        
        // 左右の回避位置生成にヒステリシスをもたせる
        if(mAvoidPointIsUpperImag == true){
            if(lowerSize < upperSize - GlobalInfo::OAConfig.avoidHysteresis){
                mAvoidPointIsUpperImag = false;
            }
        }else{
            if(upperSize < lowerSize - GlobalInfo::OAConfig.avoidHysteresis){
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
        mAvoidingPoint.x = avoidPos.real();
        mAvoidingPoint.y = avoidPos.imag();
    }else{
        // 障害物がない場合は、targetPosをavoidingPointに設定する
        mAvoidingPoint = mTargetPoint;
    }
}


ObstacleAvoidingPointGenerator::OAParameter ObstacleAvoidingPointGenerator::detectObstacle(
        const Transformation &trans,
        const geometry_msgs::Point &point, const OAParameter &parameter,
        double detectRange, double avoidRange){

    OAParameter output = parameter;

    Complex obstaclePos = Tool::toComplex(point);
    Complex trObstPos = trans.transform(obstaclePos);

    // 自分と移動目標位置を結んだ直線上にいる障害物を見つける
    // 一番近い障害物を見つけるため
    // parameterのtrAvoidRealを障害物のrealで上書きしていく
    if(trObstPos.real() > GlobalInfo::OAConfig.startDetectionPos
            && trObstPos.real() < parameter.trAvoidReal
            && fabs(trObstPos.imag()) < detectRange){

        // 避けれる上下位置を計算
        double upperImag = trObstPos.imag() + avoidRange;
        double lowerImag = trObstPos.imag() - avoidRange;

        output.trAvoidReal = trObstPos.real();
        output.trAvoidUpperImag = upperImag;
        output.trAvoidLowerImag = lowerImag;
        output.avoiding = true;
    }

    return output;
}


ObstacleAvoidingPointGenerator::OAParameter ObstacleAvoidingPointGenerator::detectObstacleOverlap(
        const Transformation &trans,
        const geometry_msgs::Point &point, const OAParameter &parameter,
        double avoidRange){

    OAParameter output = parameter;

    Complex obstaclePos = Tool::toComplex(point);
    Complex trObstPos = trans.transform(obstaclePos);

    // 回避範囲(upper <-> lower)の重なりを見つけ、最端の回避位置を生成する
    if(trObstPos.real() > GlobalInfo::OAConfig.startDetectionPos
            && trObstPos.real() < parameter.trGoalPosReal){
        double upperImag = trObstPos.imag() + avoidRange;
        double lowerImag = trObstPos.imag() - avoidRange;

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

    // Dynamic Reconfigureの設定
    dynamic_reconfigure::Server<avoiding_point_generator::parameterConfig> reconfigure_server;
    dynamic_reconfigure::Server<avoiding_point_generator::parameterConfig>::CallbackType f;
    
    f = boost::bind(&GlobalInfo::callbackReconfigure, _1, _2);
    reconfigure_server.setCallback(f);

    // ID_MAXの初期化
    int ID_MAX = 12;
    ros::param::get("id_max", ID_MAX);
    GlobalInfo::initPoints(ID_MAX);

    std::vector<ObstacleAvoidingPointGenerator> generator(ID_MAX);

    // DefenceAreaを初期化
    GlobalInfo::initDefencePoint();

    // Subscriberを作成
    ros::Subscriber subFriendIDs = nh.subscribe("existing_friends_id", 100,
            GlobalInfo::callbackFriendIDs);
    ros::Subscriber subEnemyIDs = nh.subscribe("existing_enemies_id", 100,
            GlobalInfo::callbackEnemyIDs);
    ros::Subscriber subBallPoint = nh.subscribe("ball_observer/estimation", 100,
            GlobalInfo::callbackBallPoint);

    std::vector<ros::Subscriber> subs_targetPoint;
    std::vector<ros::Subscriber> subs_friendPoint;
    std::vector<ros::Subscriber> subs_enemyPoint;
    std::vector<ros::Subscriber> subs_AIStatus;

    // Publisherを作成
    std::vector<ros::Publisher> pubs_avoidingPoint;

    for(int i=0; i< ID_MAX; i++){
        std::string topicName;

        // callbackに引数を与えるため、boost::bindを使用
        topicName = "robot_" + std::to_string(i) + "/move_base_simple/goal";
        ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(
                topicName.c_str(), 100,
                boost::bind(GlobalInfo::callbackTargetPoint, _1, i));
        subs_targetPoint.push_back(sub);

        topicName = "robot_" + std::to_string(i) + "/odom";
        sub = nh.subscribe<nav_msgs::Odometry>(
                topicName.c_str(), 100,
                boost::bind(GlobalInfo::callbackFriendPoint, _1, i));
        subs_friendPoint.push_back(sub);

        topicName = "enemy_" + std::to_string(i) + "/odom";
        sub = nh.subscribe<nav_msgs::Odometry>(
                topicName.c_str(), 100,
                boost::bind(GlobalInfo::callbackEnemyPoint, _1, i));
        subs_enemyPoint.push_back(sub);

        topicName = "robot_" + std::to_string(i) + "/ai_status";
        sub = nh.subscribe(
                topicName.c_str(), 100,
                &ObstacleAvoidingPointGenerator::callbackAIStatus,
                &generator[i]);
        subs_AIStatus.push_back(sub);

        topicName = "robot_" + std::to_string(i) + "/avoiding_point";
        ros::Publisher pub = nh.advertise<geometry_msgs::Point>(topicName, 100);
        pubs_avoidingPoint.push_back(pub);
    }

    while (ros::ok()){
        // 存在している味方ロボットのavoidingPointを計算する
        for(int id : GlobalInfo::friendIDs){
            generator[id].setRobotPoint(GlobalInfo::friendPoints[id]);
            generator[id].setTargetPoint(GlobalInfo::targetPoints[id]);
            generator[id].updateAvoidingPoint();

            pubs_avoidingPoint[id].publish(generator[id].getAvoidingPoint());
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
