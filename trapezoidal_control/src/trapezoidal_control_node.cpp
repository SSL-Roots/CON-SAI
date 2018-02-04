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
#include <consai_msgs/AIStatus.h>


class Controller{
    public:
        Controller();
        void update();
        
        geometry_msgs::Twist getCommandVelocity();
        geometry_msgs::Point getAvoidingPoint();

        void callbackTargetPose(const geometry_msgs::PoseStampedConstPtr& msg);
        void callbackTargetVel(const geometry_msgs::TwistStampedConstPtr& msg);
        void callbackRealPose(const nav_msgs::OdometryConstPtr& msg);
        void callbackReconfigure(trapezoidal_control::parameterConfig &config, uint32_t level);
        void callbackAIStatus(const consai_msgs::AIStatus& msg);
        void callbackAvoidingPoint(const geometry_msgs::Point& msg);

    private:
        const double mPeriod_;
        geometry_msgs::Twist mCommandVel;
        geometry_msgs::Pose mTargetPose;
        geometry_msgs::Twist mTargetVel;
        geometry_msgs::Pose mRealPose;
        geometry_msgs::Twist mRealVel;
        consai_msgs::AIStatus mAIStatus;
        geometry_msgs::Point mAvoidingPoint;

        bool mIsVelocityControl;
        double mPrevSpeed;
        double mAccSpeed;
        double mMaxSpeed;
        double mDecSpeedGain;
        double mDirecLimit;
        double mSpeedLimit;

        double mPrevRotation;
        double mAccRotation;
        double mMaxRotation;
        double mDecRotationGain;
        double mRotationDirec;
        double mCentrifugalLimit;

        void poseControl();
        geometry_msgs::Vector3 trapezoidalLinearControl(const geometry_msgs::Point &targetPos);
        geometry_msgs::Vector3 trapezoidalAngularControl();
        geometry_msgs::Twist velocityControl(const geometry_msgs::Twist &targetVel);

        double normalize(double angle);
        double yawFromQuaternion(geometry_msgs::Quaternion geoQ);
};

Controller::Controller()
    :mPeriod_(0.016){

    mIsVelocityControl = false;
    mPrevSpeed = 0.0;
    mAccSpeed = 0.005;
    mMaxSpeed = 2.0;
    mDecSpeedGain = 0.5;
    mDirecLimit = 15.0 * M_PI / 180.0;
    mSpeedLimit = 0.3;

    mPrevRotation = 0.0;
    mAccRotation = 0.01;
    mMaxRotation = 2.0;
    mDecRotationGain = 0.2;
    mRotationDirec = 1.0;
    mCentrifugalLimit = 0.01;
}

void Controller::update(){
    if(mIsVelocityControl){
        mCommandVel = velocityControl(mTargetVel);
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
    mAccSpeed = config.accSpeed;
    mMaxSpeed = config.maxSpeed;
    mDecSpeedGain = config.decSpeedGain;
    mDirecLimit = config.direcLimit * M_PI / 180.0;
    mSpeedLimit = config.speedLimit;

    mAccRotation = config.accRotation;
    mMaxRotation = config.maxRotation;
    mDecRotationGain = config.decRotationGain;
    mCentrifugalLimit = config.centrifugalLimit;
}

void Controller::callbackAIStatus(const consai_msgs::AIStatus& msg){
    mAIStatus = msg;
}

void Controller::callbackAvoidingPoint(const geometry_msgs::Point& msg){
    mAvoidingPoint = msg;
}

void Controller::poseControl(){
    geometry_msgs::Vector3 linearVel = trapezoidalLinearControl(mAvoidingPoint);

    // 移動方向をロボット座標系に変換
    double realYaw = yawFromQuaternion(mRealPose.orientation);
    
    mCommandVel.linear.x = cos(-realYaw)*linearVel.x - sin(-realYaw)*linearVel.y;
    mCommandVel.linear.y = sin(-realYaw)*linearVel.x + cos(-realYaw)*linearVel.y;

    mCommandVel.angular = trapezoidalAngularControl();
}

geometry_msgs::Vector3 Controller::trapezoidalLinearControl(const geometry_msgs::Point &targetPos){
    // ロボット位置と目標位置から、移動方向を求める
    double targX,targY,realX,realY,diffX,diffY;
    targX = targetPos.x;
    targY = targetPos.y;
    realX = mRealPose.position.x;
    realY = mRealPose.position.y;
    diffX = targX-realX;
    diffY = targY-realY;

    double targetDirection = atan2(diffY, diffX);

    // ロボット速度方向と移動方向の方向差を求める
    double realVX, realVY;
    realVX = mRealVel.linear.x;
    realVY = mRealVel.linear.y;

    double velDirection =std::atan2(realVY, realVX);
    double diffDirection = normalize(targetDirection - velDirection);
    
    // ロボット位置と目標位置から移動距離を求める
    double targetDistance;
    targetDistance = sqrt(diffX*diffX + diffY*diffY);

    // 一つ前の制御速度とロボット速度の大きさをもとに制動距離を求める
    double realSpeed = sqrt(realVX*realVX + realVY*realVY);
    double brakingDistance;
    brakingDistance = 0.5 * (mPrevSpeed/mAccSpeed) * mPeriod_ * mPrevSpeed 
        + mDecSpeedGain*realSpeed;

    // ロボットの速度の大きさから、移動方向を変えられるかを判断する
    // ロボットの速度が大きく、かつ速度と移動方向の角度差が大きければ
    // 制動距離を1000000にして、強制的に減速する
    if(fabs(diffDirection) > mDirecLimit && realSpeed > mSpeedLimit){
        brakingDistance = 100000;
        targetDirection = velDirection;
    }
    
    // 1:移動距離が制動距離以上であれば加速する
    double targetSpeed = mPrevSpeed;
    if(targetDistance > brakingDistance){
        targetSpeed += mAccSpeed;
        // 2:ロボット速度が限界値であれば速度を維持する
        if(targetSpeed > mMaxSpeed){
            targetSpeed -= mAccSpeed;
            // targetSpeed = mMaxSpeed;
        }
    }else{
    // 3:移動距離が制動距離以下であれば減速する
        targetSpeed -= mAccSpeed;
        if(targetSpeed < 0){
            targetSpeed = 0.0;
        }
    }
    mPrevSpeed = targetSpeed;
    
    geometry_msgs::Vector3 output;
    output.x = mPrevSpeed*cos(targetDirection);
    output.y = mPrevSpeed*sin(targetDirection);
    output.z = 0;

    return output;
}

geometry_msgs::Vector3 Controller::trapezoidalAngularControl(){
    // ロボット角度と目標角度から回転角を求める
    double targetYaw, realYaw, diffYaw;
    targetYaw = yawFromQuaternion(mTargetPose.orientation);
    realYaw = yawFromQuaternion(mRealPose.orientation);

    diffYaw = normalize(targetYaw - realYaw);

    // 前回の回転方向と目標回転方向を比較する
    if(diffYaw * mRotationDirec < 0){
        // 1:逆方向なら減速する
        mPrevRotation -= mAccRotation;

        // 回転速度が小さくなったら回転方向を逆にする
        if(mPrevRotation < 0.1){
            mRotationDirec *= -1.0;
        }
    }else{
        // 2:同方向なら通常制御
        // 一つ前の制御角速度とロボット角速度の大きさをもとに制動角を求める
        double realSpeed = std::fabs(realYaw);
        double brakingYaw;
        brakingYaw = 0.5 * (mPrevRotation/mAccRotation) * mPeriod_ * mPrevRotation
            + mDecRotationGain * realSpeed;
        
        // 1:回転角が制動角以上であれば加速する
        double targetRotation = mPrevRotation;
        double sizeDiffYaw = fabs(diffYaw);

        // 遠心力リミッター
        double limiRotation = mMaxRotation;
        double centrifugal = mPrevSpeed * mPrevRotation;

        if(centrifugal > mCentrifugalLimit){
            limiRotation = mCentrifugalLimit / mPrevSpeed;
        }

        if(sizeDiffYaw > brakingYaw){
            targetRotation += mAccRotation;
            // 2:制御角速度が限界値であれば角速度を維持する
            if(targetRotation > limiRotation){
                targetRotation = limiRotation;
            }
        }else{
            // 3:回転角が制動角以下であれば減速する
            targetRotation -= mAccRotation;
            if(targetRotation < 0){
                targetRotation = 0.0;
            }
        }
        mPrevRotation = targetRotation;
    }

    geometry_msgs::Vector3 output;
    output.x = 0.0;
    output.y = 0.0;
    output.z = mRotationDirec * mPrevRotation;

    return output;
}

geometry_msgs::Twist Controller::velocityControl(const geometry_msgs::Twist &targetVel){
    double targetVelX = targetVel.linear.x;
    double targetVelY = targetVel.linear.y;
    double targetRotation = targetVel.angular.z;
    
    // 目標速度が(0, 0)の時は、前回の移動方向を維持しながら減速
    double velDirection;
    if(targetVelX == 0 && targetVelY == 0){
        velDirection = atan2(mCommandVel.linear.y, mCommandVel.linear.x);
    }else{
        velDirection = atan2(targetVelY,targetVelX);
    }

    double targetSpeed = sqrt(targetVelX*targetVelX + targetVelY*targetVelY);

    double commandSpeed = mPrevSpeed;
    if(targetSpeed > commandSpeed){
        commandSpeed += mAccSpeed;
        if(commandSpeed > mMaxSpeed){
            commandSpeed = mMaxSpeed;
        }
    }
    if(targetSpeed < commandSpeed){
        if(targetSpeed == 0.0){
            commandSpeed -= 3.0*mAccSpeed;
        }else{
            commandSpeed -= mAccSpeed;
        }
        if(commandSpeed < 0){
            commandSpeed = 0.0;
        }
    }
    mPrevSpeed = commandSpeed;

    double commandRotation = mPrevRotation;
    if(targetRotation > commandRotation){
        commandRotation += mAccRotation;
        if(commandRotation > mMaxRotation){
            commandRotation = mMaxRotation;
        }
    }
    if(targetRotation < commandRotation){
        commandRotation -= mAccRotation;
        if(commandRotation < -mMaxRotation){
            commandRotation = -mMaxRotation;
        }
    }
    mPrevRotation = commandRotation;

    geometry_msgs::Twist output;
    output.linear.x = commandSpeed*cos(velDirection);
    output.linear.y = commandSpeed*sin(velDirection);
    output.angular.z = commandRotation;
    
    return output;
}

double Controller::normalize(double angle){
    while(angle > M_PI){
        angle -= 2*M_PI;
    }
    while(angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

double Controller::yawFromQuaternion(geometry_msgs::Quaternion geoQ){
    tf::Quaternion q(geoQ.x, geoQ.y, geoQ.z, geoQ.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(isnan(yaw)){
        yaw = 0.0;
    }
    return yaw;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trapezoidal_control");
    ros::NodeHandle nh;
    ros::Rate r(60);

    Controller controller;
    ros::Subscriber subTargetPose = nh.subscribe("move_base_simple/goal",100,
            &Controller::callbackTargetPose, &controller);
    ros::Subscriber subTargetVel = nh.subscribe("move_base_simple/target_velocity",
            100, &Controller::callbackTargetVel, &controller);
    ros::Subscriber subRealPose = nh.subscribe("odom", 100,
            &Controller::callbackRealPose, &controller);
    ros::Subscriber subAIStatus = nh.subscribe("ai_status", 100,
            &Controller::callbackAIStatus, &controller);
    ros::Subscriber subAvoidingPoint = nh.subscribe("avoiding_point", 100,
            &Controller::callbackAvoidingPoint, &controller);

    dynamic_reconfigure::Server<trapezoidal_control::parameterConfig> reconfigure_server;
    dynamic_reconfigure::Server<trapezoidal_control::parameterConfig>::CallbackType f;

    f = boost::bind(&Controller::callbackReconfigure, &controller, _1, _2);
    reconfigure_server.setCallback(f);

    ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    while (ros::ok()){
        controller.update();
        geometry_msgs::Twist cmdVel = controller.getCommandVelocity();

        publisher.publish(cmdVel);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
