#include "trapezoidal_controller.h"
#include <cmath>
#include <ros/ros.h>

Control::STATE TrapezoidalController::nextState(const double &currentPose, 
        const double &currentVelocity, const double &targetPose){
    double residual = targetPose - currentPose;

    if(mIsRotation){
        residual = normalize(residual);
    }

    double fabsResidual = std::fabs(residual);

    mResidual = residual;

    if(fabsResidual < mHysteresis){
        return Control::BRAKING;
    }

    if(!sameSign(mVelocity, residual)){
        return Control::DECELERATING;
    }

    double brakingDist = brakingDistance(currentVelocity);

    if(fabsResidual < brakingDist){
        return Control::BRAKING;
    }else{
        return Control::ACCELERATING;
    }

    return Control::KEEP_SPEED;
}

Control::STATE TrapezoidalController::nextState(const double &targetVelocity){
    double fabsTargetVelocity = std::fabs(targetVelocity);
    double fabsCurrentVelocity = std::fabs(mVelocity);

    if(fabsTargetVelocity < mAccel){
        return Control::BRAKING;
    }

    if(!sameSign(targetVelocity, mVelocity)){
        return Control::DECELERATING;
    }

    if(fabsTargetVelocity < fabsCurrentVelocity){
        return Control::DECELERATING;
    }else if(fabsTargetVelocity > fabsCurrentVelocity){
        return Control::ACCELERATING;
    }

    return Control::KEEP_SPEED;
}

bool TrapezoidalController::sameSign(const double &v1, const double &v2){
    return (v1 >= 0) ^ (v2 < 0);
}

double TrapezoidalController::brakingDistance(const double &currentVelocity){
    return 0.5 * (currentVelocity / mAccel) * mTimePeriod * currentVelocity;
    // return 0.5 * (mVelocity / mDecel) * mTimePeriod * mVelocity;
}

double TrapezoidalController::accelerate(const double &velocity){
    double output = velocity + std::copysign(mAccel, velocity);

    if(std::fabs(output) > mMaxVelocity){
        output = std::copysign(mMaxVelocity, velocity);
    }
    return output;
}

double TrapezoidalController::decelerate(const double &velocity){
    return velocity - std::copysign(mDecel, velocity);
}

double TrapezoidalController::brake(const double &velocity){
    double output = velocity - std::copysign(mDecel, velocity);

    // 速度の方向が変わったら速度を0にする
    if(!sameSign(output, velocity)){
        output = 0;
    }

    return output;
}

double TrapezoidalController::normalize(const double &angle){
    double tmpAngle = angle;
    while(tmpAngle > M_PI){
        tmpAngle -= 2*M_PI;
    }
    while(tmpAngle < -M_PI){
        tmpAngle += 2*M_PI;
    }
    return tmpAngle;
}

TrapezoidalController::TrapezoidalController(const double &accel, 
        const double &decel, const double &maxVelocity, 
        const double &hysteresis, const bool &isRotation){
    mVelocity = 0.0;
    mAccel = accel;
    mDecel = decel;
    mMaxVelocity = maxVelocity;
    mHysteresis = hysteresis;
    mTimePeriod = 0.01666;
    mState = Control::INITIALIZED;
    mIsRotation = isRotation;
}

void TrapezoidalController::setParameters(const double &accel, 
        const double &decel, const double &maxVelocity, const double &hysteresis){
    mAccel = accel;
    mDecel = decel;
    mMaxVelocity = maxVelocity;
    mHysteresis = hysteresis;
}

bool TrapezoidalController::update(const double &currentPose, 
        const double &currentVelocity, const double &targetPose){
    // Position Control
    Control::STATE state = nextState(currentPose, currentVelocity, targetPose);

    double velocity = mVelocity;
    if(state == Control::ACCELERATING){
        velocity = accelerate(velocity);
    }else if(state == Control::DECELERATING){
        velocity = decelerate(velocity);
    }else if(state == Control::BRAKING){
        velocity = brake(velocity);
    }

    mCurrentAccel = velocity - mVelocity;
    mVelocity = velocity;

    return true;
}

bool TrapezoidalController::update(const double &targetVelocity){
    // Velocity Control
    Control::STATE state = nextState(targetVelocity);

    double velocity = mVelocity;
    if(state == Control::ACCELERATING){
        velocity = accelerate(velocity);
    }else if(state == Control::DECELERATING){
        velocity = decelerate(velocity);
    }else if(state == Control::BRAKING){
        velocity = brake(velocity);
    }

    mCurrentAccel = velocity - mVelocity;
    mVelocity = velocity;

    return true;
}

double TrapezoidalController::getVelocity(void){
    return mVelocity;
}

void TrapezoidalController::setVelocity(const double &velocity){
    mVelocity = velocity;
}

double TrapezoidalController::getResidual(void){
    return mResidual;
}

double TrapezoidalController::getCurrentAccel(void){
    return mCurrentAccel;
}
