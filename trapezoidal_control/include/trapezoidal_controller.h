#ifndef TRAPEZOIDAL_CONTROLLER_H
#define TRAPEZOIDAL_CONTROLLER_H

namespace Control{
enum STATE{
    BEFORE_INIT,
    INITIALIZED,
    ACCELERATING,
    KEEP_SPEED,
    DECELERATING,
    BRAKING
};
}

class TrapezoidalController {
    private:
        double mVelocity;   // unit:m/s
        double mAccel;      // unit:m/s^2
        double mDecel;      // unit:m/s^2
        double mMaxVelocity;    // unit:m/s
        double mTimePeriod;     // unit:second
        double mHysteresis;     // unit:meter
        double mResidual;   // unit:meter
        Control::STATE mState;
        bool mIsRotation;

        Control::STATE nextState(const double &currentPose, 
                const double &currentVelocity, const double &targetPose);
        Control::STATE nextState(const double &targetVelocity);

        bool sameSign(const double &v1, const double &v2);
        double brakingDistance(const double &currentvelocity);
        double accelerate(const double &velocity);
        double decelerate(const double &velocity);
        double brake(const double &velocity);
        
        double normalize(const double &angle);
    
    public:
        TrapezoidalController(const double &accel, const double &decel, 
                const double &maxVelocity, const double &hysteresis, 
                const bool &isRotation);
        ~TrapezoidalController() {};
        void setParameters(const double &accel, const double &decel, 
                const double &maxVelocity, const double &hysteresis);
        bool update(const double &currentPose, const double &currentVelocity, 
                const double &targetPose);
        bool update(const double &targetVelocity);
        double getVelocity(void);
        void setVelocity(const double &velocity);
        double getResidual(void);
};

#endif
