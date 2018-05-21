#ifndef TRAPEZOIDAL_CONTROLLER_H
#define TRAPEZOIDAL_CONTROLLER_H

class TrapezoidalController 
{
    private:
        double mVelocity;
        double mAccel;
        double mDecel;
        double mInitialVelocity;
        double mTimePeriod;

    
    public:
        TrapezoidalController() {};
        ~TrapezoidalController() {};
        bool update(const double &target);
        double getVelocity(void);

};



#endif
