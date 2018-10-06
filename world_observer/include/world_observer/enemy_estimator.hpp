#ifndef _ENEMY_ESTIMATOR_HPP_
#define _ENEMY_ESTIMATOR_HPP_

#include  <world_observer/estimator.hpp>

class EnemyEstimator : public Estimator
{
    public:
        // constructor
        EnemyEstimator(double loop_time);
        ~EnemyEstimator();

    protected:
        double  dt;

        LinearAnalyticConditionalGaussian* sys_pdf;
        LinearAnalyticSystemModelGaussianUncertainty* sys_model;
        LinearAnalyticConditionalGaussian* meas_pdf;
        LinearAnalyticMeasurementModelGaussianUncertainty* meas_model;
        KalmanFilter* filter;
        Gaussian* prior;

        void initSystemModel();
        void initMeasurementModel();

        void  predict(ColumnVector input);
        void  update(ColumnVector measurement);

        ColumnVector  convertPoseMsgToMeasureVector(geometry_msgs::Pose pose);
        ColumnVector  convertAccelMsgToInputVector(geometry_msgs::Accel acc);
        nav_msgs::Odometry  convetEstimationToOdometry();

        bool isOutlier(ColumnVector measurement);
        double mahalanobisDistance(ColumnVector measurement);

        Estimation getResult();

        nav_msgs::Odometry  convetStateVectorToOdometry(ColumnVector state_vector);
        void collectAngleOverflow(ColumnVector& state, SymmetricMatrix& cov);
        double  pi2pi(double angle);


};



class EulerAngle
{
    public:
        static double  normalize(double angle);
        static double  normalize(double angle, double center);
};

#endif
