#ifndef _BALL_ESTIMATOR_HPP_
#define _BALL_ESTIMATOR_HPP_

#include  <world_observer/estimator.hpp>

class BallEstimator : public Estimator
{
    public:
        //constructor
        BallEstimator(double loop_time);
        ~BallEstimator();

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

};

#endif
