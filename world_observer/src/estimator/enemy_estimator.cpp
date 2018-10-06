#include  <world_observer/enemy_estimator.hpp>

#include  <tf/transform_datatypes.h>

EnemyEstimator::EnemyEstimator(double loop_time) :
    dt(loop_time)
{
    initSystemModel();
    initMeasurementModel();

    /****************************
     * Linear prior DENSITY     *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    ColumnVector prior_Mu(6);
    prior_Mu = 0.0;

    SymmetricMatrix prior_Cov(6);
    prior_Cov = 0.0;
    prior_Cov(1,1) = 100.0;
    prior_Cov(2,2) = 100.0;
    prior_Cov(3,3) = 100.0;
    prior_Cov(4,4) = 100.0;
    prior_Cov(5,5) = 100.0;
    prior_Cov(6,6) = 100.0;

    prior = new Gaussian (prior_Mu,prior_Cov);
    filter = new ExtendedKalmanFilter(prior);
}


void EnemyEstimator::initSystemModel( )
{
    // Create the matrices A and B for the linear system model
    Matrix A(6,6);
    A(1,1) = 1.0;  A(1,2) = 0.0;  A(1,3) = 0.0;  A(1,4) = dt;   A(1,5) = 0.0;  A(1,6) = 0.0;
    A(2,1) = 0.0;  A(2,2) = 1.0;  A(2,3) = 0.0;  A(2,4) = 0.0;  A(2,5) = dt;   A(2,6) = 0.0;
    A(3,1) = 0.0;  A(3,2) = 0.0;  A(3,3) = 1.0;  A(3,4) = 0.0;  A(3,5) = 0.0;  A(3,6) = dt;
    A(4,1) = 0.0;  A(4,2) = 0.0;  A(4,3) = 0.0;  A(4,4) = 1.0;  A(4,5) = 0.0;  A(4,6) = 0.0;
    A(5,1) = 0.0;  A(5,2) = 0.0;  A(5,3) = 0.0;  A(5,4) = 0.0;  A(5,5) = 1.0;  A(5,6) = 0.0;
    A(6,1) = 0.0;  A(6,2) = 0.0;  A(6,3) = 0.0;  A(6,4) = 0.0;  A(6,5) = 0.0;  A(6,6) = 1.0;

    Matrix B(6,3);
    B(1,1) = 0.0;  B(1,2) = 0.0;  B(1,3) = 0.0;
    B(2,1) = 0.0;  B(2,2) = 0.0;  B(2,3) = 0.0;
    B(3,1) = 0.0;  B(3,2) = 0.0;  B(3,3) = 0.0;
    B(4,1) = dt;   B(4,2) = 0.0;  B(4,3) = 0.0;
    B(5,1) = 0.0;  B(5,2) = dt;   B(5,3) = 0.0;
    B(6,1) = 0.0;  B(6,2) = 0.0;  B(6,3) = dt;

    vector<Matrix> AB(2);
    AB[0] = A;
    AB[1] = B;

    // create gaussian
    ColumnVector sysNoise_Mu(6);
    sysNoise_Mu = 0.0;

    SymmetricMatrix sysNoise_Cov(6);
    sysNoise_Cov = 0.0;
    sysNoise_Cov(1,1) = pow(0.01, 2);
    sysNoise_Cov(2,2) = pow(0.01, 2);
    sysNoise_Cov(3,3) = pow(0.01, 2);
    sysNoise_Cov(4,4) = pow(0.01, 2);
    sysNoise_Cov(5,5) = pow(0.01, 2);
    sysNoise_Cov(6,6) = pow(0.01, 2);

    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

    // create the model
    this->sys_pdf = new LinearAnalyticConditionalGaussian(AB, system_Uncertainty);
    this->sys_model = new  LinearAnalyticSystemModelGaussianUncertainty(this->sys_pdf);
}


void EnemyEstimator::initMeasurementModel() {
    // create matrix H for linear measurement model
    Matrix H(3,6);
    H = 0.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;
    H(3,3) = 1.0;

    // Construct the measurement noise
    ColumnVector measNoise_Mu(3);
    measNoise_Mu = 0.0;

    SymmetricMatrix measNoise_Cov(3);
    measNoise_Cov(1,1) = pow(0.01, 2);
    measNoise_Cov(2,2) = pow(0.01, 2);
    measNoise_Cov(3,3) = pow(0.01, 2);
    Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

    // create the model
    meas_pdf = new LinearAnalyticConditionalGaussian(H, measurement_Uncertainty);
    meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty (meas_pdf);
}



void EnemyEstimator::predict(ColumnVector input)
{
    Estimation  est;

    this->filter->Update(sys_model, input);
    est = getResult();
    collectAngleOverflow(est.val, est.cov);

    this->last_estimation = est;
}


void EnemyEstimator::update(ColumnVector measurement)
{
    Estimation  est;

    this->filter->Update(meas_model, measurement);
    est = getResult();

    collectAngleOverflow(est.val, est.cov);

    this->last_estimation = est;
}


EnemyEstimator::Estimation EnemyEstimator::getResult()
{
    Estimation  est;
    Pdf<ColumnVector> * posterior = filter->PostGet();

    est.val = posterior->ExpectedValueGet();
    est.cov = posterior->CovarianceGet();

    return  est;
}


void EnemyEstimator::collectAngleOverflow(ColumnVector& state, SymmetricMatrix& cov)
{
    if (state(3) < -M_PI || state(3) > M_PI) {
        state(3) = EulerAngle::normalize(state(3));

        this->prior->ExpectedValueSet(state);
        this->prior->CovarianceSet(cov);

        filter->Reset(prior);
    }
}


ColumnVector  EnemyEstimator::convertPoseMsgToMeasureVector(geometry_msgs::Pose pose)
{
    ColumnVector  measurement(3);
    double  roll, pitch, yaw;

    tf::Quaternion  q(pose.orientation.x, pose.orientation.y,pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    measurement(1) = pose.position.x;
    measurement(2) = pose.position.y;
    measurement(3) = yaw;

    // collect angle continuity
    measurement(3) = EulerAngle::normalize(measurement(3), this->last_estimation.val(3));

    return  measurement;
}


ColumnVector  EnemyEstimator::convertAccelMsgToInputVector(geometry_msgs::Accel acc)
{
    ColumnVector  vec(3);

    vec(1) = acc.linear.x;
    vec(2) = acc.linear.y;
    vec(3) = acc.angular.z;

    return  vec;
}



nav_msgs::Odometry  EnemyEstimator::convetEstimationToOdometry()
{
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = this->last_estimation.val(1);
    odom.pose.pose.position.y = this->last_estimation.val(2);

    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->last_estimation.val(3));

    odom.twist.twist.linear.x = this->last_estimation.val(4);
    odom.twist.twist.linear.y = this->last_estimation.val(5);
    odom.twist.twist.angular.z = this->last_estimation.val(6);

    return  odom;
}


nav_msgs::Odometry  EnemyEstimator::convetStateVectorToOdometry(ColumnVector state_vector)
{
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = state_vector(1);
    odom.pose.pose.position.y = state_vector(2);

    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_vector(3));

    odom.twist.twist.linear.x = state_vector(4);
    odom.twist.twist.linear.y = state_vector(5);
    odom.twist.twist.angular.z = state_vector(6);

    return  odom;
}


bool EnemyEstimator::isOutlier(ColumnVector measurement){
    return false;
}


double EnemyEstimator::mahalanobisDistance(ColumnVector measurement){
    return 0;
}


EnemyEstimator::~EnemyEstimator()
{
    delete  this->sys_pdf;
    delete  this->sys_model;
    delete  this->meas_pdf;
    delete  this->meas_model;
    delete  this->prior;
    delete  this->filter;
}



/**********************************************************
 * This is implementation of EulerAngle class
 ***********************************************************/
double  EulerAngle::normalize(double angle)
{
    while(angle>=M_PI)  angle-=2.0*M_PI;
    while(angle<=-M_PI) angle+=2.0*M_PI;
    return angle;
}


double  EulerAngle::normalize(double angle, double center)
{
    return  center + normalize(angle - center);
}
