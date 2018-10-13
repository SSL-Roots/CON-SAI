#include  <world_observer/ball_estimator.hpp>


BallEstimator::BallEstimator(double loop_time) :
    dt(loop_time)
{
    initSystemModel();
    initMeasurementModel();

    /**************************** * Linear prior DENSITY     *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    ColumnVector prior_Mu(4);
    prior_Mu = 0.0;

    SymmetricMatrix prior_Cov(4);
    prior_Cov = 0.0;
    prior_Cov(1,1) = 100.0;
    prior_Cov(2,2) = 100.0;
    prior_Cov(3,3) = 100.0;
    prior_Cov(4,4) = 100.0;

    prior = new Gaussian (prior_Mu,prior_Cov);
    filter = new ExtendedKalmanFilter(prior);
}


void BallEstimator::initSystemModel( )
{
    // Create the matrices A and B for the linear system model
    Matrix A(4, 4);
    A(1,1) = 1.0;  A(1,2) = 0.0;  A(1,3) = dt;   A(1,4) = 0.0;
    A(2,1) = 0.0;  A(2,2) = 1.0;  A(2,3) = 0.0;  A(2,4) = dt;
    A(3,1) = 0.0;  A(3,2) = 0.0;  A(3,3) = 1.0;  A(3,4) = 0.0;
    A(4,1) = 0.0;  A(4,2) = 0.0;  A(4,3) = 0.0;  A(4,4) = 1.0;

    Matrix B(4,2);
    B(1,1) = 0.0;  B(1,2) = 0.0;
    B(2,1) = 0.0;  B(2,2) = 0.0;
    B(3,1) = dt;   B(3,2) = 0.0;
    B(4,1) = 0.0;  B(4,2) = dt;

    vector<Matrix> AB(2);
    AB[0] = A;
    AB[1] = B;

    // create gaussian
    ColumnVector sysNoise_Mu(4);
    sysNoise_Mu = 0.0;

    SymmetricMatrix sysNoise_Cov(4);
    sysNoise_Cov = 0.0;
    sysNoise_Cov(1,1) = pow(0.1, 2);
    sysNoise_Cov(2,2) = pow(0.1, 2);
    sysNoise_Cov(3,3) = pow(5.0, 2);
    sysNoise_Cov(4,4) = pow(5.0, 2);

    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

    // create the model
    this->sys_pdf = new LinearAnalyticConditionalGaussian(AB, system_Uncertainty);
    this->sys_model = new  LinearAnalyticSystemModelGaussianUncertainty(this->sys_pdf);
}


void BallEstimator::initMeasurementModel() {
    // create matrix H for linear measurement model
    Matrix H(2, 4);
    H = 0.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;

    // Construct the measurement noise
    ColumnVector measNoise_Mu(2);
    measNoise_Mu = 0.0;

    SymmetricMatrix measNoise_Cov(2);
    measNoise_Cov(1,1) = pow(0.01, 2);
    measNoise_Cov(2,2) = pow(0.01, 2);
    Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

    // create the model
    meas_pdf = new LinearAnalyticConditionalGaussian(H, measurement_Uncertainty);
    meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty (meas_pdf);
}



void BallEstimator::predict(ColumnVector input)
{
    Estimation  est;

    this->filter->Update(sys_model, input);
    est = getResult();

    this->last_estimation = est;
}


void BallEstimator::update(ColumnVector measurement)
{
    Estimation  est;

    this->filter->Update(meas_model, measurement);
    est = getResult();

    this->last_estimation = est;
}


BallEstimator::Estimation BallEstimator::getResult()
{
    Estimation  est;
    Pdf<ColumnVector> * posterior = filter->PostGet();

    est.val = posterior->ExpectedValueGet();
    est.cov = posterior->CovarianceGet();

    return  est;
}


ColumnVector  BallEstimator::convertPoseMsgToMeasureVector(geometry_msgs::Pose pose)
{
    ColumnVector  measurement(2);

    measurement(1) = pose.position.x;
    measurement(2) = pose.position.y;
    return  measurement;
}


ColumnVector  BallEstimator::convertAccelMsgToInputVector(geometry_msgs::Accel acc)
{
    // TODO : Inpment
    ColumnVector  vec(2);

    vec = 0;
    return  vec;
}



nav_msgs::Odometry  BallEstimator::convetEstimationToOdometry()
{
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = this->last_estimation.val(1);
    odom.pose.pose.position.y = this->last_estimation.val(2);

    odom.twist.twist.linear.x = this->last_estimation.val(3);
    odom.twist.twist.linear.y = this->last_estimation.val(4);

    return  odom;
}


bool BallEstimator::isOutlier(ColumnVector measurement){
    // Reference: https://myenigma.hatenablog.com/entry/20140825/1408975706

    double mahala_dist = mahalanobisDistance(measurement);
    double thresh = 3.84146; //自由度1、棄却率5%のしきい値
     
    if(mahala_dist > thresh){
        return true;
    }
    return false;
}


double BallEstimator::mahalanobisDistance(ColumnVector measurement){
    double measurementX= measurement(1);
    double measurementY= measurement(2);
    double estimationX = this->last_estimation.val(1);
    double estimationY = this->last_estimation.val(2);
    double covarianceX = this->last_estimation.cov(1,1);
    double covarianceY = this->last_estimation.cov(2,2);

    double diffX = measurementX - estimationX;
    double diffY = measurementY - estimationY;

    // avoid 0 division
    if(covarianceX == 0 || covarianceY == 0){
        return 0;
    }

    return sqrt(pow(diffX, 2)/covarianceX + pow(diffY, 2)/covarianceY);
}

BallEstimator::~BallEstimator()
{
    delete  this->sys_pdf;
    delete  this->sys_model;
    delete  this->meas_pdf;
    delete  this->meas_model;
    delete  this->prior;
    delete  this->filter;
}
