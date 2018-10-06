#include  <iostream>

#include <world_observer/estimator.hpp>



Estimator::Estimator()
{
}



nav_msgs::Odometry  Estimator::estimate()
{
    std::vector<geometry_msgs::Pose> null_poses;

    return  this->estimate(null_poses);
}


nav_msgs::Odometry  Estimator::estimate(const std::vector<geometry_msgs::Pose>& poses)
{
    geometry_msgs::Accel  null_acc;

    return  this->estimate(null_acc, poses);
}


nav_msgs::Odometry  Estimator::estimate(geometry_msgs::Accel acc, const std::vector<geometry_msgs::Pose>& poses)
{
    // System update by only system model with input
    predict(convertAccelMsgToInputVector(acc));

    for (size_t i = 0; i < poses.size(); i++) {
        ColumnVector  measurement = convertPoseMsgToMeasureVector(poses.at(i));

        if (isOutlier(measurement)) {
            continue;
        }

        update(measurement);
    }

    return  convetEstimationToOdometry();
    }


    Estimator::~Estimator()
    {
    }
