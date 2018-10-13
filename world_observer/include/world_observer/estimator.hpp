#ifndef _ESTIMATOR_HPP_
#define _ESTIMATOR_HPP_

#include  "ros/ros.h"
#include  <geometry_msgs/Pose.h>
#include  <geometry_msgs/Accel.h>
#include  <nav_msgs/Odometry.h>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;


class Estimator
{
    public:
        Estimator();
        ~Estimator();

        nav_msgs::Odometry  estimate();
        nav_msgs::Odometry  estimate(const std::vector<geometry_msgs::Pose>& poses);
        nav_msgs::Odometry  estimate(geometry_msgs::Accel acc, const std::vector<geometry_msgs::Pose>& poses);

    protected:
        class Estimation {
            public:
                ColumnVector  val;
                SymmetricMatrix cov;
        };

        Estimation  last_estimation;

        virtual void  predict(ColumnVector input) = 0;
        virtual void  update(ColumnVector measurement) = 0;

        virtual ColumnVector  convertPoseMsgToMeasureVector(geometry_msgs::Pose pose) = 0;
        virtual ColumnVector  convertAccelMsgToInputVector(geometry_msgs::Accel acc) = 0;
        virtual nav_msgs::Odometry  convetEstimationToOdometry() = 0;

        virtual bool isOutlier(ColumnVector measurement) = 0;
        virtual double mahalanobisDistance(ColumnVector measurement) = 0;
};



#endif
