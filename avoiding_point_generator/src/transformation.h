#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <complex>
#include <cmath>

#include <geometry_msgs/Point.h>

typedef std::complex<double> Complex;

namespace Tool{
    constexpr double e = std::numeric_limits<double>::epsilon();

    double normalize(double angle){
        while(angle > M_PI){
            angle -= 2.0*M_PI;
        }
        while(angle < -M_PI){
            angle += 2.0*M_PI;
        }
        return angle;
    }

    double toDegree(double angle) { return angle * 180.0 * M_1_PI;}
    double toRadian(double angle) { return angle * 0.00555555 * M_PI;}
    double getAngle(const Complex &z) { return normalize(std::arg(z));}
    double getNorm(const Complex &z) { return std::norm(z);}
    double getSize(const Complex &z) { return std::abs(z);}
    Complex toUnit(double angle) { return std::polar(1.0, angle);}
    Complex toUnit(const Complex &z) { return z / getSize(z);}
    Complex getConjugate(const Complex &z) { return std::conj(z);}
    Complex toComplex(const geometry_msgs::Point &p) { return Complex(p.x, p.y);}
    bool isEqual(const double value1, const double value2){
        if(std::abs(value1 - value2) <= e){
            return true;
        }else{
            return false;
        }
    }
}

class Transformation{
    public:
        Transformation()
            : mAngle(0.0), mRotate(1.0, 0.0){}

        Transformation(const Transformation &trans){
            mAngle = Tool::normalize(trans.mAngle);
            mRotate = trans.mRotate;
            mCenter = trans.mCenter;
        }

        Transformation(const Complex &center, const double &theta){
            mAngle = Tool::normalize(theta);
            mRotate = Tool::toUnit(theta);
            mCenter = center;
        }

        void set(const Complex &center, const double &theta){
            mAngle = Tool::normalize(theta);
            mRotate = Tool::toUnit(theta);
            mCenter = center;
        }

        Complex transform(const Complex &z) const {
            return (z - mCenter) * Tool::getConjugate(mRotate);
        }

        Complex invertedTransform(const Complex &z) const {
            return z * mRotate + mCenter;
        }

        double transform(double angle) const {
            return Tool::normalize(angle - mAngle);
        }

        double invertedTransform(double angle) const {
            return Tool::normalize(angle + mAngle);
        }


    private:
        double mAngle;
        Complex mRotate;
        Complex mCenter;

};

#endif //TRANSFORMATION_H
