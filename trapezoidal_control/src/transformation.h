#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <complex>
#include <cmath>

typedef std::complex<double> Complex;

namespace Tool{
    inline double normalize(double angle){
        while(angle > M_PI){
            angle -= 2.0*M_PI;
        }
        while(angle < -M_PI){
            angle += 2.0*M_PI;
        }
        return angle;
    }

    inline double toDegree(double angle) { return angle * 180.0 * M_1_PI;}
    inline double toRadian(double angle) { return angle * 0.00555555 * M_PI;}
    inline double getAngle(const Complex &z) { return normalize(std::arg(z));}
    inline double getNorm(const Complex &z) { return std::norm(z);}
    inline double getSize(const Complex &z) { return std::abs(z);}
    inline Complex toUnit(double angle) { return std::polar(1.0, angle);}
    inline Complex toUnit(const Complex &z) { return z / getSize(z);}
    inline Complex getConjugate(const Complex &z) { return std::conj(z);}
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
