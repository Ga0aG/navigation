#ifndef EVADER_KALMANFILTER_H
#define EVADER_KALMANFILTER_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/Pose2D.h>
using namespace Eigen;

class KalmanFilter{
    private:
        Matrix4d transition_matrix;
        Matrix4d transition_covariance;
        Matrix2d observation_covariance;
        Matrix<double,2,4> observation_matrix;

    public:
        Matrix<double,4,1> mean;
        Matrix4d variance;
        KalmanFilter(Matrix<double,4,1> initState,double deltat);
        ~KalmanFilter();
        // void initialize(Matrix<double,4,1> initState,double deltat);
        void update(const geometry_msgs::Pose2D &position, Matrix<double,4,1> &estimatedState);
        void resetDeltat(double deltat);
};

#endif /*EVADER_KALMANFILTER_H*/
