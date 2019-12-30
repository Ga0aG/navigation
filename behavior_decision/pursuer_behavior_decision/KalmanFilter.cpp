#include "pursuer_behavior_decision/KalmanFilter.h"


void KalmanFilter::update(const geometry_msgs::Pose2D &position, Matrix<double,4,1> &estimatedState){
    Matrix<double,2,1> obs;
    Matrix<double,4,1> mu;
    MatrixXd Kt, delta,temp;
    obs << position.x, position.y;
    mu = transition_matrix*mean;
    delta = transition_matrix*variance*transition_matrix.transpose()+transition_covariance;
    temp = observation_matrix*delta*observation_matrix.transpose()+observation_covariance;
    Kt = delta*observation_matrix.transpose()*temp.inverse();
    mean = mu + Kt*(obs - observation_matrix*mu);
    variance = (Matrix4d::Identity() - Kt*observation_matrix)*delta;
    estimatedState = mean;
}
// void KalmanFilter::initialize(Matrix<double,4,1> initState,double deltat){
KalmanFilter::KalmanFilter(Matrix<double,4,1> initState,double deltat){
    double var_pos, var_vel, var_obs;
    var_pos = 0.1;
    var_vel = 0.3;
    var_obs = 0.1;
    mean = initState;
    transition_matrix << 1, 0, deltat, 0,
                         0, 1, 0, deltat,
                         0, 0, 1, 0,
                         0, 0, 0, 1;
    observation_matrix << 1, 0, 0, 0,
                          0, 1, 0, 0;
    transition_covariance << var_pos, 0, 0, 0,
                             0, var_pos, 0, 0,
                             0, 0, var_vel, 0,
                             0, 0, 0, var_vel;
    observation_covariance << var_obs, 0,
                              0, var_obs;
    variance << 0.2, 0, 0, 0,
                0, 0.2, 0, 0,
                0, 0, 1.0, 0,
                0, 0, 0, 1.0;
}

void KalmanFilter::resetDeltat(double deltat){
    transition_matrix(0,2) = deltat;
    transition_matrix(1,3) = deltat;
}