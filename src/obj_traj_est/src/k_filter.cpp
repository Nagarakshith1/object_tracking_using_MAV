#include <obj_traj_est/k_filter.h>

KalmanFilter::KalmanFilter(){
}

KalmanFilter::KalmanFilter(const state &x_init, const sq_matrix &q_init,const measurement_noise &r_init, const sq_matrix &p_init){
    _x = x_init;
    _Q = q_init;
    _R = r_init;
    _P = p_init;
    _H.setZero();
    _H(0,0) = 1;
    _H(1,1) = 1;
    _H(2,2) = 1;
}

void KalmanFilter::Initialize(const state &x_init, const sq_matrix &q_init,const measurement_noise &r_init, const sq_matrix &p_init){
    _x = x_init;
    _Q = q_init;
    _R = r_init;
    _P = p_init;
}

void KalmanFilter::ProcessUpdate(double dt){
    _A << 
    1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
    0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, dt, 0, 0,
    0, 0, 0, 0, 1, 0, 0, dt, 0,
    0, 0, 0 ,0 ,0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0 ,0 ,0, 0, 0, 1, 0, 
    0, 0, 0 ,0 ,0, 0, 0, 0, 0;

    _x = _A*_x;
    _P = _A*_P*_A.transpose() + _Q;

}

state KalmanFilter::MeasurementUpdate(const Eigen::Matrix<double,n_meas,1> &meas){

    auto k_gain = _P*_H.transpose()*(_H*_P*_H.transpose() + _R.transpose()).inverse();
    _x = _x + k_gain*(meas -_H*_x); 
    _P = (sq_matrix::Identity() - k_gain*_H)*_P;
    return _x;
}