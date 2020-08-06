#ifndef K_FILTER_H
#define K_FILTER_H

#include <iostream>
#include <ros/ros.h>
//#include<pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

const int n_states{9};
const int n_meas{3}; 

typedef Eigen::Matrix< double, n_states, n_states> sq_matrix;
typedef Eigen::Matrix< double, n_states, 1> state;
typedef Eigen::Matrix< double,  n_meas, n_meas> measurement_noise;

class KalmanFilter{

public:

    KalmanFilter();

    KalmanFilter(const state &x_init, const sq_matrix &q_init,const measurement_noise &r_init, const sq_matrix &p_init);

    void Initialize(const state &x_init,const sq_matrix &q_init, const measurement_noise &r_init,const sq_matrix &p_init);

    void ProcessUpdate(double dt);

    state MeasurementUpdate(const Eigen::Matrix<double,n_meas,1> &meas);


private:

    sq_matrix _A,_Q,_P;
    measurement_noise _R;
    state _x;
    Eigen::Matrix<double,n_meas,n_states> _H;

};

#endif 