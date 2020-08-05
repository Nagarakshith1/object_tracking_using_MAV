#ifndef K_FILTER_H
#define K_FILTER_H

#include <iostream>
#include <Eigen/Dense>

const int n_states{9};
const int n_meas{3}; 

typedef Eigen::Matrix< double, n_states, n_states> sq_matrix;
typedef Eigen::Matrix< double, n_states, 1> state;
typedef Eigen::Matrix< double,  n_meas, n_meas> measurement_noise;

class KalmanFilter{

public:

    KalmanFilter();

    KalmanFilter(const state &x_init, const sq_matrix &q_init,const measurement_noise &r_init, const sq_matrix &p_init) : _x{x_init}, _Q{q_init}, _R{r_init}, _P{p_init} 
    {
        _H.setzero();
        _H(0,0) = 1;
        _H(1,1) = 1;
        _H(2,2) = 1;
    }

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