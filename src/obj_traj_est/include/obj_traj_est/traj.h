#ifndef TRAJ_H
#define TRAJ_H

#include <Eigen/Core>
#include <iostream>
#include <ros/ros.h>

#include  "obj_traj_est/traj_msg"

class Traj{

int _n_order;
int _n_dimensions;
float _planning_horizon;
Eigen::Matrixd<double> _coefficients;

public:
	Traj(const Eigen::Matrixd<double> &c, float planning_horizon);
	obj_traj_est::traj_msg to_rosmsg();

}



#endif