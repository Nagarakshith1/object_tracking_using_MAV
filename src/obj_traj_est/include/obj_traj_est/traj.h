#ifndef TRAJ_H
#define TRAJ_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include  "obj_traj_est/traj_msg.h"

class Traj{

int _n_order;
int _n_dimensions;
float _planning_horizon;
Eigen::MatrixXd _coefficients;
Eigen::MatrixXd _deriv_1_coeff;
Eigen::MatrixXd _deriv_2_coeff;
Eigen::MatrixXd _deriv_3_coeff;
Eigen::VectorXd _power_vec;
Eigen::VectorXd _power_vec_deriv_1;
Eigen::VectorXd _power_vec_deriv_2;
Eigen::VectorXd _power_vec_deriv_3;



public:
	Traj(const Eigen::MatrixXd &c, float planning_horizon);
	void set_coefficients(const Eigen::MatrixXd &c);
	void set_planning_horizon(double time);
	Eigen::MatrixXd get_coefficients();
	obj_traj_est::traj_msg to_rosmsg();
	Eigen::MatrixXd evaluate(const Eigen::VectorXd &times,int order);
	Eigen::VectorXd gen_basis(double time, int order);
	visualization_msgs::Marker to_vismsg();

};



#endif