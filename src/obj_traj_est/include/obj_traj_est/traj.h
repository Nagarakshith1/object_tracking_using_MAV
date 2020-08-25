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

int _n_order;							// Order of the polynomial
int _n_dimensions;						// Dimensions of the trajectory
float _planning_horizon;				// Planning horizon in seconds
Eigen::MatrixXd _coefficients;			// Coefficients of the polynomials
Eigen::MatrixXd _deriv_1_coeff;			// Coefficients of the first derivative polynomial
Eigen::MatrixXd _deriv_2_coeff;			// Coefficients of the second derivative polynomial
Eigen::MatrixXd _deriv_3_coeff;			// Coefficients of the third derivative polynomial
Eigen::VectorXd _power_vec;				// Powers of the polynomial
Eigen::VectorXd _power_vec_deriv_1;		// Powers of the first derivative polynomial
Eigen::VectorXd _power_vec_deriv_2;		// Powers of the second derivative polynomial
Eigen::VectorXd _power_vec_deriv_3;		// Powers of the third derivative polynomial



public:
	Traj();
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