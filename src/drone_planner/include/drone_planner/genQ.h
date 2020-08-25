#ifndef GENQ_H
#define GENQ_H

#include <Eigen/Core>

class GenQ{
	
	const int _n_order;				// Order of the polynomial
	double _planning_horizon;		// Planning horizon in seconds
	Eigen::MatrixXd _power_vector;  // Power vector of the polynomial
	Eigen::MatrixXd _power_matrix;	// Power matrix of the basis for the polynomial square
	Eigen::MatrixXd _Q_block_zeros; // Zero matrix of the block size
	Eigen::MatrixXd _time_matrix;   // Time matrix to account for the planning horizon

public:
	GenQ(int order,double horizon);

	Eigen::MatrixXd power_matrix_deriv(int deriv_order);
	Eigen::MatrixXd genQ(const Eigen::MatrixXd &power_matrix, const Eigen::MatrixXd &deriv_matrix);
	void gen_all_Q();

	Eigen::MatrixXd _Q;				// Q matrix
	Eigen::MatrixXd _Q_derv_1;		// Q matrix for the square of the first derivative polynomial
	Eigen::MatrixXd _Q_derv_2;		// Q matrix for the square of the second derivative polynomial
	Eigen::MatrixXd _Q_derv_3;		// Q matrix for the square of the third derivative polynomial
};

#endif