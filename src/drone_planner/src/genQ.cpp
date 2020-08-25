#include "drone_planner/genQ.h"

GenQ::GenQ(int order,double horizon):_n_order{order},_planning_horizon{horizon} {
	_power_vector = Eigen::VectorXd::LinSpaced(_n_order + 1,_n_order,0);
	_power_matrix = _power_vector.replicate(1,_n_order + 1) + _power_vector.transpose().replicate(_n_order + 1,1);
	_Q_block_zeros = Eigen::MatrixXd::Zero(_n_order + 1,_n_order + 1);
	_time_matrix = _planning_horizon*Eigen::MatrixXd::Ones(_n_order + 1,_n_order + 1);
	gen_all_Q();
}

/**
	Get the power matrix for the square of the polynomial basis
	@param deriv_order The order of the derivative of the polynomial
	@return The power matrix
*/
Eigen::MatrixXd GenQ::power_matrix_deriv(int deriv_order) {
	return _power_matrix - 2*(deriv_order)*(Eigen::MatrixXd::Ones(_n_order + 1,_n_order + 1));
}

/**
	Generate the Q_matrix 
	@param power_matrix The power matrix for the square of the polynomial basis of any derivative
	@param deriv_matrix The coefficient matrix for the polynomial basis of any derivative
	@return Q matrix
*/
Eigen::MatrixXd GenQ::genQ(const Eigen::MatrixXd &power_matrix, const Eigen::MatrixXd &deriv_matrix) {	
	// Calculate the coefficient matrix for the square of the polynomial basis of any derivative
	auto coeff_deriv = deriv_matrix.colwise().sum().transpose() * deriv_matrix.colwise().sum();

	// Get the power matrix for the integrated square of the Q_matrix
	auto temp_int = power_matrix + Eigen::MatrixXd::Ones(_n_order + 1,_n_order + 1);
	auto power_matrix_int = (power_matrix.array()>=0).select(temp_int,power_matrix);

	// Divide the derivtive coefficient matrix by the integrated power matrix
	auto temp = coeff_deriv.array() / power_matrix_int.array().abs();
	auto Q_block = temp * Eigen::pow(_time_matrix.array(),power_matrix_int.array());
	
	// Format for the all the dimensions
	Eigen::MatrixXd Q;
	Q.conservativeResize(3*(_n_order+1),3*(_n_order+1));
	Q <<Q_block,_Q_block_zeros,_Q_block_zeros,
		_Q_block_zeros,Q_block,_Q_block_zeros,
		_Q_block_zeros,_Q_block_zeros,Q_block;
	return Q;
}

/**
	Generate all the Q matrices
*/ 
void GenQ::gen_all_Q() {

	// Generate Q
	_Q = genQ(_power_matrix,Eigen::MatrixXd::Ones(1,_n_order+1));

	// Generate Q_1
	Eigen::MatrixXd deriv_1;
	deriv_1 = Eigen::MatrixXd::Zero(_n_order+1,_n_order+1);
	deriv_1.bottomLeftCorner(_n_order,_n_order) = _power_vector.topLeftCorner(_n_order,1).asDiagonal();
	_Q_derv_1 = genQ(power_matrix_deriv(1),deriv_1);

	// Generate Q_2
	auto deriv_2 = deriv_1 * deriv_1;
	_Q_derv_2 = genQ(power_matrix_deriv(2),deriv_2);

	// Generate Q_3
	auto deriv_3 = deriv_1 * deriv_2;
	_Q_derv_3 = genQ(power_matrix_deriv(3),deriv_3);
	
}

	









