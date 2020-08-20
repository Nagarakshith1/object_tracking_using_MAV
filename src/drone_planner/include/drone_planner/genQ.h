#ifndef GENQ_H
#define GENQ_H

#include <Eigen/Core>

class GenQ{
	
	const int _n_order;
	double _planning_horizon;
	Eigen::MatrixXd _power_vector;
	Eigen::MatrixXd _power_matrix;
	Eigen::MatrixXd _Q_block_zeros;
	Eigen::MatrixXd _time_matrix;

public:
	GenQ(int order,double horizon);

	Eigen::MatrixXd power_matrix_deriv(int deriv_order);
	Eigen::MatrixXd genQ(const Eigen::MatrixXd &power_matrix, const Eigen::MatrixXd &deriv_matrix);
	void gen_all_Q();

	Eigen::MatrixXd _Q;
	Eigen::MatrixXd _Q_derv_1;
	Eigen::MatrixXd _Q_derv_2;
	Eigen::MatrixXd _Q_derv_3;

};



#endif