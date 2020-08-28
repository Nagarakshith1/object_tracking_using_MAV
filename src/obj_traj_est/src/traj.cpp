#include <obj_traj_est/traj.h>
#include <ros/ros.h>

Traj::Traj () {}

Traj::Traj (const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &c, float planning_horizon) {
	_coefficients = c;
	_planning_horizon = planning_horizon;
	_n_order = c.rows() - 1;
	_n_dimensions = c.cols();

	_power_vec = Eigen::VectorXd::LinSpaced(_n_order + 1,_n_order,0);
	_power_vec_deriv_1.conservativeResize(_n_order + 1,1);
	_power_vec_deriv_1 << Eigen::VectorXd::LinSpaced(_n_order,_n_order - 1,0),0;
	
	_power_vec_deriv_2.conservativeResize(_n_order + 1,1);
	_power_vec_deriv_2 << Eigen::VectorXd::LinSpaced(_n_order - 1,_n_order - 2,0),0,0;
	
	_power_vec_deriv_3.conservativeResize(_n_order + 1,1);
	_power_vec_deriv_3 << Eigen::VectorXd::LinSpaced(_n_order - 2,_n_order - 3,0),0,0,0;

	_power_vec_deriv_4.conservativeResize(_n_order + 1,1);
	_power_vec_deriv_4 << Eigen::VectorXd::LinSpaced(_n_order - 3,_n_order - 4,0),0,0,0,0;

	Eigen::MatrixXd deriv_1 = Eigen::MatrixXd::Zero(_n_order + 1,_n_order + 1);
	deriv_1.bottomLeftCorner(_n_order,_n_order) = _power_vec.topLeftCorner(_n_order,1).asDiagonal();
	auto deriv_2 = deriv_1 * deriv_1;
	auto deriv_3 = deriv_1 * deriv_2;
	auto deriv_4 = deriv_1 * deriv_3;

	_deriv_1_coeff = deriv_1.colwise().sum().transpose();
	_deriv_2_coeff = deriv_2.colwise().sum().transpose();
	_deriv_3_coeff = deriv_3.colwise().sum().transpose();
	_deriv_4_coeff = deriv_4.colwise().sum().transpose(); 		
}
 
void Traj::set_coefficients(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &c) {
	_coefficients = c;
}
void Traj::set_planning_horizon(double time) {
	_planning_horizon = time;
}

Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> Traj::get_coefficients() {
	return _coefficients;
}

/**
	Evaluate the trajectory at time instances
	@param times The time instances
	@param order The derivative of the trajectory to be evaluated
	@return Evaluated values
*/
Eigen::MatrixXd Traj::evaluate(const Eigen::VectorXd &times, int order = 0) {
	Eigen::MatrixXd basis;	
	basis.conservativeResize(times.rows(),_n_order+1);
	for(int i = 0; i < times.size(); i++){
		basis.row(i) = gen_basis(times[i], order);
	}
	return basis * _coefficients;
}	

/**
	Generate the basis vector
	@param time The time instance 
	@param order The order of derivative for the basis
	@return Basis vector
*/
Eigen::VectorXd Traj::gen_basis(double time, int order) {
	auto time_vec = time * Eigen::VectorXd::Ones(_n_order + 1);
	switch(order){
		case 0: return Eigen::pow(time_vec.array(),_power_vec.array());
				break;
		case 1: return (_deriv_1_coeff.array() * Eigen::pow(time_vec.array(),_power_vec_deriv_1.array()));
				break;
		case 2: return(_deriv_2_coeff.array() * Eigen::pow(time_vec.array(),_power_vec_deriv_2.array()));
				break;
		case 3: return (_deriv_3_coeff.array() * Eigen::pow(time_vec.array(),_power_vec_deriv_3.array()));
				break;
		case 4: return (_deriv_4_coeff.array() * Eigen::pow(time_vec.array(),_power_vec_deriv_4.array()));
				break;
	}
}

/**
	Convert the coefficients to rosmsg
*/
obj_traj_est::traj_msg Traj::to_rosmsg() {
	obj_traj_est::traj_msg msg;
	for(int i = 0; i <= _n_order; i++){
		msg.coeff_x.push_back(_coefficients(i,0));
		msg.coeff_y.push_back(_coefficients(i,1));
		msg.coeff_z.push_back(_coefficients(i,2));
	}
	return msg;
}

/**
	Generate the Visualization msg for RViz
*/

visualization_msgs::Marker Traj::to_vismsg() {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "vis_obj_traj";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	marker.color.a = 1.0; 
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	// Time discetization for the markers
	double dt = 0.1;
	
	// Get the positions of target in the future
	auto path = Traj::evaluate(Eigen::VectorXd::LinSpaced(int(_planning_horizon/dt), 0.0, _planning_horizon).transpose());
	
	for (int i = 0; i < path.rows(); i++) {
		geometry_msgs::Point p;
		p.x = path(i,0);
		p.y = path(i,1);
		p.z = path(i,2);
		marker.points.push_back(p);
	}
	return marker;
}
