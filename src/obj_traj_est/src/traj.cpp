#include <obj_traj_est/traj.h>

Traj::Traj(const Eigen::Matrixd<double> &c, float planning_horizon){
	_coefficients = c;
	_planning_horizon = planning_horizon;
	_n_order = c.rows()-1;
	_n_dimensions = c.cols();
}
obj_traj_est::traj_msg Traj::to_rosmsg(){
	
	obj_traj_est::traj_msg msg;

	for(int i=0;i<=_n_order;i++){
		msg.coeff_x.push_back(_coefficients(i,0));
		msg.coeff_y.push_back(_coefficients(i,1));
		msg.coeff_z.push_back(_coefficients(i,2));
	}
	msg.header.stamp = ros::TimeNow();
	return msg

}