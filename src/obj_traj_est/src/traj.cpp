#include <obj_traj_est/traj.h>

Traj::Traj(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &c, float planning_horizon){
	_coefficients = c;
	_planning_horizon = planning_horizon;
	_n_order = c.rows()-1;
	_n_dimensions = c.cols();
}

void Traj::set_coefficients(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &c){
	_coefficients = c;
}

Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> Traj::get_coefficients(){
	return _coefficients;
}

Eigen::MatrixXd Traj::evaluate(Eigen::VectorXd times){
	Eigen::MatrixXd basis;
	int order = 7;
	basis.conservativeResize(times.rows(),_n_order+1);
	for(int i=_n_order;i>0;--i){
		basis.col(_n_order-i) << times.array().pow(i).matrix();
	}
	basis.col(_n_order).setOnes();
	return basis*_coefficients;
}	

obj_traj_est::traj_msg Traj::to_rosmsg(){
	
	obj_traj_est::traj_msg msg;

	for(int i=0;i<=_n_order;i++){
		msg.coeff_x.push_back(_coefficients(i,0));
		msg.coeff_y.push_back(_coefficients(i,1));
		msg.coeff_z.push_back(_coefficients(i,2));
	}
	// msg.header.stamp = ros::TimeNow();
	return msg;

}


visualization_msgs::Marker Traj::to_vismsg(){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "vis_obj_traj";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	double dt = 0.1;
	
	auto path = Traj::evaluate(Eigen::VectorXd::LinSpaced(int(_planning_horizon/dt),0.0,_planning_horizon).transpose());

	for (int i=0; i<path.rows();i++){
		geometry_msgs::Point p;

		p.x = path(i,0);
		p.y = path(i,1);
		p.z = path(i,2);

		marker.points.push_back(p);
	}
	return marker;

}
