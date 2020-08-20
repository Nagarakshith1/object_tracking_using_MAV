#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include "obj_traj_est/traj.h"
#include "drone_planner/genQ.h"
#include <nlopt.hpp>

int planning_rate;
double planning_horizon;
const int n_order = 7;
const int n_dim = 3;
const int n_coeff = n_order+1;
const int n_sol = n_dim*(n_coeff-2);

std::vector<double> fixed_c(6);

double lambda_0 = 0;
double lambda_1 = 0.3;
double lambda_3 = 0;
std::vector<double> sol(n_sol,0);
const double gravity  = 9.81;
const int n_vis_normals = 4;
Eigen::Matrix<double,3,n_vis_normals> normals;
Traj drone_tr(Eigen::Matrix<double,n_coeff,n_dim>::Zero(),1);

Eigen::Matrix<double, n_dim*(n_coeff),n_dim*(n_coeff)> Q_pos;
Eigen::Matrix<double, n_dim*(n_coeff),n_dim*(n_coeff)> Q_vel;
Eigen::Matrix<double, n_dim*(n_coeff),n_dim*(n_coeff)> Q_acc;
Eigen::Matrix<double, n_dim*(n_coeff),n_dim*(n_coeff)> Q_jerk;

Eigen::Matrix<double,n_dim*(n_coeff),1> h;
struct DroneState{
	double x_pos;
	double y_pos;
	double z_pos;
	double x_vel;
	double y_vel;
	double z_vel;
}drone_state;

ros::Publisher drone_vel_pub;
ros::Publisher drone_vis_pub;

geometry_msgs::Twist twist_cmd;
visualization_msgs::Marker vis_msg;
// void genQ(){

// 	// Eigen::Matrix<double,n_order+1,1> power_vector;
// 	auto power_vector = Eigen::VectorXd::LinSpaced(n_order+1,n_order,0);
// 	auto power_matrix = power_vector.replicate(1,n_order+1)+power_vector.transpose().replicate(n_order+1,1);

// 	auto power_matrix_deriv_1 = power_matrix - 2*1*(Eigen::MatrixXd::Ones(n_order+1,n_order+1));
// 	auto power_matrix_deriv_2 = power_matrix - 2*2*(Eigen::MatrixXd::Ones(n_order+1,n_order+1));
// 	auto power_matrix_deriv_3 = power_matrix - 2*3*(Eigen::MatrixXd::Ones(n_order+1,n_order+1));

// 	Eigen::Matrix<double,n_order+1,n_order+1> deriv = Eigen::Matrix<double,n_order+1,n_order+1>::Zero();
// 	deriv.bottomLeftCorner(n_order,n_order) = power_vector.topLeftCorner(n_order,1).asDiagonal();

// 	auto deriv_1 = deriv;
// 	auto deriv_2 = deriv * deriv_1;
// 	auto deriv_3 = deriv * deriv_2;
// 	// auto deriv_4 = deriv * deriv_3;

// 	auto power_matrix_int = power_matrix + Eigen::MatrixXd::Ones(n_order+1,n_order+1);
// 	auto Q_block = Eigen::MatrixXd::Ones(n_order+1,n_order+1).array()/power_matrix_int.array();
	
// 	auto coeff_deriv_1 = deriv_1.colwise().sum().transpose()*deriv_1.colwise().sum();
// 	auto temp_int1 = power_matrix_deriv_1 + Eigen::MatrixXd::Ones(n_order+1,n_order+1);
// 	auto power_matrix_deriv_1_int = (power_matrix_deriv_1.array()>=0).select(temp_int1,power_matrix_deriv_1);
// 	auto Q_block_derv_1 = coeff_deriv_1.array()/power_matrix_deriv_1_int.array().abs();

// 	auto coeff_deriv_2 = deriv_2.colwise().sum().transpose()*deriv_2.colwise().sum();
// 	auto temp_int2 = power_matrix_deriv_2 + Eigen::MatrixXd::Ones(n_order+1,n_order+1);
// 	auto power_matrix_deriv_2_int = (power_matrix_deriv_2.array()>=0).select(temp_int2,power_matrix_deriv_2);
// 	auto Q_block_derv_2 = coeff_deriv_2.array()/power_matrix_deriv_2_int.array().abs();	

// 	auto coeff_deriv_3 = deriv_3.colwise().sum().transpose()*deriv_3.colwise().sum();
// 	auto temp_int3 = power_matrix_deriv_3 + Eigen::MatrixXd::Ones(n_order+1,n_order+1);
// 	auto power_matrix_deriv_3_int = (power_matrix_deriv_3.array()>=0).select(temp_int3,power_matrix_deriv_3);
// 	auto Q_block_derv_3 = coeff_deriv_3.array()/power_matrix_deriv_3_int.array().abs();

// 	auto Q_block_zeros = Eigen::MatrixXd::Zero(n_order+1,n_order+1);

// 	auto time_matrix = planning_horizon*Eigen::MatrixXd::Ones(n_order+1,n_order+1);
	
// 	auto time_power_matrix = Eigen::pow(time_matrix.array(),power_matrix_int.array());
// 	auto temp = Q_block*time_power_matrix;
// 	Q <<temp,Q_block_zeros,Q_block_zeros,
// 		Q_block_zeros,temp,Q_block_zeros,
// 		Q_block_zeros,Q_block_zeros,temp;

// 	auto time_power_matrix1 = Eigen::pow(time_matrix.array(),power_matrix_deriv_1_int.array());
// 	auto temp1 = Q_block_derv_1*time_power_matrix;
// 	Q_derv_1 <<temp,Q_block_zeros,Q_block_zeros,
// 			   Q_block_zeros,temp,Q_block_zeros,
// 			   Q_block_zeros,Q_block_zeros,temp;

// 	auto time_power_matrix2 = Eigen::pow(time_matrix.array(),power_matrix_deriv_2_int.array());
// 	auto temp2 = Q_block_derv_2*time_power_matrix;
// 	Q_derv_2 <<temp,Q_block_zeros,Q_block_zeros,
// 			   Q_block_zeros,temp,Q_block_zeros,
// 			   Q_block_zeros,Q_block_zeros,temp;

// 	auto time_power_matrix3 = Eigen::pow(time_matrix.array(),power_matrix_deriv_3_int.array());
// 	auto temp3 = Q_block_derv_3*time_power_matrix;
// 	Q_derv_3 <<temp,Q_block_zeros,Q_block_zeros,
// 			   Q_block_zeros,temp,Q_block_zeros,
// 			   Q_block_zeros,Q_block_zeros,temp;


// 	// ROS_INFO_STREAM("\n"<<Q_block_derv_1);
// 	ROS_INFO_STREAM("\n"<<Q_block_derv_2);
// 	// ROS_INFO_STREAM("\n"<<Q_block_derv_3);
// }
void drone_odom_callback(const nav_msgs::Odometry &msg){
	
	drone_state.x_pos = msg.pose.pose.position.x;
	drone_state.y_pos = msg.pose.pose.position.y;
	// need to change later
	drone_state.z_pos = 3; 
	drone_state.x_vel = msg.twist.twist.linear.x;
	drone_state.y_vel = msg.twist.twist.linear.y;
	// need to change later
	drone_state.z_vel = 0;
	
}

Eigen::Matrix<double,n_dim*n_coeff,1> genC(const std::vector<double> &x){
	Eigen::Matrix<double,n_dim*n_coeff,1> c;
	c(n_order) = fixed_c[0];
	c(n_order-1) = fixed_c[1];
	c((n_dim-1)*n_order+1) = fixed_c[2];
	c((n_dim-1)*n_order) = fixed_c[3];
	c(n_dim*n_coeff-1) = fixed_c[4];
	c(n_dim*n_coeff-2) = fixed_c[5];
	int k=0;
	for(int i=0; i<n_dim;i++){
		for(int j=0;j<n_order-1;j++){
			c(n_coeff*i+j) = x[k++];
		}
	}
	return c;
}

Eigen::Matrix<double,n_sol*n_dim,1> genX(const Eigen::MatrixXd &c){
	Eigen::Matrix<double,n_sol*n_dim,1> x;
	int k=0;
	for(int i=0; i<n_dim;i++){
		for(int j=0;j<n_order-1;j++){
			x[k++] = c(n_coeff*i+j);
		}
	}
	return x;

}
Eigen::Matrix<double,n_dim*n_coeff,n_dim> genB(double t,int order){
	
	auto time_power_vec = drone_tr.gen_basis(t,order);
	auto zero_vec = Eigen::VectorXd::Zero(n_coeff);
	Eigen::Matrix<double,n_dim*n_coeff,n_dim> B;
	B<< time_power_vec,       zero_vec, 	  zero_vec,
		      zero_vec, time_power_vec, 	  zero_vec,
		      zero_vec,       zero_vec, time_power_vec;
	return B;
}

double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data){
	auto c = genC(x);

	auto h_copy = h;
	// Add an offest of the desired height
	h_copy(n_dim*n_coeff-1) = 3;
	
	auto Q_net = lambda_0*Q_pos + lambda_1*Q_vel + lambda_3*Q_jerk;
	
	auto     f = -2 * Q_net.transpose() * h_copy;
    auto alpha =  h_copy.transpose() * Q_net * h_copy;
  	auto     J =  c.transpose() * Q_net * c + f.transpose()*c + alpha;

  if(!grad.empty()){
  	auto grad_eig = genX(2 * c.transpose() * Q_net + f.transpose());
  	for(int i = 0;i < grad.size();i++){
  		grad[i]=grad_eig[i];
  	}
  }
  
  return J(0,0);

}
void vision_constraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
	
	int  iter = m/n_vis_normals;
	
	auto c = genC(std::vector<double>(x,x+n_sol));
	
	for(int i=0;i<iter;i++){
		double t = (i+1.0)/iter*planning_horizon;
		

		Eigen::Vector3d b = genB(t,0).transpose()*(h-c);
		

		auto e3 = Eigen::Vector3d::UnitZ();
		double angle = std::acos(e3.transpose() * (- b / b.norm()));
    	Eigen::Vector3d axis = e3.cross(-b);
    	axis.normalize();
    	Eigen::Matrix3d R;
    	R = Eigen::AngleAxisd(angle,axis).toRotationMatrix();
    	Eigen::Matrix<double, 3, n_vis_normals> rotated_normals = R * (-normals);
    	Eigen::Vector3d vertex = -gravity * e3;
    	
    	Eigen::Matrix<double, 1, n_vis_normals> ineq = (genB(t,2).transpose() * c - vertex).transpose() * rotated_normals;
    	

    	
    	for(int j=0;j<n_vis_normals;j++){
    		result[i*n_vis_normals+j] = ineq[j];
    	}
	    if(grad){

	    	Eigen::Matrix<double, n_vis_normals, n_coeff *n_dim> grad_c = rotated_normals.transpose() * genB(t,2).transpose();
	    	Eigen::Matrix<double, n_vis_normals, n_sol *n_dim> grad_x;
			for(int j=0;j<n_vis_normals;j++){
				grad_x.row(j) = genX(grad_c.row(j));
			}
			for(int l=0;l<n_vis_normals;l++){
				int m = i*n_vis_normals + l;
				for(int j=0;j<n_sol;j++){
					grad[m*n_sol+j] = grad_x(l,j);
				}
			}


	    }


	}
}
Eigen::Matrix<double,n_coeff,n_dim> sol_vec_to_coeff(std::vector<double> &sol_vec){
	auto c = genC(sol_vec);
	Eigen::Matrix<double,n_coeff,1> x_coeff(c.data());
	Eigen::Matrix<double,n_coeff,1> y_coeff(c.data()+n_coeff);
	Eigen::Matrix<double,n_coeff,1> z_coeff(c.data()+2*n_coeff);
	Eigen::Matrix<double,n_coeff,n_dim> drone_coeff;
	drone_coeff << x_coeff,y_coeff,z_coeff;
	// ROS_INFO_STREAM("\n"<<drone_coeff);
	return drone_coeff;
}

void planner_callback(const obj_traj_est::traj_msg &msg){
	
	Eigen::Matrix<double,n_coeff,1> x(msg.coeff_x.data());
	Eigen::Matrix<double,n_coeff,1> y(msg.coeff_y.data());
	Eigen::Matrix<double,n_coeff,1> z(msg.coeff_z.data());
	h << x,y,z;
	

	fixed_c[0] = drone_state.x_pos;
	fixed_c[1] = drone_state.x_vel;
	fixed_c[2] = drone_state.y_pos;
	fixed_c[3] = drone_state.y_vel;
	fixed_c[4] = drone_state.z_pos;
	fixed_c[5] = drone_state.z_vel;

	std::vector<double> sol_vec (n_sol,0);
	double objective_value;
	nlopt::result result;
	nlopt::opt opt(nlopt::LD_SLSQP, sol_vec.size());
	opt.set_min_objective(objective_function, NULL);
	opt.set_maxtime(1.0/planning_rate - 0.002);

	// Set vision constraints
	std::vector<double> vis_tol(10*n_vis_normals,1e-2);
	opt.add_inequality_mconstraint(vision_constraints, NULL, vis_tol);
	result = opt.optimize(sol_vec,objective_value);
	auto drone_coeff = sol_vec_to_coeff(sol_vec);
	drone_tr = Traj(drone_coeff,planning_horizon);

	vis_msg = drone_tr.to_vismsg();
	drone_vis_pub.publish(vis_msg);

	double begin = ros::Time::now().toSec();
	double curr = ros::Time::now().toSec();

	while(curr-begin<planning_horizon-2){
		curr = ros::Time::now().toSec();
		Eigen::VectorXd t(1);
		t(0,0) = curr-begin;
		auto vel = drone_tr.evaluate(t,1);
		twist_cmd.linear.x = vel(0);
		twist_cmd.linear.y = vel(1);
		twist_cmd.linear.z = vel(2);

		twist_cmd.angular.x = 0;
		twist_cmd.angular.y = 0;
		twist_cmd.angular.z = 0;
		drone_vel_pub.publish(twist_cmd);
	}

}


int main(int argc, char **argv)
{	
	ros::init(argc,argv,"drone_planner");
	ros::NodeHandle n;
	ros::Subscriber drone_odom_sub = n.subscribe("/drone/odom",1,&drone_odom_callback);
	ros::Subscriber obj_traj_sub = n.subscribe("/obj_traj_coeff",1,&planner_callback);
	drone_vel_pub = n.advertise<geometry_msgs::Twist>("/drone/cmd_vel",1);
	drone_vis_pub = n.advertise<visualization_msgs::Marker>("/drone_traj_vis",1);
	

	planning_horizon = 3;

	

	drone_tr.set_planning_horizon(planning_horizon);
	GenQ q(n_order,planning_horizon);

	Q_pos = q._Q;
	Q_vel = q._Q_derv_1;
	Q_acc = q._Q_derv_2;
	Q_jerk = q._Q_derv_3;


	double x_fov = M_PI/2;
	double y_fov = M_PI/2;

	double x_max = tan(x_fov/2);
	double y_max = tan(y_fov/2);

	Eigen::Matrix<double,3,n_vis_normals> rays;
	rays << x_max, -x_max, -x_max,  x_max,
			y_max,  y_max, -y_max, -y_max,
			   1,     1,     1,     1;
	Eigen::Vector3d l1 = rays.col(n_vis_normals-1);
	Eigen::Vector3d l2;

	// Construct all the normals
	for(int i=0; i<n_vis_normals;i++){
		l2 = rays.col(i);
		auto n = l1.cross(l2);
		n.normalize();
		normals.col(i) = n;
		l1 = l2;
	}

	planning_rate = 20;
	ros::Rate replan_rate(planning_rate);
	while(ros::ok()){

		ros::spinOnce();
		replan_rate.sleep();
	}

}
