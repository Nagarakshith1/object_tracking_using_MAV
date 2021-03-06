#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include "obj_traj_est/traj.h"
#include "drone_planner/genQ.h"
#include <nlopt.hpp>

int planning_rate;					// The rate of drone trjectory planning
double planning_horizon;			// The planning horizon in seconds
int n_order;						// The order of the drone trajectory
int n_dim;							// The number of dimensions for the trajectory planning
int n_coeff;						// Number of coefficients in the polynomial
int n_sol; 							// Number of coefficients of the polynomial for the solver
double drone_height;				// The minimum height at which the drone has to fly.

double weights;						// Weight of the robot
double lambda_0;					// Position weighting factor in the cost function
double lambda_1;					// Velocity weighting factor in the cost function
double lambda_3;					// Jerk weighting factor in the cost function
double confidence_constant;

const double gravity  = 9.81;
double max_accel_coeff;
double max_accel;
double max_angacc;
const int n_vis_normals = 4; 		// The number of normals for the visual constraints
const int n_moment_constrains = 4;  // The number of moment constrains
std::vector<double> fixed_c(6);		// The vector to store the known coefficients 
const double thresh = 0.05;
Traj drone_tr; 

Eigen::MatrixXd normals; 			// Normals of the planes of the pyramid formed by the camera
Eigen::MatrixXd Q_pos; 				// Coefficient matrix for position 
Eigen::MatrixXd Q_vel; 				// Coefficient matrix for velocity 
Eigen::MatrixXd Q_acc; 				// Coefficient matrix for acceleration 
Eigen::MatrixXd Q_jerk; 			// Coefficient matrix for jerk 
Eigen::MatrixXd h; 					// Target trjectory coefficient matrix

struct DroneState {
	double x_pos;
	double y_pos;
	double z_pos;
	double x_vel;
	double y_vel;
	double z_vel;
} drone_state;						 // Store the drone state based on the odometry msg

ros::Publisher drone_traj_pub;		 // Drone Trajectory publisher
ros::Publisher drone_vis_pub;		 // Drone trajectory visualization publisher

obj_traj_est::traj_msg tr_msg_drone; // Msg for traj of drone
visualization_msgs::Marker vis_msg;  // Msg for the drone trajectory visualization

/**
	Stores the drone state based on the odometry
	@param msg Drone odometry message
*/
void drone_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
	
	drone_state.x_pos = msg->pose.pose.position.x;
	drone_state.y_pos = msg->pose.pose.position.y;


	drone_state.z_pos = msg->pose.pose.position.z; 

	drone_state.x_vel = msg->twist.twist.linear.x;
	drone_state.y_vel = msg->twist.twist.linear.y;
	

	drone_state.z_vel = msg->twist.twist.linear.z;
}


/**
	Generate the full coeffecient vector
	@param x The coefficient vector given to the solver
	@return The full coeffecient vector
*/
Eigen::MatrixXd genC(const std::vector<double> &x) {
	Eigen::MatrixXd c = Eigen::MatrixXd::Zero(n_dim * n_coeff,1);
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

/**
	Generate the coefficient vector for the solver
	@param c The full coefficient vector
	@return The coefficient vector for the solver
*/
Eigen::MatrixXd genX(const Eigen::MatrixXd &c) {
	Eigen::MatrixXd x = Eigen::MatrixXd::Zero(n_sol * n_dim,1);
	int k = 0;
	for(int i = 0; i < n_dim; i++){
		for(int j = 0; j < n_order - 1; j++){
			x(k++) = c(n_coeff * i + j);
		}
	}

	return x;
}

/**
	Generate the Basis matrix of any order
	@param t The time instance
	@param order The order of the derivative of the polynomial
*/ 
Eigen::MatrixXd genB(double t, int order) {
	auto time_power_vec = drone_tr.gen_basis(t, order);   
	auto zero_vec = Eigen::VectorXd::Zero(n_coeff);
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_dim * n_coeff,n_dim);
	B<< time_power_vec,       zero_vec, 	  zero_vec,
		      zero_vec, time_power_vec, 	  zero_vec,
		      zero_vec,       zero_vec, time_power_vec;
	return B;
}


/**
	The objective function to be minimized
	@params Refer the nlopt docs
*/
double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
	auto c = genC(x);
	auto h_copy = h;

	// Obtain the position relative vector between the drone and the target
	Eigen::Vector3d b = genB(0, 0).transpose()*(h - c);
	// Obtain the velocity relative vector between the drone and the target
	Eigen::Vector3d b_dot = genB(0, 1).transpose()*(h - c);

	if(b_dot.dot(b) <= 0 || b_dot.norm() < thresh){
		lambda_0 = 0.5;
	}
	else lambda_0 = 0.3;
	
	// Calculate the axis and the angle about which the normals have to be rotated
	auto e3 = Eigen::Vector3d::UnitZ();
	double angle = std::fabs(std::acos(e3.transpose() * (- b / b.norm())));

	// Add an offest of the desired height
	h_copy(n_dim*n_coeff - 1) = drone_height + confidence_constant * angle;
	
	auto Q_net = lambda_0 * Q_pos + lambda_1 * Q_vel + lambda_3 * Q_jerk;
	
	auto     f = -2 * Q_net.transpose() * h_copy;
    auto alpha =  h_copy.transpose() * Q_net * h_copy;
  	auto     J =  c.transpose() * Q_net * c + f.transpose() * c + alpha;

  if (!grad.empty()) {
  	auto grad_eig = genX(2 * c.transpose() * Q_net + f.transpose());
  	for(int i = 0; i < grad.size(); i++) {
  		grad[i] = grad_eig(i);
  	}
  }
  return J(0,0);
}


void thrust_constraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {

	int iter = m;
	auto c = genC(std::vector<double>(x,x + n_sol));

	Eigen::Vector3d g = -gravity * Eigen::Vector3d::UnitZ();
	double rhs = max_accel * max_accel - gravity * gravity;

	for(int i = 0; i < iter; i++) {
		// Create the time instances to add the constraints at
		double t = (i + 1.0) / iter * planning_horizon;
		auto B = genB(t,2);
		double equation = (c.transpose()*(B * B.transpose()) * c + 2 * g.transpose() * B.transpose() * c)(0,0) - rhs;
		result[i] = equation;

		if(grad) {
			auto grad_eig = genX(2*(c.transpose() * (B * B.transpose()) + g.transpose() * B.transpose()));
			for(int k = 0; k < n_sol ; k++) {
  				grad[i * n_sol + k] = grad_eig(k);
  			}
		}

	}
}


void angular_moment_constraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {

	int iter = m/n_moment_constrains;
	auto c = genC(std::vector<double>(x,x + n_sol));
	Eigen::Vector3d g = gravity * Eigen::Vector3d::UnitZ();
	Eigen::Vector3d bc;
	bc << 1,0,0;

	for(int i = 0; i < iter; i++) {
		// Create the time instances to add the constraints at
		double t = (i + 1.0) / iter * planning_horizon;

		Eigen::MatrixXd B_2 = genB(t,2);
		Eigen::MatrixXd B_3 = genB(t,3);
		Eigen::MatrixXd B_4 = genB(t,4);

		Eigen::MatrixXd x_2t = B_2.transpose()*c;
		Eigen::MatrixXd x_3t = B_3.transpose()*c;
		Eigen::MatrixXd x_4t = B_4.transpose()*c;

		Eigen::Vector3d x_2 = Eigen::Map<Eigen::Vector3d>(x_2t.data(),n_dim);
		Eigen::Vector3d x_3 = Eigen::Map<Eigen::Vector3d>(x_3t.data(),n_dim);
		Eigen::Vector3d x_4 = Eigen::Map<Eigen::Vector3d>(x_4t.data(),n_dim);

		Eigen::Vector3d b3 = (x_2 + g).normalized();
		Eigen::Vector3d b2 = b3.cross(bc).normalized();
		Eigen::Vector3d b1 = b2.cross(b3).normalized();

		double f = m*((x_2 + g).norm());
		double f_dot = m*(b3.dot(x_3));

		double omega_1 = -(m/f)*b2.dot(x_3);
		double omega_2 = (m/f)*b1.dot(x_3);
		double omega_3 = -b2(2)*omega_2/b3(2);

		auto omegadot_1 = -(m/f)*b2.dot(x_4) - 2*(f_dot/f)*omega_1 + omega_3*omega_2;
		auto omegadot_2 = (m/f)*b1.dot(x_4) - 2*(f_dot/f)*omega_2 - omega_3*omega_1;
		
		result[i*n_moment_constrains + 0] = omegadot_1 - max_angacc;
		result[i*n_moment_constrains + 1] = -omegadot_1 - max_angacc;
		result[i*n_moment_constrains + 2] = omegadot_2 - max_angacc;
		result[i*n_moment_constrains + 3] = -omegadot_2 - max_angacc; 


		if(grad) {

			Eigen::MatrixXd grad_omega1 = -(m/f) * b2.transpose() * B_3.transpose();

			Eigen::MatrixXd grad_omega2 = (m/f) * b1.transpose() * B_3.transpose();
			
			Eigen::MatrixXd grad_omegadot1 = -(m/f) * (b2.transpose() *  B_4.transpose()) - 2*(f_dot/f)*grad_omega1;
			
			Eigen::MatrixXd grad_omegadot2 = (m/f) * (b1.transpose() * B_4.transpose()) - 2*(f_dot/f)*grad_omega2; 
		
			auto grad_omegadot1_opt = genX(grad_omegadot1);
			auto grad_omegadot2_opt = genX(grad_omegadot2);

			for(int j = 0; j < n_sol; j++){

				grad[ i*(n_sol*n_moment_constrains) + j ] = grad_omegadot1_opt(j);
				grad[ i*(n_sol*n_moment_constrains) + (j + n_sol) ] = -grad_omegadot1_opt(j);
				grad[ i*(n_sol*n_moment_constrains) + (j + 2*n_sol) ] = grad_omegadot2_opt(j);
				grad[ i*(n_sol*n_moment_constrains) + (j + 3*n_sol) ] = -grad_omegadot2_opt(j);

			}
		}

	}
}


/**
	The vision constraints to be added
	@params Refer the nlopt docs
*/ 
void vision_constraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {	
	int  iter = m / n_vis_normals;
	auto c = genC(std::vector<double>(x,x + n_sol));
	
	for(int i = 0; i < iter; i++){

		// Create the time instances to add the constraints at
		double t = (i + 1.0) / iter * planning_horizon;
		
		// Obtain the position relative vector between the drone and the target
		Eigen::Vector3d b = genB(t, 0).transpose()*(h - c);
		
		// Calculate the axis and the angle about which the normals have to be rotated
		auto e3 = Eigen::Vector3d::UnitZ();
		double angle = std::acos(e3.transpose() * (- b / b.norm()));

    	Eigen::Vector3d axis = e3.cross(-b);
    	axis.normalize();

    	Eigen::Matrix3d R;
    	R = Eigen::AngleAxisd(angle,axis).toRotationMatrix();

    	Eigen::MatrixXd rotated_normals = R * (-normals);
    	Eigen::Vector3d vertex = -gravity * e3;
    	
    	// Add the inequality constraint on the thrust vector and the normals
    	Eigen::MatrixXd ineq = (genB(t, 2).transpose() * c - vertex).transpose() * rotated_normals;
    	
    	for(int j = 0; j < n_vis_normals; j++) {

    		result[i * n_vis_normals + j] = ineq(j);
    	}

	    if(grad) {	

	    	Eigen::MatrixXd grad_c = rotated_normals.transpose() * genB(t, 2).transpose();
	    	Eigen::MatrixXd grad_x = Eigen::MatrixXd::Zero(n_vis_normals,n_sol * n_dim);
	    	
			for(int j = 0; j < n_vis_normals; j++){

				grad_x.row(j) = genX(grad_c.row(j)).transpose();

			}
			
			for(int l = 0; l < n_vis_normals; l++){

				int m = i * n_vis_normals + l;
				for(int j = 0; j < n_sol; j++){
					grad[m * n_sol + j] = grad_x(l,j);
				}

			}
	    }
	}
}

/**
	Convert the solution vector to the coefficient matrix
	@param sol_vec The solution vector obtained from the solver
	@return The drone trajectory coefficient matrix
*/ 
Eigen::MatrixXd sol_vec_to_coeff(std::vector<double> &sol_vec) {
	auto c = genC(sol_vec);
	Eigen::Map<Eigen::VectorXd> x_coeff(c.data(),n_coeff);
	Eigen::Map<Eigen::VectorXd> y_coeff(c.data() + n_coeff,n_coeff);
	Eigen::Map<Eigen::VectorXd> z_coeff(c.data() + 2 * n_coeff,n_coeff);
	Eigen::MatrixXd drone_coeff = Eigen::MatrixXd::Zero(n_coeff,n_dim);
	drone_coeff << x_coeff,y_coeff,z_coeff;
	return drone_coeff;
}

/**
	Perform the optimization and publish the drone trajectory coefficients
	@param msg Target trajectory coefficient message
*/ 
void planner_callback(const obj_traj_est::traj_msg::ConstPtr &msg) {
	// Get the target trajectory coefficient vector
	Eigen::Map<Eigen::VectorXd> x(const_cast<double*>(msg->coeff_x.data()),msg->coeff_x.size());
	Eigen::Map<Eigen::VectorXd> y(const_cast<double*>(msg->coeff_y.data()),msg->coeff_y.size());
	Eigen::Map<Eigen::VectorXd> z(const_cast<double*>(msg->coeff_z.data()),msg->coeff_z.size());
	h << x,y,z;
	
	// Known coefficients
	fixed_c[0] = drone_state.x_pos;
	fixed_c[1] = drone_state.x_vel;
	fixed_c[2] = drone_state.y_pos;
	fixed_c[3] = drone_state.y_vel;
	fixed_c[4] = drone_state.z_pos;
	fixed_c[5] = drone_state.z_vel;

	// Solution vector for the optimizer
	std::vector<double> sol_vec (n_sol,0);

	// The value of the objective function
	double objective_value;
	nlopt::result result;
	nlopt::opt opt(nlopt::LD_SLSQP, sol_vec.size());
	opt.set_min_objective(objective_function, NULL);
	opt.set_maxtime(1.0 / planning_rate - 0.002);

	// Set vision constraints
	std::vector<double> vis_tol(10 * n_vis_normals,1e-2);
	opt.add_inequality_mconstraint(vision_constraints, NULL, vis_tol);
	
	// Set thrust constraints
	std::vector<double> thrust_tol(10, 1e-2);
	opt.add_inequality_mconstraint(thrust_constraints, NULL, thrust_tol);

	// Set angular constraints
	std::vector<double> angular_tol( 10*n_moment_constrains, 1e-2 );
	opt.add_inequality_mconstraint(angular_moment_constraints, NULL, thrust_tol);

	try{

		// Peform the optimization
		result = opt.optimize(sol_vec, objective_value);

		auto drone_coeff = sol_vec_to_coeff(sol_vec);
		drone_tr = Traj(drone_coeff, planning_horizon);

		tr_msg_drone = drone_tr.to_rosmsg();
		drone_traj_pub.publish(tr_msg_drone);

		// Publish the visualization message
		vis_msg = drone_tr.to_vismsg();
		drone_vis_pub.publish(vis_msg);
	} catch(...){
		ROS_WARN_STREAM("NLOPT ERROR Received");
	}
}


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "drone_planner");
	ros::NodeHandle n("~");
	
	ros::Subscriber drone_odom_sub = n.subscribe("odom", 1, &drone_odom_callback);
	ros::Subscriber obj_traj_sub = n.subscribe("obj_traj_coeff", 1, &planner_callback);
	
	drone_traj_pub = n.advertise<obj_traj_est::traj_msg>("drone_traj_coeff", 1);
	drone_vis_pub = n.advertise<visualization_msgs::Marker>("drone_traj_vis", 1);
	
	// The field of view of the camera
	double x_fov;
	double y_fov;

	n.getParam("n_order", n_order);
	n.getParam("weights", weights);
	n.getParam("n_dim", n_dim);
	n.getParam("planning_horizon", planning_horizon);
	n.getParam("planning_rate", planning_rate);
	n.getParam("drone_height", drone_height);
	n.getParam("lambda_0", lambda_0);
	n.getParam("lambda_1", lambda_1);
	n.getParam("lambda_3", lambda_3);
	n.getParam("max_accel_coeff", max_accel_coeff);
	n.getParam("max_angacc", max_angacc);
	n.getParam("confidence_constant", confidence_constant);

	n.param("x_fov", x_fov, M_PI / 2);
	n.param("y_fov", y_fov, M_PI  / 2);

	n_coeff = n_order + 1;
    n_sol = n_dim * (n_coeff - 2);
	max_accel = max_accel_coeff*gravity; 

	h = Eigen::MatrixXd::Zero(n_dim * n_coeff, 1);
	drone_tr = Traj(Eigen::MatrixXd::Zero(n_coeff,n_dim), planning_horizon);
	normals = Eigen::MatrixXd::Zero(3,n_vis_normals);

	GenQ q(n_order,planning_horizon);

	Q_pos = q._Q;
	Q_vel = q._Q_derv_1;
	Q_acc = q._Q_derv_2;
	Q_jerk = q._Q_derv_3;

	double x_max = tan(x_fov/2);
	double y_max = tan(y_fov/2);

	Eigen::Matrix<double,3,n_vis_normals> rays;
	rays << x_max, -x_max, -x_max,  x_max,
			y_max,  y_max, -y_max, -y_max,
			   1,     1,     1,     1;
	
	Eigen::Vector3d l1 = rays.col(n_vis_normals-1);
	Eigen::Vector3d l2;

	// Construct all the normals
	for(int i = 0; i < n_vis_normals; i++){
		l2 = rays.col(i);
		auto n = l1.cross(l2);
		n.normalize();
		normals.col(i) = n;
		l1 = l2;
	}

	ros::Rate replan_rate(planning_rate);
	while(ros::ok()){
		ros::spinOnce();
		replan_rate.sleep();
	}
	return 0;
}
