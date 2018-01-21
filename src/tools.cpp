#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd error(4);
	error << 0,0,0,0;
	if (estimations.size()== 0)
		return error;
	if (estimations.size() != ground_truth.size())
		return error;
	for (int i = 0; i < estimations.size(); i++)
	{
		VectorXd intermediate = estimations[i] - ground_truth[i];
		intermediate = intermediate.array() * intermediate.array();
		error += intermediate;
	}
	error = error /estimations.size();
	return error.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	
	float sum_squares = px*px + py*py ;
	float sqrt_squares = sqrt(sum_squares);
	float c = sum_squares*sqrt_squares;
	
	//check division by zero
	if (px*px + py*py < 0.00000001)
		return Hj; // error
	//compute the Jacobian matrix
	/*
	 Hj << px/(sqrt_squares),   py/(sqrt_squares),   0, 0,
		  -py/sum_squares,     px/sum_squares,      0, 0,
          py*(vx*py-vy*px)/(c),px*(vy*px-vx*py)/(c), px/(sqrt_squares), py/(sqrt_squares);
	 */
	Hj << px/(sqrt_squares), py/(sqrt_squares), 0, 0,
	-py/sum_squares, px/sum_squares, 0, 0,
	py*(vx*py-vy*px)/(sum_squares*sqrt_squares),px*(vy*px-vx*py)/(sum_squares*sqrt_squares) , px/(sqrt_squares), py/(sqrt_squares);
	

	

	
	return Hj;
	
}

VectorXd Tools::ToCartesian(const VectorXd &polar_measurements) {
	VectorXd cartesian(4);
	float rho = polar_measurements[0];
	float theta = polar_measurements[1];
	float rho_dot = polar_measurements[2];
	
	
	float px = rho * cos(theta);
	float py = rho * sin(theta);
	float vx = rho_dot * cos(theta);
	float vy = rho_dot * sin(theta);
	
	cartesian << px, py, vx, vy;
	return cartesian;
}

