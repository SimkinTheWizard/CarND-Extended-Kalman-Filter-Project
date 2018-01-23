#include "kalman_filter.h"
#include <iostream>

#define PI 3.14

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

Eigen::VectorXd KalmanFilter::GetX()
{
	return x_;
}

Eigen::MatrixXd KalmanFilter::GetP()
{
	return P_;
}

void KalmanFilter::SetH(Eigen::MatrixXd H)
{
	H_ = H;
}

void KalmanFilter::SetR(Eigen::MatrixXd R)
{
	R_ = R;
}

void KalmanFilter::UpdateF(float dt)
{
	/*
	Eigen::MatrixXd F (4,4);
	F << 1, 0, dt, 0,
	0, 1, 0,  dt,
	0, 0, 1,  0,
	0, 0, 0,  1;
	 F_ = F;
	 */
	F_ << 1, 0, dt, 0,
	      0, 1, 0,  dt,
		  0, 0, 1,  0,
	      0, 0, 0,  1;
}

void KalmanFilter::UpdateQ(float dt, float noise_ax, float noise_ay)
{

	float dt4 = (dt*dt*dt*dt)/4;
	float dt3 = (dt*dt*dt)/2;
	float dt2 = (dt*dt);
	/*
	Eigen::MatrixXd Q (4,4);
	Q << dt4*noise_ax, 0, dt3*noise_ax, 0,
	0, dt4*noise_ay, 0, dt3*noise_ay,
	dt3*noise_ax, 0, dt2*noise_ax, 0,
	0, dt3*noise_ay, 0 ,dt2*noise_ay;*/
	Q_ << dt4*noise_ax, 0, dt3*noise_ax, 0,
	0, dt4*noise_ay, 0, dt3*noise_ay,
	dt3*noise_ax, 0, dt2*noise_ax, 0,
	0, dt3*noise_ay, 0 ,dt2*noise_ay;

}

void KalmanFilter::Init(const VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const VectorXd hx) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	//VectorXd z_pred = hx;
	VectorXd y = z - hx;

	float theta = y(1);
	while (theta>PI)
		theta -= 2*PI;
	while (theta<-PI)
		theta += 2*PI;
	y(1) = theta;
	
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
