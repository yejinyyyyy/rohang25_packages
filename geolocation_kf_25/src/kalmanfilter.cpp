#include <kalmanfilter.h>


kalmanfilter::kalmanfilter(Eigen::MatrixXd A, Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd P0, Eigen::MatrixXd x0, Eigen::MatrixXd B)
{

    this->A = A;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->B = B;

    this->P = P0;
    this->x = x0;

}

kalmanfilter::kalmanfilter(Eigen::MatrixXd A, Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd P0, Eigen::MatrixXd x0)
{

    this->A = A;
    this->H = H;
    this->Q = Q;
    this->R = R;

    this->P = P0;
    this->x = x0;

}

kalmanfilter::~kalmanfilter()
{

}

void kalmanfilter::update(Eigen::MatrixXd z)
{
    Eigen::MatrixXd temp;

    xp = A * x;
    Pp = A * P * (A.transpose()) + Q;
	
	//printf("z_out        :: %lf %lf %d\n", z(0), 0.1, z(0) < 0.1);
	
	if(myAbs(z(0)) < 0.0001)  // skip update if measurement near zero
	{
		this->x = xp;
		this->P = Pp;
		return;
	}

	//printf("z_out        :: %lf\n", z(0));
	
	temp = this->H * this->Pp * (this->H.transpose()) + this->R;
	this->K = this->Pp * (this->H.transpose()) * (temp.inverse());

	this->x = this->xp + this->K * (z - this->H * this->xp);
	this->P = this->Pp - this->K * this->H * this->Pp;
	
}

/*
void kalmanfilter22222222::update(Eigen::MatrixXd z)
{
    Eigen::MatrixXd temp;

    xp = A * x + 1;
    Pp = A * P * (A.transpose()) + Q;
	
	//printf("z_out        :: %lf %lf %d\n", z(0), 0.1, z(0) < 0.1);
	
	if(myAbs(z(0)) < 0.0001)
	{
		this->x = xp;
		this->P = Pp;
		return;
	}//

	//printf("z_out        :: %lf\n", z(0));
	
	temp = this->H1 * this->Pp * (this->H.transpose()) + this->R;
	this->K = this->Pp * (this->H.transpose()) * (temp.inverse());

	this->x = this->xp + this->K * (z - this->H * this->xp);
	this->P = this->Pp - this->K * this->H * this->Pp;
	
}
*/

double myAbs(double value)
{

	if(value < 0)
	{
		return -value;
	}

	return value;
}

void kalmanfilter::update(Eigen::MatrixXd z, Eigen::Matrix<double, 3, 1> u, double dt)
{

    Eigen::MatrixXd temp;

    xp = A * x + B*u;	// State Prediction
    Pp = A * P * (A.transpose()) + Q*dt*dt;	// Error covariance prediction
	
	if(myAbs(z(0)) < 0.00001)
	{
		this->x = xp;
		this->P = Pp;
		return;
	}
	
	temp = this->H * this->Pp * (this->H.transpose()) + this->R;
	this->K = this->Pp * (this->H.transpose()) * (temp.inverse());  // Kalman gain

	this->x = this->xp + this->K * (z - this->H * this->xp); // State update
	this->P = this->Pp - this->K * this->H * this->Pp; 		 // Error covariance update

	return;
}
