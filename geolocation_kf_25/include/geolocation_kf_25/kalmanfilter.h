#ifndef KALMAN_FILTER
#define KALMAN_FILTER
#include <Eigen/Dense>
#include <iostream>

struct reserved_argument
{
    Eigen::Matrix<double, 3, 1> T1;
    Eigen::MatrixXd T2;
    Eigen::MatrixXd T3;
    Eigen::MatrixXd T4;

    int iarg[20];
    double darg[20];
    unsigned char carg[20];

};
typedef struct reserved_argument arg_array;

class kalmanfilter
{

public:

    kalmanfilter(Eigen::MatrixXd A, Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd P0, Eigen::MatrixXd x0, Eigen::MatrixXd B);
    kalmanfilter(Eigen::MatrixXd A, Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd P0, Eigen::MatrixXd x0);
    ~kalmanfilter();
    void update(Eigen::MatrixXd z);
    void update(Eigen::MatrixXd z, Eigen::Matrix<double, 3, 1> u, double dt);
    
public:

// System Model
    Eigen::MatrixXd A;
	Eigen::MatrixXd H;
	Eigen::MatrixXd Q;
	Eigen::MatrixXd R;
    Eigen::MatrixXd B;

// State
	Eigen::MatrixXd P;
	Eigen::MatrixXd x;
	Eigen::MatrixXd xp;
	Eigen::MatrixXd Pp;
	Eigen::MatrixXd K;

};

double myAbs(double value);


#endif