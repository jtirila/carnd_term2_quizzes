#include <iostream>
#include "ukf.h"

UKF::UKF() {
  //TODO Auto-generated constructor stub
  Init();
}

UKF::~UKF() {
  //TODO Auto-generated destructor stub
}

void UKF::Init() {

}


/*******************************************************************************
* Programming assignment functions:
*******************************************************************************/

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  //define spreading parameter
  double spreading_c = std::sqrt(3);

  //set example state
  VectorXd x = VectorXd(n_x);

  x <<   5.7441,
      1.3800,
      2.2049,
      0.5015,
      0.3528;

  //create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
      0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;


  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);


  //create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  x_aug.head(n_x) = x;

  Xsig_aug.block(0, 0, n_x, n_x);

  P_aug.block(0, 0, n_x, n_x) = P;
  P_aug.block(0, n_x, n_x, 2) = MatrixXd::Zero(n_x, 2);
  P_aug.block(n_x, 0, 2, n_x) = MatrixXd::Zero(2, n_x);
  P_aug(n_x, n_x) = std_a * std_a;
  P_aug(n_x - 1, n_x) = 0;
  P_aug(n_x, n_x - 1) = 0;
  P_aug(n_x + 1, n_x + 1) = std_yawdd * std_yawdd;


  MatrixXd A = P_aug.llt().matrixL();

  MatrixXd x_aug_repl = x_aug.replicate(1, n_aug);

  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0, 1, n_aug, n_aug) = x_aug_repl + spreading_c * A;
  Xsig_aug.block(0, n_aug + 1, n_aug, n_aug) = x_aug_repl - spreading_c * A;


/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;

/* expected result:
   Xsig_aug =
  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
    1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
       0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
       0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
*/

}