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

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
           5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
             1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
           2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
           0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
           0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
                0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
                0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; //time diff in sec
/*******************************************************************************
 * Student part begin
 ******************************************************************************/


  // Augmented state vector:
  // [px, py, v, phi, phidot, va, vphidotdot]
  VectorXd x_aug;
  VectorXd acc_vec(5);
  acc_vec.fill(0.0);
  VectorXd noise_vec(5);

  double px, py, v, phi, phidot, va, phidotdot;

  for(int column = 0; column < Xsig_aug.cols(); column++) {
    px = Xsig_aug(0, column);
    py = Xsig_aug(1, column);
    v = Xsig_aug(2, column);
    phi = Xsig_aug(3, column);
    phidot = Xsig_aug(4, column);
    va = Xsig_aug(5, column);
    phidotdot = Xsig_aug(6, column);


    if(std::abs(phidot) < 0.00001){
      acc_vec(0) = 0.0;
      acc_vec(1) = 0.0;
    } else {
      acc_vec(0) = v / phidot * (std::sin(phi + phidot * delta_t) - std::sin(phi));
      acc_vec(1) = v / phidot * (-std::cos(phi + phidot * delta_t) + std::cos(phi));
    }
    // acc_vec(2) no change
    acc_vec(3) = phidot * delta_t;
    // acc_vec(4) no change


    double half_delta_t_sqr = delta_t * delta_t / 2;

    noise_vec(0) = half_delta_t_sqr * std::cos(phi) * va;
    noise_vec(1) = half_delta_t_sqr * std::sin(phi) * va;
    noise_vec(2) = delta_t * va;
    noise_vec(3) = half_delta_t_sqr * phidotdot;
    noise_vec(4) = delta_t * phidotdot;


    Xsig_pred.col(column) = (Eigen::VectorXd(5) << px, py, v, phi, phidot).finished() + acc_vec + noise_vec;
  }



/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;

}
