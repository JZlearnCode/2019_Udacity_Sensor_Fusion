#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  is_initialized_ = false;

  // State related variables 
  n_x_ = 5;
  n_aug_ = 7;
  n_sigma_ = 2 * n_aug_ + 1;
  lambda_ = 3 - n_x_;
  
  // initial state vector
  x_ = VectorXd::Zero(n_x_);

  Xsig_pred_ = MatrixXd::Zero(n_x_, n_sigma_);

  // initial covariance matrix
  P_ = MatrixXd::Zero(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

  // Define values close to zero 
  near_zero_value_ = 0.001;

  // /**
  //  * measurement noise values provided by the sensor manufacturer.
  //  */

  // // Laser measurement noise standard deviation position1 in m
  // std_laspx_ = 0.15;

  // // Laser measurement noise standard deviation position2 in m
  // std_laspy_ = 0.15;

  // Radar related values
  // Radar measurement noise standard deviation radius in m
  std_rad_r_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_rad_phi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_rad_rd_ = 0.3;

  // set weights
  weights_ = VectorXd::Zero(n_sigma_);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<n_sigma_; ++i) {   
    weights_(i) = 0.5/(lambda_+n_aug_);
  }
}

UKF::~UKF() {}

/*
* Step 1: Generate augmented sigma points 
* output : Xsig_aug  (n_aug_, 2 * n_aug_ + 1)
MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_);
*/
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_aug) {


  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create augmented mean state (first n_x elements)
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  
  Xsig_aug->col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug->col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug->col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
}

/*
* Example from Udacity course material

delta_t time diff in sec
*/
void UKF::SigmaPointPrediction(const double delta_t, const MatrixXd& Xsig_aug) {
  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero 
    if (fabs(yawd) > near_zero_value_) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
 
}

/*
* Example from Udacity course material
*/
void UKF::PredictMeanAndCovariance() {
  // predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sigma_; ++i) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  // predicted state covariance matrix
  for (int i = 0; i < n_sigma_; ++i) {  // iterate over sigma points
    // state difference
    // Xsig_pred_ = [px_p py_p v_p yaw_p yawd_p]
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization  [0, 360] --> [-180, 180]
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/*
Example from Udacity course material 
*/
void UKF::PredictRadarMeasurement() {
  // transform sigma points into measurement space
  for (int i = 0; i < n_sigma_; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v_x = cos(yaw)*v;
    double v_y = sin(yaw)*v;

    // measurement model
    Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig_(1,i) = atan2(p_y,p_x);                                // phi
    Zsig_(2,i) = (p_x*v_x + p_y*v_y) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred_.fill(0.0);
  for (int i=0; i < n_sigma_; ++i) {
    z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }

  // predicted measurement covariance 
  S_.fill(0.0);
  for (int i = 0; i < n_sigma_; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    // angle normalization  [0, 360] --> [-180, 180]
    // radar state [range,   bearing,  radial velocity]
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  S_ = S_ + R_radar_;
}

// /*
// Example from Udacity course material 
// */
// void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out) {

//   // set state dimension
//   int n_x = 5;

//   // set augmented dimension
//   int n_aug = 7;

//   // set measurement dimension, radar can measure r, phi, and r_dot
//   int n_z = 3;

//   // define spreading parameter
//   double lambda = 3 - n_aug;

//   // set vector for weights
//   VectorXd weights = VectorXd(2*n_aug+1);
//   double weight_0 = lambda/(lambda+n_aug);
//   double weight = 0.5/(lambda+n_aug);
//   weights(0) = weight_0;

//   for (int i=1; i<2*n_aug+1; ++i) {  
//     weights(i) = weight;
//   }

//   // create example matrix with predicted sigma points in state space
//   MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
//   Xsig_pred <<
//      5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
//        1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
//       2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
//      0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
//       0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

//   // create example vector for predicted state mean
//   VectorXd x = VectorXd(n_x);
//   x <<
//      5.93637,
//      1.49035,
//      2.20528,
//     0.536853,
//     0.353577;

//   // create example matrix for predicted state covariance
//   MatrixXd P = MatrixXd(n_x,n_x);
//   P <<
//     0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
//     -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
//     0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
//    -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
//    -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

//   // create example matrix with sigma points in measurement space
//   MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
//   Zsig <<
//     6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
//    0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
//     2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

//   // create example vector for mean predicted measurement
//   VectorXd z_pred = VectorXd(n_z);
//   z_pred <<
//       6.12155,
//      0.245993,
//       2.10313;

//   // create example matrix for predicted measurement covariance
//   MatrixXd S = MatrixXd(n_z,n_z);
//   S <<
//       0.0946171, -0.000139448,   0.00407016,
//    -0.000139448,  0.000617548, -0.000770652,
//      0.00407016, -0.000770652,    0.0180917;

//   // create example vector for incoming radar measurement
//   VectorXd z = VectorXd(n_z);
//   z <<
//      5.9214,   // rho in m
//      0.2187,   // phi in rad
//      2.0062;   // rho_dot in m/s

//   // create matrix for cross correlation Tc
//   MatrixXd Tc = MatrixXd(n_x, n_z);

//   // calculate cross correlation matrix
//   Tc.fill(0.0);
//   for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
//     // residual
//     VectorXd z_diff = Zsig.col(i) - z_pred;
//     // angle normalization
//     while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//     while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

//     // state difference
//     VectorXd x_diff = Xsig_pred.col(i) - x;
//     // angle normalization
//     while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
//     while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

//     Tc = Tc + weights(i) * x_diff * z_diff.transpose();
//   }

//   // Kalman gain K;
//   MatrixXd K = Tc * S.inverse();

//   // residual
//   VectorXd z_diff = z - z_pred;

//   // angle normalization
//   while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//   while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

//   // update state mean and covariance matrix
//   x = x + K * z_diff;
//   P = P - K*S*K.transpose();

//   // write result
//   *x_out = x;
//   *P_out = P;
// }

// void UKF::Initialize(const MeasurementPackage& meas_package) {
// }

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  // set measurement dimension
  // radar can measure r, phi, and r_dot
  // lidar can measure px, py 
  //if (meas_package.)
  //// add measurement noise covariance matrix
  // R_radar_ = MatrixXd(n_z_, n_z_);
  // R_radar_ <<  std_rad_r_*std_rad_r_, 0, 0,
  //              0, std_rad_phi_*std_rad_phi_, 0,
  //              0, 0,std_rad_rd_*std_rad_rd_;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}