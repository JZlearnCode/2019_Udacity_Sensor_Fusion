#include "ukf.h"

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

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 5.0;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.8;

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

    // Laser measurement noise standard deviation position1 in m
    std_las_px_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_las_py_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_rad_r_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_rad_phi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_rad_rd_ = 0.3;

    /**
     * End DO NOT MODIFY section for measurement noise values
     */

    /**
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */
    // Initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // State dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Number of sigma points
    n_sigma_ = 2 * n_aug_ + 1;

    // Sigma point spreading parameter
    lambda_ = 3 - n_x_;

    // initial state vector
    x_ = VectorXd::Zero(n_x_);

    // initial covariance matrix
    P_ = MatrixXd::Zero(n_x_, n_x_);

    // predicted sigma points matrix
    Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

    // Weights of sigma points
    weights_ = VectorXd::Zero(2 * n_aug_ + 1);

    // if difference in time between two measurements 
    // is too large, subdivide the prediction step is needed for stability
    dt_threshold_ = 0.1; 
    default_dt_ = 0.05; 

    n_z_radar_ = 3;
    n_z_lidar_ = 2; 

    R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
    R_radar_ <<  std_rad_r_*std_rad_r_, 0, 0,
                0, std_rad_phi_*std_rad_phi_, 0,
                0, 0,std_rad_rd_*std_rad_rd_;

    R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
    R_lidar_ <<  std_las_px_ * std_las_px_, 0,
                0, std_las_py_ * std_las_py_;      

}

void UKF::Initialize(const MeasurementPackage& meas_package)  {
  double px = 0;
  double py = 0; 
  // initialize with first measurement 
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // radar measurement in polar coordinates [range, bearing,radial velocity]
    // convert to state in cartesian coordinate [px py v yaw yawd]
    double radar_range = meas_package.raw_measurements_(0);
    double radar_bearing = meas_package.raw_measurements_(1);
    px = radar_range * cos(radar_bearing);
    py = radar_range * sin(radar_bearing);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    px = meas_package.raw_measurements_(0);
    py = meas_package.raw_measurements_(1);
  }
  x_.fill(0.0);
  x_[0] = px;
  x_[1] = py; 

  previous_timestamp_ =  meas_package.timestamp_;
  is_initialized_ = true; 
  return;
}
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) {
        return;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) {
        return;
    }

    if (!is_initialized_) {
        Initialize(meas_package);
        return; 
    }

    long dt = (meas_package.timestamp_ - previous_timestamp_) * 1e-6;
    previous_timestamp_ = meas_package.timestamp_;

    // use smaller timestamp to improve stability
    while (dt > dt_threshold_) {
      Prediction(default_dt_);
      dt = dt - default_dt_; 
    }
    Prediction(dt);
    PredictSensorMeasurement(meas_package); 

    UpdateState(meas_package.raw_measurements_);
}

void UKF::InitializeMeasurement(MeasurementPackage meas_package) {  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    //set measurement dimension, radar can measure r, phi, and r_dot
    n_z_ = n_z_radar_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    //set measurement dimension, laser can measure px, py
    n_z_ = n_z_lidar_;
  }

  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd::Zero(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  z_pred_ = VectorXd::Zero(n_z_);

  //measurement covariance matrix S
  S_ = MatrixXd::Zero(n_z_, n_z_);
}

void UKF::PredictSensorMeasurement(MeasurementPackage meas_package) {
  InitializeMeasurement(meas_package);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    PredictRadarMeasurement();
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    PredictLidarMeasurement();
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
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    S_ = S_ + R_radar_;
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    S_ = S_ + R_lidar_;
  }
}

void UKF::PredictLidarMeasurement() {
  // transform sigma points into measurement space
  for (int i = 0; i < n_sigma_; ++i) {  // 2n+1 simga points
    // measurement model
    Zsig_(0,i) = Xsig_pred_(0,i); //px  
    Zsig_(1,i) = Xsig_pred_(1,i); //py
  }
}

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
}

void UKF::Prediction(long delta_t) {
  /**
   * Estimate the object's location. 
   * Modify the state vector, x_. 
   * Predict sigma points, the state, and the state covariance matrix.
   */
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, n_sigma_);
  GenerateAugmentedSigmaPoints(Xsig_aug);
  SigmaPointPrediction(Xsig_aug, delta_t);
  PredictMeanAndCovariance();
}


/*
* Step 1: Generate augmented sigma points 
* output : Xsig_aug  (n_aug_, 2 * n_aug_ + 1)
MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_);
*/
void UKF::GenerateAugmentedSigmaPoints(MatrixXd& Xsig_aug) {


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

  
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
}


void UKF::SigmaPointPrediction(MatrixXd& Xsig_aug, double dt) {
    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
        }
        else {
            px_p = p_x + v*dt*cos(yaw);
            py_p = p_y + v*dt*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*dt;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
        py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
        v_p = v_p + nu_a*dt;

        yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
        yawd_p = yawd_p + nu_yawdd*dt;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
}


void UKF::PredictMeanAndCovariance() {
    // set weights and these weights will be shared during update
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (lambda_ + n_aug_);
        weights_(i) = weight;
    }

    //predicted state mean
    x_.fill(0.0);
    //iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);
    //iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        while (x_diff(3)> M_PI) {
            x_diff(3) -= 2. * M_PI;
        }
        while (x_diff(3)<-M_PI) {
            x_diff(3) += 2. * M_PI;
        }

        P_ += weights_(i) * x_diff * x_diff.transpose() ;
    }
}

void UKF::UpdateState(const VectorXd& z) {
    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sigma_; ++i) {  // 2n+1 simga points
      // residual
      VectorXd z_diff = Zsig_.col(i) - z_pred_;
      // angle normalization  [0, 360] --> [-180, 180]
      // radar state [range,   bearing,  radial velocity]
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      // state difference 
      // Xsig_pred_ = [px_p py_p v_p yaw_p yawd_p]
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      // angle normalization  [0, 360] --> [-180, 180]
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    // [n_x X n_z] = [n_x X n_z]  *  [n_z X n_z]  
    MatrixXd K = Tc * S_.inverse();

    // residual
    // [n_z]  =  [n_z]   -  [n_z]
    VectorXd z_diff = z - z_pred_;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // update state mean and covariance matrix
    // [n_x] = [n_x] + [n_x X n_z] * [n_z] 
    x_ = x_ + K * z_diff;
    // [n_x X n_x] = [n_x X n_x] - [n_x X n_z] * [n_z X n_z]* [n_z X n_x]
    P_ = P_ - K*S_*K.transpose();
}