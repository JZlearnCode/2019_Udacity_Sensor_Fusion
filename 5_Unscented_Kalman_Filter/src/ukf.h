#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /*
  * Initialize 
  */
  void Initialize(const MeasurementPackage& meas_package);
  
  void GenerateAugmentedSigmaPoints(Eigen::MatrixXd& Xsig_aug);
  
  void SigmaPointPrediction(long delta_t, const Eigen::MatrixXd& Xsig_aug);

  void PredictMeanAndCovariance();

  void PredictRadarMeasurement();

  void PredictLidarMeasurement();

  void PredictSensorMeasurement(MeasurementPackage meas_package); 

  void UpdateState(const Eigen::VectorXd& z);
  
  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(long delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // if difference in time between two measurements 
  // is too large, subdivide the prediction step is needed for stability
  double dt_threshold_; 
  double default_dt_; 

  // Laser measurement noise standard deviation position1 in m
  double std_las_px_;

  // Laser measurement noise standard deviation position2 in m
  double std_las_py_;

  // Radar measurement noise standard deviation radius in m
  double std_rad_r_;

  // Radar measurement noise standard deviation angle in rad
  double std_rad_phi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_rad_rd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimensionf
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Number of sigma points
  int n_sigma_; 

  // Sigma point spreading parameter
  double lambda_;

  // A value representing small value near zero 
  double near_zero_value_;

  // set measurement dimension
  int n_z_;
  //radar can measure r, phi, and r_dot
  int n_z_radar_ = 3;
  //lidar measures position x,y 
  int n_z_lidar_ = 2; 

  // create matrix for sigma points in measurement space
  Eigen::MatrixXd Zsig_;
  // mean predicted measurement
  Eigen::VectorXd z_pred_;
  // measurement covariacne matrix S
  Eigen::MatrixXd S_; 
  // Radar measurement noise covariance matrix 
  Eigen::MatrixXd R_radar_;
  // Lidar measurement noise covariance matrix 
  Eigen::MatrixXd R_lidar_;

  //Time for previous step 
  long previous_timestamp_;
};

#endif  // UKF_H