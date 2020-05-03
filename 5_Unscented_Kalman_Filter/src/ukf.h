#pragma once

#include <iostream>
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
    virtual ~UKF() = default;
    
    //DONE: JIN     Initizlize
    void Initialize(const MeasurementPackage& meas_package);


    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);
    void PredictSensorMeasurement(MeasurementPackage meas_package);
    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    //DONE: JIN
    void Prediction(long delta_t);

    void GenerateAugmentedSigmaPoints(Eigen::MatrixXd& Xsig_out);
    void SigmaPointPrediction(Eigen::MatrixXd& Xsig_aug, double dt);
    void PredictMeanAndCovariance();

    void InitializeMeasurement(MeasurementPackage meas_package);
    void PredictRadarMeasurement();
    void PredictLidarMeasurement();
    void UpdateState(const Eigen::VectorXd& z);


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

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

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

    // State dimension
    int n_x_;

    // Augmented state dimension
    int n_aug_;

    // Number of sigma points
    int n_sigma_; 

    // Sigma point spreading parameter
    double lambda_;

    // current NIS for radar
    double NIS_radar_;

    // current NIS for laser
    double NIS_laser_;

private:
    // previous timestamp
    long previous_timestamp_;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z_;

    //create matrix for sigma points in measurement space
    Eigen::MatrixXd Zsig_;

    //mean predicted measurement
    Eigen::VectorXd z_pred_;

    //measurement covariance matrix S
    Eigen::MatrixXd S_;

    //measurement noise covariance matrix
    Eigen::MatrixXd R_lidar_, R_radar_;

    // if difference in time between two measurements 
    // is too large, subdivide the prediction step is needed for stability
    double dt_threshold_; 
    double default_dt_; 
    //radar can measure r, phi, and r_dot
    int n_z_radar_ = 3;
    //lidar measures position x,y 
    int n_z_lidar_ = 2; 

};