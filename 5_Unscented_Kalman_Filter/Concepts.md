# Motion Model
## CVRT
Constant turn rate and velocity magnitude.

### State Vector 
<img src="media/1_CVRT.png"/>

### Process Model 
Deterministic part: 

State vector at time `t+1` can be estimated using state at time `t` based on CVRT model. 
<img src="media/2_CVRT_process_model_deterministic.png"/>

Stochasitic part: 

Process Noise contains logitudinal acceleration noise and yaw acceleration noise, both are Gaussian distribution with zero mean. 
<img src="media/3_CVRT_process_noise.png"/>

# Radar Measurement


 
# Unscented Kalman Filter 
## Prediction Step 
Prediction step uses state mean and state covariance at timestep `k` to predict state mean and state covariance at timestep `k+1`. 

Here `k|k` means estimation for timestep `k` using measurement from timestep `k`. `k+1|k` means estimation for timestep `k+1` using measurement from timestep `k`. 

<img src="media/4_prediction_step_overview.png"/>

#### S1. Generate Sigma Points
Sigma points consists of 1 point at mean state, then 2 points for each state dimension. Here
λ controls how far the sigma point is from mean state. 

<img src="media/5_generate_sigma_points.png"/>

```cpp
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {
  // set state dimension
  int n_x = 5;
  // define spreading parameter
  double lambda = 3 - n_x;
  // state vector 
  VectorXd x = VectorXd(n_x);
  // covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);
  // calculate square root of P
  MatrixXd A = P.llt().matrixL();
  // set first column of sigma point matrix
  Xsig.col(0) = x;
  // set remaining sigma points
  for (int i = 0; i < n_x; ++i) {
    Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);
    Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);
  }
  *Xsig_out = Xsig;
}
```

Since process noise has a nonlinear effect on state, augmented sigma points is used. 

Augmented state consists of state vector and noise vector. The additional sigma points represents the uncertainty caused by the process noise. 

Matrix Q contains the variances of the process noise, the zeros is due to independence between longitudinal and yaw acceleration noise.

<img src="media/6_generate_augmented_sigma_points.png
"/>

```cpp
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
  // set state dimension
  int n_x = 5;
  // set augmented dimension
  int n_aug = 7;
  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;
  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;
  // define spreading parameter
  double lambda = 3 - n_aug;
  // set example state
  VectorXd x = VectorXd(n_x);
  // create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  // create augmented mean state (first n_x elements)
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;
  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }

  // write result
  *Xsig_out = Xsig_aug;
}

```

#### S2. Predict Sigma Points

#### S3. Predict Mean and Covariance

## Correction Step
#### S4. Predict Measurement
S4 transforms sigma points to measurement state, 
then use them to calculate mean and covariance. 

S1 needs augmented state, but S4 doesn't. S1 needs augmentation since 
process noise has non-linear effect on state. S4 doesn't need augmentation
since measurement noise had pure additive effect.


#### S5. Update State 

