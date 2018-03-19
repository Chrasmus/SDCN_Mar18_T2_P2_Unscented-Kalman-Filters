#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //set state dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd::Zero(n_x_);

  // initial state covariance matrix
  P_ = MatrixXd::Zero(n_x_, n_x_);
  P_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.4;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* time when the state is true, in us
  time_us_ = 0;

  ///* Augmented state dimension (lesson 21)
  n_aug_ = 7;

  ///* predicted sigma points matrix (lesson 21)
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  ///* Weights of sigma points, with zeros
  weights_ = VectorXd::Zero(2*n_aug_+1);

  ///* Sigma point spreading parameter, for augmented state! (lesson 18)
  lambda_ = 3 - n_aug_; //n_x_;

  // previous timestamp
  previous_timestamp_ = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "UKF: x_ init start" << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho     = meas_package.raw_measurements_[0];
      float phi     = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[2];

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      //float vx = rho_dot * cos(phi);
      //float vy = rho_dot * sin(phi);
      //float v  = sqrt(vx*vx + vy*vy);

      x_(0) = px;
      x_(1) = py;
      //x_(2) = v;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = meas_package.timestamp_;
    //cout << "UKF: x_ init end" << endl;
    return;
  }
  // process each measurement after initialization
  else {

    // PREDICTION
    //cout << "UKF: prediction start" << endl;

    // dt is time difference expressed in seconds
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    time_us_ = meas_package.timestamp_;

    Prediction(dt);
    //cout << "UKF: prediction end" << endl;

    // UPDATE

    //cout << "UKF: update start" << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      //cout << "UKF: update Radar start" << endl;
      UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      //cout << "UKF: update Laser start" << endl;
      UpdateLidar(meas_package);
    }
    //cout << "UKF: update end" << endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /* OBSOLETE CODE, SINCE AUGMENTED SIGMA POINTS IS USED HERE

  // GENERATE SIGMA POINTS

  // create sigma points matrix, with zeros
  MatrixXd Xsig = MatrixXd::Zero(n_x_, 2 + n_x_ + 1);

  // calculate square root of state covariance matrix P_
  MatrixXd A = P_.llt().matrixL();

  // set first column of sigma point matrix to x_
  Xsig(0) = x_;

  // set remaining sigma points
  for (int i = 0; i <n_x_; i++){
    Xsig.col(i+1)      = x_ + sqrt(lambda+n_x_) * A.col(i);
    Xsig.col(i+1+n_x_) = x_ - sqrt(lambda+n_x_) * A.col(i);
  }
  */

  // GENERATE AUGMENTED SIGMA POINTS
  // create augmented mean state vector, with zeros
  VectorXd x_aug = VectorXd::Zero(n_aug_);

  // create augmented state covariance matrix, with zeros
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);

  //  create augmented sigma point matrix, with zeros
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

  // fill augmented mean state vector with values
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // fill augmented covariance matrix with values
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix, calculated from P_aug
  MatrixXd L = P_aug.llt().matrixL();

  // calculate augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  // PREDICT SIGMA POINTS

  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    // extract values for better readability
    double p_x      = Xsig_aug(0,i);
    double p_y      = Xsig_aug(1,i);
    double v        = Xsig_aug(2,i);
    double yaw      = Xsig_aug(3,i);
    double yawd     = Xsig_aug(4,i);
    double nu_a     = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p    = v;
    double yaw_p  = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p  = v_p + nu_a*delta_t;

    yaw_p  = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into the right column in Xsig_pred (lesson #21)
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  // PREDICT MEAN AND COVARIANCE, x_ and P_

  //cout << "ukf : prediction : x_ and P_" << endl;
  //cout << "ukf : prediction : weights_" << endl;

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //cout << "ukf : prediction : weights_" << weights_ << endl;
  //cout << "ukf : prediction : state mean x_" << endl;

  // predicted state mean x_ (x' = F*x+u)
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //cout << "ukf : prediction : state P_" << endl;

  // predicted state covariance matrix P_ (P' = F*P*Ft)
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    //cout << "1. x_diff(3) = " << x_diff(3) << endl;
    while (x_diff(3) >  M_PI) x_diff(3) -= 2.*M_PI;
    //cout << "2. i = " << i << endl;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;
    //cout << "3. i = " << i << endl;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    //cout << "4. i = " << i << endl;
  }
  //cout << "UKF: ending function prediction" << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // get the measurements
  //cout << "UKF: starting update lidar" << endl;

  VectorXd z = meas_package.raw_measurements_;

  // set measurement dimension for lidar
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    // measurement model
    Zsig(0,i) = p_x; //sqrt(p_x*p_x + p_y*p_y);
    Zsig(1,i) = p_y; //atan2(p_y, p_x);
  }
  // create and fill mean predicted measurement vector
  VectorXd z_pred = VectorXd::Zero(n_z);
  for (int i=0; i < 2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // create and fill innovation covariance matrix S
  // matrix for predicted measurement covariance
  // S = H * P *Ht + R
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

    // add measurement noise covariance matrix R to covariance matrix S
    MatrixXd R = MatrixXd::Zero(n_z, n_z);
    R <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;
    S = S + R;

    // create matrix for cross correlation Tc, with zeros
    MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

    // calculate cross correlation matrix Tc
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

      // residual
      VectorXd z_diff = Zsig.col(i) - z_pred;
      // state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;

      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // THE RESULT
    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

  // Calculate NIS for laser (UKF lesson 8/32, 4:22)
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  //cout << "UKF: ending update lidar" << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //cout << "UKF: starting update radar" << endl;

  // get the measurements
  //cout << "UKF: inside update radar" << endl;
  VectorXd z = meas_package.raw_measurements_;

  // set measurement dimension for radar
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model, check for division with zero
    if (p_x == 0 && p_y == 0) {
      Zsig(0,i) = 0;
      Zsig(1,i) = 0;
      Zsig(2,i) = 0;
    } else {
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
      Zsig(1,i) = atan2(p_y,p_x);                                 //phi
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
  }
  //cout << "UKF: inside update radar 1" << endl;

  // create and fill mean predicted measurement vector
  VectorXd z_pred = VectorXd::Zero(n_z);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // create and fill innovation covariance matrix S
  // matrix for predicted measurement covariance
  // S = H * P *Ht + R
  //cout << "UKF: inside update radar 2" << endl;
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix R to covariance matrix S
  //cout << "UKF: inside update radar 3" << endl;
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  //cout << "UKF: inside update radar 4" << endl;
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0, std_radrd_*std_radrd_;
  S = S + R;

  // create matrix for cross correlation Tc, with zeros
  //cout << "UKF: inside update radar 6" << endl;
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  // calculate cross correlation matrix Tc
  //cout << "UKF: inside update radar 7" << endl;
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

  // THE RESULT
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for radar (UKF lesson 8/32, 4:22)
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  //cout << "UKF: ending update radar" << endl;

}
