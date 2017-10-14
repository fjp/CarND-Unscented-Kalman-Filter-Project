#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

    // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // State dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Sigma point spreading parameter
    lambda_ = 3 - n_aug;


    // initial state vector
    x_ = VectorXd(n_x_);
    x_ = VectorXd::Ones(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_ = MatrixXd::Identy(n_x_, n_x_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 3;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.3;

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

    /**
TODO:

Complete the initialization. See ukf.h for other member properties.

Hint: one or more values initialized above might be wildly off...
*/

    // Weights of sigma points
    VectorXd weights_ ;
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
     *          *  Initialization
     *****************************************************************************/
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    if (!is_initialized) {
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
              Convert radar from polar to cartesian coordinates and initialize state.
              */
            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            float px = rho*cos(phi);
            float py = rho*sin(phi);
            float vx = 0;
            float vy = 0;

            ekf_.x_ << px, py, vx, vy;
            cout << "First measurement was radar" << endl;


            Hj_ = tools.CalculateJacobian(ekf_.x_);

            //measurement matrix
            ekf_.H_ = Hj_;

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
              Initialize state.
              */
            //set the state with the initial location and zero velocity
            float px = measurement_pack.raw_measurements_[0];
            float py = measurement_pack.raw_measurements_[1];
            float vx = 0;
            float vy = 0;
            ekf_.x_ << px, py, vx, vy;
            cout << "First measurement was laser" << endl;

            //measurement matrix
            ekf_.H_ = H_laser_;
        }

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    meas_package
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
}
