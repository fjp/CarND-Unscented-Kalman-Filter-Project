#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
    public:

        ///* initially set to false, set to true in first call of ProcessMeasurement
        bool is_initialized_;

        ///* if this is false, laser measurements will be ignored (except for init)
        bool use_laser_;

        ///* if this is false, radar measurements will be ignored (except for init)
        bool use_radar_;

        ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        VectorXd x_;

        ///* state covariance matrix
        MatrixXd P_;

        ///* augmented state covariance matrix
        MatrixXd P_aug_;

        ///* predicted sigma points matrix
        MatrixXd Xsig_pred_;

        ///* time when the state is true, in us
        long long time_us_;

        ///* Process noise standard deviation longitudinal acceleration in m/s^2
        double std_a_;

        ///* Process noise standard deviation yaw acceleration in rad/s^2
        double std_yawdd_;

        ///* Laser measurement noise standard deviation position1 in m
        double std_laspx_;

        ///* Laser measurement noise standard deviation position2 in m
        double std_laspy_;

        ///* Measurement noise covariance matrix for laser
        MatrixXd R_las_;

        ///* Radar measurement noise standard deviation radius in m
        double std_radr_;

        ///* Radar measurement noise standard deviation angle in rad
        double std_radphi_;

        ///* Radar measurement noise standard deviation radius change in m/s
        double std_radrd_ ;

        ///* Measurement noise covariance matrix for radar
        MatrixXd R_rad_;

        ///* Weights of sigma points
        VectorXd weights_;

        ///* State dimension
        int n_x_;

        ///* Augmented state dimension
        int n_aug_;

        ///* Measurement dimension for laser
        int n_z_las_;

        ///* Measurement dimension for radar
        int n_z_rad_;

        ///* Sigma point spreading parameter
        double lambda_;

        ///* the current NIS for radar
        double NIS_radar_;

        ///* the current NIS for laser
        double NIS_laser_;


        /**
         * Constructor
         */
        UKF();

        /**
         * Destructor
         */
        virtual ~UKF();

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
        void Prediction(double delta_t);

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


        /**
         * Create augmented sigma point matrix
         * @param Xsig_aug The augmented sigma point matrix
         */
        void AugmentedSigmaPoints(MatrixXd& Xsig_aug);

        /**
         * Predict the sigma points using the process model
         * @param Xsig_aug The augmented sigma point matrix
         */
        void SigmaPointPrediction(MatrixXd& Xsig_aug, double delta_t);

        /**
         * Calculate the predicted mean and covariance from the sigma points
         */
        void PredictMeanAndCovariance();

        /**
         * Transform the predicted state into the measurement space and
         * calculate the measurement mean and its covariance using the
         * radar measurement model
         * @param Zsig The matrix with the predicted sigma points in the measurement space
         * @param z_pred The mean predicted measurement
         * @param S The radar measurement covariance matrix
         */
        void PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S);

        /**
         * Update the state depending on the current measurement (radar or lidar)
         * @param Xsig_pred The matrix with the predicted sigma points
         */
        void UpdateState(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S, VectorXd& z, double& NIS);

};

#endif /* UKF_H */
