#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


#include "Eigen/Dense"
// #include "iostream"
using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter {
public:

  // state vector and Input Variable
  Eigen::VectorXd x_, u_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  //Control Matrix
  Eigen::MatrixXd B_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  /**
  * Constructor
  */
  /*KalmanFilter();*/ 

  /**
   * Destructor
   */
  // virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   * @param B_in Control Matrix
   */
  void Init(const VectorXd &x_in, const MatrixXd &P_in, const  MatrixXd &F_in,
	       const  MatrixXd &H_in, const  MatrixXd &R_in, const  MatrixXd &Q_in, const MatrixXd &B_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();
  void Predict(const Eigen::VectorXd &u_in);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
