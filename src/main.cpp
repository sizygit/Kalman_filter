#include "main.h"
#include "kalman_filter.h"
#include "math.h"
#include <vector>

using Eigen::Vector2d;

KalmanFilter kalman_t;

int main()
{
	/*example for - ROCKET ALTITUDE ESTIMATION
	  https://www.kalmanfilter.net/multiExamples.html */
	Vector2d x_vec(0, 0);
	VectorXd u_vec(1), z_vec(1);
	MatrixXd P_cov(2, 2), A_cov(2, 2), H_cov(1, 2);
	MatrixXd Q_cov(2, 2), R_cov(1, 1), B_cov(2, 1);
	float delta_t = 0.25;
	std::vector<double> z={6.43,1.3,39.43,45.89,41.44,48.7,78.06,80.08,61.77,75.15,110.39,127.83,158.75,156.55,213.32,229.82,262.8,297.57,335.69,367.92,377.19,411.18,460.7,468.39,553.9,583.97,655.15,723.09,736.85,787.22};
	std::vector<double> u={39.81,39.67,39.81,39.84,40.05,39.85,39.78,39.65,39.67,39.78,39.59,39.87,39.85,39.59,39.84,39.9,39.63,39.59,39.76,39.79,39.73,39.93,39.83,39.85,39.94,39.86,39.76,39.86,39.74,39.94};
	for (size_t i = 0; i < u.size(); i++)
	{
		u[i] =u[i] -9.8;
	}
	
	A_cov << 1, delta_t, 0, 1;
	B_cov << 0.5 * delta_t * delta_t, delta_t;
	P_cov << 500, 0, 0, 500;
	H_cov << 1, 0;
	R_cov << 400;
	Q_cov << pow(delta_t, 4) * 0.01 / 4, pow(delta_t, 3) * 0.01 / 2,
		pow(delta_t, 3) * 0.01 / 2, pow(delta_t, 2);

	/*initializa kalman filter*/
	kalman_t.Init(x_vec, P_cov, A_cov, H_cov, R_cov, Q_cov, B_cov);
	kalman_t.Predict();
	for (size_t i = 0; i < z.size(); i++)
	{
		u_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((u.data()+i), 1);
		z_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((z.data()+i), 1);
		// std::cout<<z_vec;
		kalman_t.Update(z_vec);
		kalman_t.Predict(u_vec);	
		std::cout <<i+1 <<"'s p_predict is "<< std::endl << kalman_t.P_ << std::endl;
		std::cout <<i+1 <<"'s x_predict is "<<i<< std::endl << kalman_t.x_ << std::endl;
	}
}