#include "IMU/imudata.h"


namespace vill {

    // covariance of measurement
    // From continuous noise_density of dataset sigma_g/sigma_a   rad/s/sqrt(Hz) -- m/s^2/sqrt(Hz)

    /**
     * For EuRoc dataset, according to V1_01_easy/imu0/sensor.yaml
     * The params:
     * sigma_g: 1.6968e-4       rad / s / sqrt(Hz)
     * sigma_gw: 1.9393e-5      rad / s^2 / sqrt(Hz)
     * sigma_a: 2.0e-3          m / s^2 / sqrt(Hz)
     * sigma_aw: 3.0e-3         m / s^3 / sqrt(Hz)
     */

    // *1e3/*1e2 chosen by experiments
    double IMUData::_gyrBiasRw2 = 2.0e-6 * 2.0e-6 * 10;  //2e-12*1e3
    double IMUData::_accBiasRw2 = 1.0e-3 * 2.0e-3 * 10;  //4.5e-8*1e2//2.0e-3 * 2.0e-3 * 10;

    Matrix3d IMUData::_gyrMeasCov =
            Matrix3d::Identity() * 1.7e-4 * 1.7e-4 / 0.005 * 100;       // sigma_g * sigma_g / dt, ~6e-6*10
    Matrix3d IMUData::_accMeasCov =
            Matrix3d::Identity() * 2.0e-3 * 2.0e-3 / 0.005 * 100;       // sigma_a * sigma_a / dt, ~8e-4*10

    // covariance of bias random walk
//     Matrix3d IMUData::_inverseIdentityx;
//     _inverseIdentityx<<-1,0,0,0,1,0,0,0,1;
    Matrix3d IMUData::_gyrBiasRWCov = Matrix3d::Identity()* _gyrBiasRw2;     // sigma_gw * sigma_gw * dt, ~2e-12	
    Matrix3d IMUData::_accBiasRWCov = Matrix3d::Identity() * _accBiasRw2;     // sigma_aw * sigma_aw * dt, ~4.5e-8
	std::ostream & operator <<(std::ostream &out, IMUData &imu){
		out << imu._t<< " " << imu._a <<" "<< imu._g;
		return out;
	}
}
