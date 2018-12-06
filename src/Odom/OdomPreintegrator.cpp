#include "Odom/OdomPreintegrator.h"

namespace LIV
{

OdomPreintegrator::OdomPreintegrator(const OdomPreintegrator& pre):
    _delta_P(pre._delta_P), 
    _delta_R(pre._delta_R),
    
    _cov_P_Phi(pre._cov_P_Phi),
    _delta_time(pre._delta_time),
    _Tog(pre._Tog)
{

}

OdomPreintegrator::OdomPreintegrator()
{
    // delta measurements, position/velocity/rotation(matrix)
    _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // noise covariance propagation of delta measurements
    _cov_P_Phi.setZero();

    _delta_time = 0;
    _Tog<<0.0290,-0.9996, -0.0035, 0.2569,
    -0.1107,	0.0000,	-0.9939,	-0.0681,
    0.9934 ,	0.0292,	-0.1103,	-0.1547,
    0,		0,		0,		1;
}

void OdomPreintegrator::reset()
{
    // delta measurements, position/velocity/rotation(matrix)
    _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // noise covariance propagation of delta measurements
    _cov_P_Phi.setZero();

    _delta_time = 0;

}

// incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
void OdomPreintegrator::update(const Vector3d& v, const Vector3d& w, const double& dt)
{
    Eigen::Matrix3d Rog = _Tog.block<3,3>(0,0);
    Matrix3d dR = Expmap(w*dt);

    // noise covariance propagation of delta measurements
    // err_k+1 = A*err_k + B*err_gyro + C*err_acc
    Matrix3d I3x3 = Matrix3d::Identity();
    Matrix6d A = Matrix6d::Identity();
    Matrix3d Jr = Sophus::SO3d::JacobianR(Rog*w*dt);///
    A.block<3,3>(3,3) = dR.transpose();
    A.block<3,3>(0,3) = -_delta_R*skew(v)*dt;
    A.block<3,3>(0,0) = I3x3;
    Matrix<double,6,3> B = Matrix<double,6,3>::Zero();
//     B.block<3,3>(3,0) = I3x3*dt; ///////////
    B.block<3,3>(0,0) = Jr * Rog *dt; 
    Matrix<double,6,3> C = Matrix<double,6,3>::Zero();
    C.block<3,3>(0,0) = _delta_R*dt;
    _cov_P_Phi = A*_cov_P_Phi*A.transpose() +////???_cov_P
		 B*OdomData::getOmeMeasCov()*B.transpose() +
		 C*OdomData::getVelMeasCov()*C.transpose();
		
//     _J_P_Biasg += -_delta_R * skew(v) * _J_R_Biasg * dt;
//     _J_R_Biasg += - dR.transpose() * Jr * Rog * dt;
		 		 
		 
    // delta measurements, position/velocity/rotation(matrix)
    // update P first, then V, then R. because P's update need V&R's previous state
    _delta_P += _delta_R*v*dt;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_R = normalizeRotationM(_delta_R*dR);  // normalize rotation, in case of numerical error accumulation

    // delta time
    _delta_time += dt;

}

}
