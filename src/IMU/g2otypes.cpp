#include "IMU/g2otypes.h"
#include <boost/graph/graph_concepts.hpp>
#include "Eigen/Core"
#include <Eigen/Geometry>
// #include "Thirdparty/g2o/g2o/types"

using namespace vill;
namespace g2o {
  
    Matrix3d fromEuler(const Vector3d& v) {
      //UNOPTIMIZED
      double roll  = v[0];
      double pitch = v[1];
      double yaw   = v[2];
      double sy = std::sin(yaw*double(0.5));
      double cy = std::cos(yaw*double(0.5));
      double sp = std::sin(pitch*double(0.5));
      double cp = std::cos(pitch*double(0.5));
      double sr = std::sin(roll*double(0.5));
      double cr = std::cos(roll*double(0.5));
      double w = cr*cp*cy + sr*sp*sy;
      double x = sr*cp*cy - cr*sp*sy;
      double y = cr*sp*cy + sr*cp*sy;
      double z = cr*cp*sy - sr*sp*cy;
      return Quaterniond(w,x,y,z).toRotationMatrix();
    }

  bool EdgeXYZLaser::read(istream& is)
  {

  }

  bool EdgeXYZLaser::write(ostream& os) const
  {

  }

//   bool EdgePoseSE3::read(istream& is)
//   {
// 
//   }
// 
//   bool EdgePoseSE3::write(ostream& os) const
//   {
// 
//   }
// 
//   void EdgePoseSE3::linearizeOplus()
//   {
//   g2o::BaseUnaryEdge< 6, g2o::SE3Quat, g2o::VertexSE3Expmap >::linearizeOplus();
//   return ;
// 	  
//   }

  bool EdgePointToPlain::write(std::ostream& os) const
  {

  }

  bool EdgePointToPlain::read(istream& is)
  {

  }
  
	void EdgePrioriNavStatePVRBias::computeError()
	{
        const VertexNavStatePVR *vNSPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const VertexNavStateBias *vNSBias = static_cast<const VertexNavStateBias *>(_vertices[1]);
        const NavState &nsPVRest = vNSPVR->estimate();
        const NavState &nsBiasest = vNSBias->estimate();
        const NavState &nsprior = _measurement;

        // P V R bg+dbg ba+dba
        Vector15d err = Vector15d::Zero();
        // PVR terms
        Sophus::SO3d Riprior_inv = nsprior.Get_R().inverse();
        // eP = R_prior^-1*(P_est - P_prior) --not <eP = P_prior - P_est>--
        err.segment<3>(0) = Riprior_inv * (nsPVRest.Get_P() - nsprior.Get_P());
        // eV = (V_est - V_prior) --not <V_prior - V_est>--
        err.segment<3>(3) = (nsPVRest.Get_V() - nsprior.Get_V());
        // eR = log(R_prior^-1 * R_est)
        err.segment<3>(6) = (Riprior_inv * nsPVRest.Get_R()).log();

        // Bias terms
        // eB = Bias_prior - Bias_est
        // err_bg = (bg_prior+dbg_prior) - (bg+dbg)
        err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) -
                            (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());
        // err_ba = (ba_prior+dba_prior) - (ba+dba)
        err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) -
                             (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());

        _error = err;

        //Test log
        if ((nsPVRest.Get_BiasGyr() - nsBiasest.Get_BiasGyr()).norm() > 1e-6 ||
            (nsPVRest.Get_BiasAcc() - nsBiasest.Get_BiasAcc()).norm() > 1e-6) {
            std::cerr << "bias gyr not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias" << std::endl
                      << nsPVRest.Get_BiasGyr().transpose() << " / " << nsBiasest.Get_BiasGyr().transpose()
                      << std::endl;
            std::cerr << "bias acc not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias" << std::endl
                      << nsPVRest.Get_BiasAcc().transpose() << " / " << nsBiasest.Get_BiasAcc().transpose()
                      << std::endl;
        }
	}
	
	void EdgePrioriNavStatePVRBias::linearizeOplus()
	{
        // Estimated NavState
        const VertexNavStatePVR *vPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const NavState &nsPVRest = vPVR->estimate();
        const NavState &nsprior = _measurement;

        _jacobianOplusXi = Matrix<double, 15, 9>::Zero();
        _jacobianOplusXi.block<3, 3>(0, 0) = (nsprior.Get_R().inverse() * nsPVRest.Get_R()).matrix();
        _jacobianOplusXi.block<3, 3>(3, 3) = Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(6, 6) = Sophus::SO3::JacobianRInv(_error.segment<3>(6));

        _jacobianOplusXj = Matrix<double, 15, 6>::Zero();
        _jacobianOplusXj.block<3, 3>(9, 0) = -Matrix3d::Identity();
        _jacobianOplusXj.block<3, 3>(12, 3) = -Matrix3d::Identity();
	
	}


  
    void EdgeNavStatePVR::computeError() {
        //
        const VertexNavStatePVR *vPVRi = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const VertexNavStatePVR *vPVRj = static_cast<const VertexNavStatePVR *>(_vertices[1]);
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[2]);

        // terms need to computer error in vertex i, except for bias error
        const NavState &NSPVRi = vPVRi->estimate();
        Vector3d Pi = NSPVRi.Get_P();
        Vector3d Vi = NSPVRi.Get_V();
        SO3d Ri = NSPVRi.Get_R();
        // Bias from the bias vertex
        const NavState &NSBiasi = vBiasi->estimate();
        Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
        Vector3d dBai = NSBiasi.Get_dBias_Acc();

        // terms need to computer error in vertex j, except for bias error
        const NavState &NSPVRj = vPVRj->estimate();
        Vector3d Pj = NSPVRj.Get_P();
        Vector3d Vj = NSPVRj.Get_V();
        SO3d Rj = NSPVRj.Get_R();

        // IMU Preintegration measurement
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();   // Delta Time
        double dT2 = dTij * dTij;
        Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
        Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
        Sophus::SO3d dRij = Sophus::SO3(M.getDeltaR());  // Delta Rotation pre-integration measurement

        // tmp variable, transpose of Ri
        Sophus::SO3d RiT = Ri.inverse();
        // residual error of Delta Position measurement
        Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2)
                        - (dPij + M.getJPBiasg() * dBgi +
                           M.getJPBiasa() * dBai);   // this line includes correction term of bias change.
        // residual error of Delta Velocity measurement
        Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij)
                        - (dVij + M.getJVBiasg() * dBgi +
                           M.getJVBiasa() * dBai);   //this line includes correction term of bias change
        // residual error of Delta Rotation measurement
        Sophus::SO3d dR_dbg = Sophus::SO3d::exp(M.getJRBiasg() * dBgi);
        Sophus::SO3d rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
        Vector3d rPhiij = rRij.log();


        Vector9d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=9
        err.setZero();

        // 9-Dim error vector order:
        // position-velocity-rotation
        // rPij - rVij - rPhiij
        err.segment<3>(0) = rPij;       // position error
        err.segment<3>(3) = rVij;       // velocity error
        err.segment<3>(6) = rPhiij;     // rotation phi error

        _error = err;
    }

    void EdgeNavStatePVR::linearizeOplus() {
        //
        const VertexNavStatePVR *vPVRi = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const VertexNavStatePVR *vPVRj = static_cast<const VertexNavStatePVR *>(_vertices[1]);
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[2]);

        // terms need to computer error in vertex i, except for bias error
        const NavState &NSPVRi = vPVRi->estimate();
        Vector3d Pi = NSPVRi.Get_P();
        Vector3d Vi = NSPVRi.Get_V();
        Matrix3d Ri = NSPVRi.Get_RotMatrix();
        // bias
        const NavState &NSBiasi = vBiasi->estimate();
        Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
        //    Vector3d dBai = NSBiasi.Get_dBias_Acc();

        // terms need to computer error in vertex j, except for bias error
        const NavState &NSPVRj = vPVRj->estimate();
        Vector3d Pj = NSPVRj.Get_P();
        Vector3d Vj = NSPVRj.Get_V();
        Matrix3d Rj = NSPVRj.Get_RotMatrix();

        // IMU Preintegration measurement
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();   // Delta Time
        double dT2 = dTij * dTij;

        // some temp variable
        Matrix3d I3x3 = Matrix3d::Identity();   // I_3x3
        Matrix3d O3x3 = Matrix3d::Zero();       // 0_3x3
        Matrix3d RiT = Ri.transpose();          // Ri^T
        Matrix3d RjT = Rj.transpose();          // Rj^T
        Vector3d rPhiij = _error.segment<3>(6); // residual of rotation, rPhiij
        Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
        Matrix3d J_rPhi_dbg = M.getJRBiasg();              // jacobian of preintegrated rotation-angle to gyro bias i

        // 1.
        // increment is the same as Forster 15'RSS
        // pi = pi + Ri*dpi,    pj = pj + Rj*dpj
        // vi = vi + dvi,       vj = vj + dvj
        // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
        //      Note: the optimized bias term is the 'delta bias'
        // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
        // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

        // 2.
        // 9-Dim error vector order in PVR:
        // position-velocity-rotation
        // rPij - rVij - rPhiij
        //      Jacobian row order:
        // J_rPij_xxx
        // J_rVij_xxx
        // J_rPhiij_xxx

        // 3.
        // order in 'update_' in PVR
        // Vertex_i : dPi, dVi, dPhi_i
        // Vertex_j : dPj, dVj, dPhi_j
        // 6-Dim error vector order in Bias:
        // dBiasg_i - dBiasa_i

        // 4.
        // For Vertex_PVR_i
        Matrix<double, 9, 9> JPVRi;
        JPVRi.setZero();

        // 4.1
        // J_rPij_xxx_i for Vertex_PVR_i
        JPVRi.block<3, 3>(0, 0) = -I3x3;      //J_rP_dpi
        JPVRi.block<3, 3>(0, 3) = -RiT * dTij;  //J_rP_dvi
        JPVRi.block<3, 3>(0, 6) = Sophus::SO3::hat(
                RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    //J_rP_dPhi_i

        // 4.2
        // J_rVij_xxx_i for Vertex_PVR_i
        JPVRi.block<3, 3>(3, 0) = O3x3;    //dpi
        JPVRi.block<3, 3>(3, 3) = -RiT;    //dvi
        JPVRi.block<3, 3>(3, 6) = Sophus::SO3::hat(RiT * (Vj - Vi - GravityVec * dTij));    //dphi_i

        // 4.3
        // J_rPhiij_xxx_i for Vertex_PVR_i
        Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
        Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg * dBgi);
        JPVRi.block<3, 3>(6, 0) = O3x3;    //dpi
        JPVRi.block<3, 3>(6, 3) = O3x3;    //dvi
        JPVRi.block<3, 3>(6, 6) = -JrInv_rPhi * RjT * Ri;    //dphi_i


        // 5.
        // For Vertex_PVR_j
        Matrix<double, 9, 9> JPVRj;
        JPVRj.setZero();

        // 5.1
        // J_rPij_xxx_j for Vertex_PVR_j
        JPVRj.block<3, 3>(0, 0) = RiT * Rj;  //dpj
        JPVRj.block<3, 3>(0, 3) = O3x3;    //dvj
        JPVRj.block<3, 3>(0, 6) = O3x3;    //dphi_j

        // 5.2
        // J_rVij_xxx_j for Vertex_PVR_j
        JPVRj.block<3, 3>(3, 0) = O3x3;    //dpj
        JPVRj.block<3, 3>(3, 3) = RiT;    //dvj
        JPVRj.block<3, 3>(3, 6) = O3x3;    //dphi_j

        // 5.3
        // J_rPhiij_xxx_j for Vertex_PVR_j
        JPVRj.block<3, 3>(6, 0) = O3x3;    //dpj
        JPVRj.block<3, 3>(6, 3) = O3x3;    //dvj
        JPVRj.block<3, 3>(6, 6) = JrInv_rPhi;    //dphi_j


        // 6.
        // For Vertex_Bias_i
        Matrix<double, 9, 6> JBiasi;
        JBiasi.setZero();

        // 5.1
        // J_rPij_xxx_j for Vertex_Bias_i
        JBiasi.block<3, 3>(0, 0) = -M.getJPBiasg();     //J_rP_dbgi
        JBiasi.block<3, 3>(0, 3) = -M.getJPBiasa();     //J_rP_dbai

        // J_rVij_xxx_j for Vertex_Bias_i
        JBiasi.block<3, 3>(3, 0) = -M.getJVBiasg();    //dbg_i
        JBiasi.block<3, 3>(3, 3) = -M.getJVBiasa();    //dba_i

        // J_rPhiij_xxx_j for Vertex_Bias_i
        JBiasi.block<3, 3>(6, 0) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
        JBiasi.block<3, 3>(6, 3) = O3x3;    //dba_i

        // Evaluate _jacobianOplus
        _jacobianOplus[0] = JPVRi;
        _jacobianOplus[1] = JPVRj;
        _jacobianOplus[2] = JBiasi;
    }

    void EdgeNavStateBias::computeError() {
        const VertexNavStateBias *vBiasi = static_cast<const VertexNavStateBias *>(_vertices[0]);
        const VertexNavStateBias *vBiasj = static_cast<const VertexNavStateBias *>(_vertices[1]);

        const NavState &NSi = vBiasi->estimate();
        const NavState &NSj = vBiasj->estimate();

        // residual error of Gyroscope's bias, Forster 15'RSS
        Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                          - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

        // residual error of Accelerometer's bias, Forster 15'RSS
        Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                          - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

        Vector6d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=6
        err.setZero();
        // 6-Dim error vector order:
        // deltabiasGyr_i-deltabiasAcc_i
        // rBiasGi - rBiasAi
        err.segment<3>(0) = rBiasG;     // bias gyro error
        err.segment<3>(3) = rBiasA;    // bias acc error

        _error = err;
    }

    void EdgeNavStateBias::linearizeOplus() {
        // 6-Dim error vector order:
        // deltabiasGyr_i-deltabiasAcc_i
        // rBiasGi - rBiasAi

        _jacobianOplusXi = -Matrix<double, 6, 6>::Identity();
        _jacobianOplusXj = Matrix<double, 6, 6>::Identity();
    }

//     void EdgeExtrinsicCalibration::linearizeOplus() {
//         const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
//         const VertexNavStatePVR *vNavState = static_cast<const VertexNavStatePVR *>(_vertices[1]);
// 	const VertexExCalib *vExCalib = static_cast<const VertexExCalib *>(_vertices[2]);
// 	
// 	SE3d Tbc = vExCalib->estimate();
// 	Matrix3d Rbc = Tbc.rotationMatrix();
// 	Vector3d Pbc = Tbc.translation();
// 
//         const NavState &ns = vNavState->estimate();
//         Matrix3d Rwb = ns.Get_RotMatrix();
//         Vector3d Pwb = ns.Get_P();
//         const Vector3d &Pw = vPoint->estimate();
// 
//         Matrix3d Rcb = Rbc.transpose();
//         Vector3d Pc = Rbc.transpose() * Rwb.transpose() * (Pw - Pwb) - Rbc.transpose() * Pbc;
// 
//         double x = Pc[0];
//         double y = Pc[1];
//         double z = Pc[2];
// 
//         // Jacobian of camera projection
//         Matrix<double, 2, 3> Maux;
//         Maux.setZero();
//         Maux(0, 0) = fx;
//         Maux(0, 1) = 0;
//         Maux(0, 2) = -x / z * fx;
//         Maux(1, 0) = 0;
//         Maux(1, 1) = fy;
//         Maux(1, 2) = -y / z * fy;
//         Matrix<double, 2, 3> Jpi = Maux / z;
// 
//         // error = obs - pi( Pc )
//         // Pw <- Pw + dPw,          for Point3D
//         // Rwb <- Rwb*exp(dtheta),  for NavState.R
//         // Pwb <- Pwb + Rwb*dPwb,   for NavState.P
// 	// Rbc <- Rbc*exp(dtheta),  for Calib.R
// 	// Pbc <- Pbc + dp,         for Calib.p
// 
//         // Jacobian of error w.r.t Pw
//         _jacobianOplus[0] = -Jpi * Rbc.transpose() * Rwb.transpose();
// 
//         // Jacobian of Pc/error w.r.t dPwb
//         Matrix<double, 2, 3> JdPwb = -Jpi * (-Rbc.transpose());
//         // Jacobian of Pc/error w.r.t dRwb
//         Vector3d Paux = Rbc.transpose() * Rwb.transpose() * (Pw - Pwb);
//         Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rbc.transpose());
// 
//         // Jacobian of Pc w.r.t NavState
//         // order in 'update_': dP, dV, dPhi
//         Matrix<double, 2, 9> JNavState = Matrix<double, 2, 9>::Zero();
//         JNavState.block<2, 3>(0, 0) = JdPwb;
//         JNavState.block<2, 3>(0, 6) = JdRwb;
// 
//         // Jacobian of error w.r.t NavState
//         _jacobianOplus[1] = JNavState;
// 	
// 	// Jacobian of Pc/error w.r.t dPbc
//         Matrix<double, 2, 3> JdPcb = -Jpi * (-Rbc.transpose());
//         // Jacobian of Pc/error w.r.t dRbc
//         Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
//         Matrix<double, 2, 3> JdRbc = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);
// 
//         // Jacobian of Pc w.r.t Calib
//         Matrix<double, 2, 6> JCalib = Matrix<double, 2, 6>::Zero();
//         JCalib.block<2, 3>(0, 0) = JdPcb;
//         JCalib.block<2, 3>(0, 3) = JdRbc;
// 	
// 	// Jacobian of error w.r.t Calib
//         _jacobianOplus[2] = JCalib;
// 	
//     }
    
#ifdef TEST_STEREO
    //rocky  add for stereo vio
//
void EdgeStereoNavStatePVRPointXYZ::linearizeOplus()
{
    const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    const VertexNavStatePVR* vNavState = static_cast<const VertexNavStatePVR*>(_vertices[1]);

    const NavState& ns = vNavState->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];
    double z_2 = z * z;

    const Matrix3d R =  Rcb * Rwb.transpose() ;//Rcw
    _jacobianOplusXi(0, 0) = -fx * R(0, 0) / z + fx * x * R(2, 0) / z_2;
    _jacobianOplusXi(0, 1) = -fx * R(0, 1) / z + fx * x * R(2, 1) / z_2;
    _jacobianOplusXi(0, 2) = -fx * R(0, 2) / z + fx * x * R(2, 2) / z_2;

    _jacobianOplusXi(1, 0) = -fy * R(1, 0) / z + fy * y * R(2, 0) / z_2;
    _jacobianOplusXi(1, 1) = -fy * R(1, 1) / z + fy * y * R(2, 1) / z_2;
    _jacobianOplusXi(1, 2) = -fy * R(1, 2) / z + fy * y * R(2, 2) / z_2;

    _jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - bf * R(2, 0) / z_2;
    _jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) - bf * R(2, 1) / z_2;
    _jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2) - bf * R(2, 2) / z_2;


    // Jacobian of camera projection
    Matrix<double, 3, 3> stereoMaux;
    stereoMaux.setZero();
    stereoMaux(0, 0) = fx;
    stereoMaux(0, 1) = 0;
    stereoMaux(0, 2) = -x / z * fx;
    stereoMaux(1, 0) = 0;
    stereoMaux(1, 1) = fy;
    stereoMaux(1, 2) = -y / z * fy;
    stereoMaux(2, 0) = fx;
    stereoMaux(2, 1) = 0;
    stereoMaux(2, 2) = -x / z * fx + bf / z;
    Matrix<double, 3, 3> stereoJpi = -stereoMaux / z; //重投影误差对于相机坐标系下的点Pc的雅克比矩阵,注意：这里已经加了负号


//rocky
//jacobian  of Pc with PVR
    Matrix<double, 3, 9> JPcPVR = Matrix<double, 3, 9>::Zero();
    JPcPVR.block<3, 3>(0, 0) = -Rcb;

    Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
    JPcPVR.block<3, 3>(0, 6) = Sophus::SO3::hat(Paux) * Rcb;

    _jacobianOplusXj = stereoJpi * JPcPVR;




    /*
        // Jacobian of Pc/error w.r.t dPwb
        //Matrix3d J_Pc_dPwb = -Rcb;//j见orbvio论文的式（5）
        Matrix<double,2,3> JdPwb = - Jpi * (-Rcb);
        // Jacobian of Pc/error w.r.t dRwb
        Vector3d Paux = Rcb*Rwb.transpose()*(Pw-Pwb);
        //Matrix3d J_Pc_dRwb = Sophus::SO3::hat(Paux) * Rcb;
        Matrix<double,2,3> JdRwb = - Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        // Jacobian of Pc w.r.t NavState
        // order in 'update_': dP, dV, dPhi
        Matrix<double,2,9> JNavState = Matrix<double,2,9>::Zero();
        JNavState.block<2,3>(0,0) = JdPwb;
        //JNavState.block<2,3>(0,3) = 0;
        JNavState.block<2,3>(0,6) = JdRwb;
        //JNavState.block<2,3>(0,9) = 0;
        //JNavState.block<2,3>(0,12) = 0;

        // Jacobian of error w.r.t NavState
        _jacobianOplusXj = JNavState;
        */

}

//rocky for stereo vio  EdgeStereoNavStatePVRPointXYZOnlyPose
void EdgeStereoNavStatePVRPointXYZOnlyPose::linearizeOplus()
{
    const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);

    const NavState& ns = vNSPVR->estimate();
    Matrix3d Rwb = ns.Get_RotMatrix();
    Vector3d Pwb = ns.Get_P();
    //const Vector3d& Pw = vPoint->estimate();

    Matrix3d Rcb = Rbc.transpose();
    Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];
    //double z_2 = z * z;

    // Jacobian of camera projection
    Matrix<double, 3, 3> stereoMaux;
    stereoMaux.setZero();
    stereoMaux(0, 0) = fx;
    stereoMaux(0, 1) = 0;
    stereoMaux(0, 2) = -x / z * fx;
    stereoMaux(1, 0) = 0;
    stereoMaux(1, 1) = fy;
    stereoMaux(1, 2) = -y / z * fy;
    stereoMaux(2, 0) = fx;
    stereoMaux(2, 1) = 0;
    stereoMaux(2, 2) = -x / z * fx + bf / z;
    Matrix<double, 3, 3>stereoJpi = -stereoMaux / z; //重投影误差对于相机坐标系下的点Pc的雅克比矩阵,注意：这里已经加了负号


//rocky
//jacobian  of Pc with PVR
    Matrix<double, 3, 9> JPcPVR = Matrix<double, 3, 9>::Zero();
    JPcPVR.block<3, 3>(0, 0) = -Rcb;

    Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
    JPcPVR.block<3, 3>(0, 6) = Sophus::SO3::hat(Paux) * Rcb;

    // Jacobian of error w.r.t NavStatePVR
    _jacobianOplusXi =stereoJpi * JPcPVR;
}
#endif
    
    void EdgeNavStatePVRPointXYZ::linearizeOplus() {
        const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
        const VertexNavStatePVR *vNavState = static_cast<const VertexNavStatePVR *>(_vertices[1]);

        const NavState &ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d &Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // Jacobian of camera projection
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - pi( Pc )
        // Pw <- Pw + dPw,          for Point3D
        // Rwb <- Rwb*exp(dtheta),  for NavState.R
        // Pwb <- Pwb + Rwb*dPwb,   for NavState.P

        // Jacobian of error w.r.t Pw
        _jacobianOplusXi = -Jpi * Rcb * Rwb.transpose();

        // Jacobian of Pc/error w.r.t dPwb
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb);
        // Jacobian of Pc/error w.r.t dRwb
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        // Jacobian of Pc w.r.t NavState
        // order in 'update_': dP, dV, dPhi
        Matrix<double, 2, 9> JNavState = Matrix<double, 2, 9>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        // Jacobian of error w.r.t NavState
        _jacobianOplusXj = JNavState;
    }

    void EdgeNavStatePVRPointXYZOnlyPose::linearizeOplus() {
        const VertexNavStatePVR *vNSPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);

        const NavState &ns = vNSPVR->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // Jacobian of camera projection
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - pi( Pc )
        // Pw <- Pw + dPw,          for Point3D
        // Rwb <- Rwb*exp(dtheta),  for NavState.R
        // Pwb <- Pwb + Rwb*dPwb,   for NavState.P

        // Jacobian of Pc/error w.r.t dPwb
        //Matrix3d J_Pc_dPwb = -Rcb;
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb);
        // Jacobian of Pc/error w.r.t dRwb
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        // Jacobian of Pc w.r.t NavStatePVR
        // order in 'update_': dP, dV, dPhi
        Matrix<double, 2, 9> JNavState = Matrix<double, 2, 9>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        // Jacobian of error w.r.t NavStatePVR
        _jacobianOplusXi = JNavState;
    }
    

    
    void EdgeNavStatePriorPVRBias::computeError() {
        const VertexNavStatePVR *vNSPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const VertexNavStateBias *vNSBias = static_cast<const VertexNavStateBias *>(_vertices[1]);
        const NavState &nsPVRest = vNSPVR->estimate();
        const NavState &nsBiasest = vNSBias->estimate();
        const NavState &nsprior = _measurement;

        // P V R bg+dbg ba+dba
        Vector15d err = Vector15d::Zero();
        // PVR terms
        Sophus::SO3d Riprior_inv = nsprior.Get_R().inverse();
        // eP = R_prior^-1*(P_est - P_prior) --not <eP = P_prior - P_est>--
        err.segment<3>(0) = Riprior_inv * (nsPVRest.Get_P() - nsprior.Get_P());
        // eV = (V_est - V_prior) --not <V_prior - V_est>--
        err.segment<3>(3) = (nsPVRest.Get_V() - nsprior.Get_V());
        // eR = log(R_prior^-1 * R_est)
        err.segment<3>(6) = (Riprior_inv * nsPVRest.Get_R()).log();

        // Bias terms
        // eB = Bias_prior - Bias_est
        // err_bg = (bg_prior+dbg_prior) - (bg+dbg)
        err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) -
                            (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());
        // err_ba = (ba_prior+dba_prior) - (ba+dba)
        err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) -
                             (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());

        _error = err;

        //Test log
        if ((nsPVRest.Get_BiasGyr() - nsBiasest.Get_BiasGyr()).norm() > 1e-6 ||
            (nsPVRest.Get_BiasAcc() - nsBiasest.Get_BiasAcc()).norm() > 1e-6) {
            std::cerr << "bias gyr not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias" << std::endl
                      << nsPVRest.Get_BiasGyr().transpose() << " / " << nsBiasest.Get_BiasGyr().transpose()
                      << std::endl;
            std::cerr << "bias acc not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias" << std::endl
                      << nsPVRest.Get_BiasAcc().transpose() << " / " << nsBiasest.Get_BiasAcc().transpose()
                      << std::endl;
        }
    }

    void EdgeNavStatePriorPVRBias::linearizeOplus() {
        // Estimated NavState
        const VertexNavStatePVR *vPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);
        const NavState &nsPVRest = vPVR->estimate();
        const NavState &nsprior = _measurement;

        _jacobianOplusXi = Matrix<double, 15, 9>::Zero();
        _jacobianOplusXi.block<3, 3>(0, 0) = (nsprior.Get_R().inverse() * nsPVRest.Get_R()).matrix();
        _jacobianOplusXi.block<3, 3>(3, 3) = Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(6, 6) = Sophus::SO3::JacobianRInv(_error.segment<3>(6));

        _jacobianOplusXj = Matrix<double, 15, 6>::Zero();
        _jacobianOplusXj.block<3, 3>(9, 0) = -Matrix3d::Identity();
        _jacobianOplusXj.block<3, 3>(12, 3) = -Matrix3d::Identity();

    }

//--------------------------------------

/**
 * @brief EdgeNavStatePrior::EdgeNavStatePrior
 */
    void EdgeNavStatePrior::computeError() {
        // Estimated NavState
        const VertexNavState *v = static_cast<const VertexNavState *>(_vertices[0]);
        const NavState &nsest = v->estimate();
        // Measurement: NavState_prior
        const NavState &nsprior = _measurement;

        // P V R bg+dbg ba+dba
        Vector15d err = Vector15d::Zero();

        // err_P = P - P_prior
        err.segment<3>(0) = nsprior.Get_P() - nsest.Get_P();
        // err_V = V - V_prior
        err.segment<3>(3) = nsprior.Get_V() - nsest.Get_V();
        // err_R = log (R * R_prior^-1)
        err.segment<3>(6) = (nsprior.Get_R().inverse() * nsest.Get_R()).log();
        // err_bg = (bg+dbg) - (bg_prior+dbg_prior)
        err.segment<3>(9) =
                (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsest.Get_BiasGyr() + nsest.Get_dBias_Gyr());
        // err_ba = (ba+dba) - (ba_prior+dba_prior)
        err.segment<3>(12) =
                (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsest.Get_BiasAcc() + nsest.Get_dBias_Acc());

        _error = err;

        //Debug log
        //std::cout<<"prior edge error: "<<std::endl<<_error.transpose()<<std::endl;
    }

    void EdgeNavStatePrior::linearizeOplus() {
        // 1.
        // increment is the same as Forster 15'RSS
        // pi = pi + Ri*dpi
        // vi = vi + dvi
        // Ri = Ri*Exp(dphi_i)
        //      Note: the optimized bias term is the 'delta bias'
        // dBgi = dBgi + dbgi_update
        // dBai = dBai + dbai_update

        // Estimated NavState
        const VertexNavState *v = static_cast<const VertexNavState *>(_vertices[0]);
        const NavState &nsest = v->estimate();

        _jacobianOplusXi.block<3, 3>(0, 0) = -nsest.Get_RotMatrix();
        _jacobianOplusXi.block<3, 3>(3, 3) = -Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(6, 6) = Sophus::SO3::JacobianRInv(_error.segment<3>(6));
        _jacobianOplusXi.block<3, 3>(9, 9) = -Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(12, 12) = -Matrix3d::Identity();
    }

/**
 * @brief VertexGravityW::VertexGravityW
 */
    VertexGravityW::VertexGravityW() : BaseVertex<2, Vector3d>() {}

    void VertexGravityW::oplusImpl(const double *update_) {
        Eigen::Map<const Vector2d> update(update_);
        Vector3d update3 = Vector3d::Zero();
        update3.head(2) = update;
        Sophus::SO3d dR = Sophus::SO3d::exp(update3);
        _estimate = dR * _estimate;
    }
    



/**
 * @brief EdgeNavStateGw::EdgeNavStateGw
 */
    EdgeNavStateGw::EdgeNavStateGw() : BaseMultiEdge<15, IMUPreintegrator>() {
        resize(3);
    }

    void EdgeNavStateGw::computeError() {
        const VertexNavState *vi = static_cast<const VertexNavState *>(_vertices[0]);
        const VertexNavState *vj = static_cast<const VertexNavState *>(_vertices[1]);
        const VertexGravityW *vg = static_cast<const VertexGravityW *>(_vertices[2]);

        Vector3d GravityVec = vg->estimate();

        // terms need to computer error in vertex i, except for bias error
        const NavState &NSi = vi->estimate();
        Vector3d Pi = NSi.Get_P();
        Vector3d Vi = NSi.Get_V();
        Sophus::SO3d Ri = NSi.Get_R();
        Vector3d dBgi = NSi.Get_dBias_Gyr();
        Vector3d dBai = NSi.Get_dBias_Acc();

        // terms need to computer error in vertex j, except for bias error
        const NavState &NSj = vj->estimate();
        Vector3d Pj = NSj.Get_P();
        Vector3d Vj = NSj.Get_V();
        Sophus::SO3d Rj = NSj.Get_R();

        // IMU Preintegration measurement
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();   // Delta Time
        double dT2 = dTij * dTij;
        Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
        Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
        Sophus::SO3d dRij = Sophus::SO3(M.getDeltaR());

        // tmp variable, transpose of Ri
        Sophus::SO3d RiT = Ri.inverse();
        // residual error of Delta Position measurement
        Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2)
                        - (dPij + M.getJPBiasg() * dBgi +
                           M.getJPBiasa() * dBai);   // this line includes correction term of bias change.
        // residual error of Delta Velocity measurement
        Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij)
                        - (dVij + M.getJVBiasg() * dBgi +
                           M.getJVBiasa() * dBai);   //this line includes correction term of bias change
        // residual error of Delta Rotation measurement
        Sophus::SO3d dR_dbg = Sophus::SO3d::exp(M.getJRBiasg() * dBgi);
        Sophus::SO3d rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
        Vector3d rPhiij = rRij.log();

        // residual error of Gyroscope's bias, Forster 15'RSS
        Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                          - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

        // residual error of Accelerometer's bias, Forster 15'RSS
        Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                          - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

        Vector15d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=15
        err.setZero();
        // 15-Dim error vector order:
        // position-velocity-rotation-deltabiasGyr_i-deltabiasAcc_i
        // rPij - rVij - rPhiij - rBiasGi - rBiasAi
        err.segment<3>(0) = rPij;       // position error
        err.segment<3>(3) = rVij;       // velocity error
        err.segment<3>(6) = rPhiij;     // rotation phi error
        err.segment<3>(9) = rBiasG;     // bias gyro error
        err.segment<3>(12) = rBiasA;    // bias acc error

        _error = err;
    }

    void EdgeNavStateGw::linearizeOplus() {
        const VertexNavState *vi = static_cast<const VertexNavState *>(_vertices[0]);
        const VertexNavState *vj = static_cast<const VertexNavState *>(_vertices[1]);
        const VertexGravityW *vg = static_cast<const VertexGravityW *>(_vertices[2]);

        Vector3d GravityVec = vg->estimate();


        // terms need to computer error in vertex i, except for bias error
        const NavState &NSi = vi->estimate();
        Vector3d Pi = NSi.Get_P();
        Vector3d Vi = NSi.Get_V();
        Matrix3d Ri = NSi.Get_RotMatrix();
        Vector3d dBgi = NSi.Get_dBias_Gyr();

        // terms need to computer error in vertex j, except for bias error
        const NavState &NSj = vj->estimate();
        Vector3d Pj = NSj.Get_P();
        Vector3d Vj = NSj.Get_V();
        Matrix3d Rj = NSj.Get_RotMatrix();

        // IMU Preintegration measurement
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();   // Delta Time
        double dT2 = dTij * dTij;

        // some temp variable
        Matrix3d I3x3 = Matrix3d::Identity();   // I_3x3
        Matrix3d O3x3 = Matrix3d::Zero();       // 0_3x3
        Matrix3d RiT = Ri.transpose();          // Ri^T
        Matrix3d RjT = Rj.transpose();          // Rj^T
        Vector3d rPhiij = _error.segment<3>(6); // residual of rotation, rPhiij
        Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
        Matrix3d J_rPhi_dbg = M.getJRBiasg();              // jacobian of preintegrated rotation-angle to gyro bias i

        // 1.
        // increment is the same as Forster 15'RSS
        // pi = pi + Ri*dpi,    pj = pj + Rj*dpj
        // vi = vi + dvi,       vj = vj + dvj
        // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
        //      Note: the optimized bias term is the 'delta bias'
        // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
        // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

        // 2.
        // 15-Dim error vector order:
        // position-velocity-rotation-deltabiasGyr_i-deltabiasAcc_i
        // rPij - rVij - rPhiij - rBiasGi - rBiasAi
        //      Jacobian row order:
        // J_rPij_xxx
        // J_rVij_xxx
        // J_rPhiij_xxx
        // J_rBiasGi_xxx
        // J_rBiasAi_xxx

        // 3.
        // order in 'update_'
        // Vertex_i : dPi, dVi, dPhi_i, dBiasGyr_i, dBiasAcc_i,
        // Vertex_j : dPj, dVj, dPhi_j, dBiasGyr_j, dBiasAcc_j
        //      Jacobian column order:
        // J_xxx_dPi, J_xxx_dVi, J_xxx_dPhi_i, J_xxx_dBiasGyr_i, J_xxx_dBiasAcc_i
        // J_xxx_dPj, J_xxx_dVj, J_xxx_dPhi_j, J_xxx_dBiasGyr_j, J_xxx_dBiasAcc_j

        // 4.
        // For Vertex_i
        Matrix<double, 15, 15> _jacobianOplusXi;
        _jacobianOplusXi.setZero();

        // 4.1
        // J_rPij_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(0, 0) = -I3x3;      //J_rP_dpi
        _jacobianOplusXi.block<3, 3>(0, 3) = -RiT * dTij;  //J_rP_dvi
        _jacobianOplusXi.block<3, 3>(0, 6) = Sophus::SO3::hat(
                RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    //J_rP_dPhi_i
        _jacobianOplusXi.block<3, 3>(0, 9) = -M.getJPBiasg();     //J_rP_dbgi
        _jacobianOplusXi.block<3, 3>(0, 12) = -M.getJPBiasa();     //J_rP_dbai

        // 4.2
        // J_rVij_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(3, 0) = O3x3;    //dpi
        _jacobianOplusXi.block<3, 3>(3, 3) = -RiT;    //dvi
        _jacobianOplusXi.block<3, 3>(3, 6) = Sophus::SO3::hat(RiT * (Vj - Vi - GravityVec * dTij));    //dphi_i
        _jacobianOplusXi.block<3, 3>(3, 9) = -M.getJVBiasg();    //dbg_i
        _jacobianOplusXi.block<3, 3>(3, 12) = -M.getJVBiasa();    //dba_i

        // 4.3
        // J_rPhiij_xxx_i for Vertex_i
        Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
        Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg * dBgi);
        _jacobianOplusXi.block<3, 3>(6, 0) = O3x3;    //dpi
        _jacobianOplusXi.block<3, 3>(6, 3) = O3x3;    //dvi
        _jacobianOplusXi.block<3, 3>(6, 6) = -JrInv_rPhi * RjT * Ri;    //dphi_i
        _jacobianOplusXi.block<3, 3>(6, 9) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
        _jacobianOplusXi.block<3, 3>(6, 12) = O3x3;    //dba_i

        // 4.4
        // J_rBiasGi_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(9, 0) = O3x3;    //dpi
        _jacobianOplusXi.block<3, 3>(9, 3) = O3x3;    //dvi
        _jacobianOplusXi.block<3, 3>(9, 6) = O3x3;    //dphi_i
        _jacobianOplusXi.block<3, 3>(9, 9) = -I3x3;    //dbg_i
        _jacobianOplusXi.block<3, 3>(9, 12) = O3x3;    //dba_i

        // 4.5
        // J_rBiasAi_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(12, 0) = O3x3;     //dpi
        _jacobianOplusXi.block<3, 3>(12, 3) = O3x3;     //dvi
        _jacobianOplusXi.block<3, 3>(12, 6) = O3x3;     //dphi_i
        _jacobianOplusXi.block<3, 3>(12, 9) = O3x3;    //dbg_i
        _jacobianOplusXi.block<3, 3>(12, 12) = -I3x3;     //dba_i

        // 5.
        // For Vertex_j
        Matrix<double, 15, 15> _jacobianOplusXj;
        _jacobianOplusXj.setZero();

        // 5.1
        // J_rPij_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(0, 0) = RiT * Rj;  //dpj
        _jacobianOplusXj.block<3, 3>(0, 3) = O3x3;    //dvj
        _jacobianOplusXj.block<3, 3>(0, 6) = O3x3;    //dphi_j
        _jacobianOplusXj.block<3, 3>(0, 9) = O3x3;    //dbg_j, all zero
        _jacobianOplusXj.block<3, 3>(0, 12) = O3x3;    //dba_j, all zero

        // 5.2
        // J_rVij_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(3, 0) = O3x3;    //dpj
        _jacobianOplusXj.block<3, 3>(3, 3) = RiT;    //dvj
        _jacobianOplusXj.block<3, 3>(3, 6) = O3x3;    //dphi_j
        _jacobianOplusXj.block<3, 3>(3, 9) = O3x3;    //dbg_j, all zero
        _jacobianOplusXj.block<3, 3>(3, 12) = O3x3;    //dba_j, all zero

        // 5.3
        // J_rPhiij_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(6, 0) = O3x3;    //dpj
        _jacobianOplusXj.block<3, 3>(6, 3) = O3x3;    //dvj
        _jacobianOplusXj.block<3, 3>(6, 6) = JrInv_rPhi;    //dphi_j
        _jacobianOplusXj.block<3, 3>(6, 9) = O3x3;    //dbg_j, all zero
        _jacobianOplusXj.block<3, 3>(6, 12) = O3x3;    //dba_j, all zero

        // 5.4
        // J_rBiasGi_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(9, 0) = O3x3;     //dpj
        _jacobianOplusXj.block<3, 3>(9, 3) = O3x3;     //dvj
        _jacobianOplusXj.block<3, 3>(9, 6) = O3x3;     //dphi_j
        _jacobianOplusXj.block<3, 3>(9, 9) = I3x3;    //dbg_j
        _jacobianOplusXj.block<3, 3>(9, 12) = O3x3;     //dba_j

        // 5.5
        // J_rBiasAi_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(12, 0) = O3x3;    //dpj
        _jacobianOplusXj.block<3, 3>(12, 3) = O3x3;    //dvj
        _jacobianOplusXj.block<3, 3>(12, 6) = O3x3;    //dphi_j
        _jacobianOplusXj.block<3, 3>(12, 9) = O3x3;    //dbg_j
        _jacobianOplusXj.block<3, 3>(12, 12) = I3x3;    //dba_j


        // Gravity in world
        Matrix<double, 15, 2> Jgw;
        Jgw.setZero();
        Matrix3d GwHat = Sophus::SO3::hat(GravityVec);

        Jgw.block<3, 2>(0, 0) = RiT * 0.5 * dT2 * GwHat.block<3, 2>(0, 0); //rPij
        Jgw.block<3, 2>(3, 0) = RiT * dTij * GwHat.block<3, 2>(0, 0); //rVij
        //Jgw.block<3,2>(3,0) = ; //rRij
        //Jgw.block<3,2>(3,0) = 0; //rBg
        //Jgw.block<3,2>(3,0) = 0; //rBa

        // evaluate
        _jacobianOplus[0] = _jacobianOplusXi;
        _jacobianOplus[1] = _jacobianOplusXj;
        _jacobianOplus[2] = Jgw;
    }


/**
 * @brief VertexNavState::VertexNavState
 */
    VertexNavState::VertexNavState() : BaseVertex<15, NavState>() {
    }

// Todo
    bool VertexNavState::read(std::istream &is) { return true; }

    bool VertexNavState::write(std::ostream &os) const { return true; }

    void VertexNavState::oplusImpl(const double *update_) {
        // 1.
        // order in 'update_'
        // dP, dV, dPhi, dBiasGyr, dBiasAcc

        // 2.
        // the same as Forster 15'RSS
        // pi = pi + Ri*dpi,    pj = pj + Rj*dpj
        // vi = vi + dvi,       vj = vj + dvj
        // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
        //      Note: the optimized bias term is the 'delta bias'
        // delta_biasg_i = delta_biasg_i + dbgi,    delta_biasg_j = delta_biasg_j + dbgj
        // delta_biasa_i = delta_biasa_i + dbai,    delta_biasa_j = delta_biasa_j + dbaj

        Eigen::Map<const Vector15d> update(update_);
        _estimate.IncSmall(update);

        //std::cout<<"id "<<id()<<" ns update: "<<update.transpose()<<std::endl;
    }

/**
 * @brief EdgeNavState::EdgeNavState
 */
    EdgeNavState::EdgeNavState() : BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState>() {
    }

// Todo
    bool EdgeNavState::read(std::istream &is) { return true; }

    bool EdgeNavState::write(std::ostream &os) const { return true; }

/**
 * @brief EdgeNavState::computeError
 * In g2o, computeError() is called in computeActiveErrors(), before buildSystem()
 */
    void EdgeNavState::computeError() {
        const VertexNavState *vi = static_cast<const VertexNavState *>(_vertices[0]);
        const VertexNavState *vj = static_cast<const VertexNavState *>(_vertices[1]);

        // terms need to computer error in vertex i, except for bias error
        const NavState &NSi = vi->estimate();
        Vector3d Pi = NSi.Get_P();
        Vector3d Vi = NSi.Get_V();
        Sophus::SO3d Ri = NSi.Get_R();
        Vector3d dBgi = NSi.Get_dBias_Gyr();
        Vector3d dBai = NSi.Get_dBias_Acc();

        // terms need to computer error in vertex j, except for bias error
        const NavState &NSj = vj->estimate();
        Vector3d Pj = NSj.Get_P();
        Vector3d Vj = NSj.Get_V();
        Sophus::SO3d Rj = NSj.Get_R();

        // IMU Preintegration measurement
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();   // Delta Time
        double dT2 = dTij * dTij;
        Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
        Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
        Sophus::SO3d dRij = Sophus::SO3d(M.getDeltaR());

        // tmp variable, transpose of Ri
        //Matrix3d RiT = Ri.transpose();
        Sophus::SO3d RiT = Ri.inverse();
        // residual error of Delta Position measurement
        Vector3d rPij = RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2)
                        - (dPij + M.getJPBiasg() * dBgi +
                           M.getJPBiasa() * dBai);   // this line includes correction term of bias change.
        // residual error of Delta Velocity measurement
        Vector3d rVij = RiT * (Vj - Vi - GravityVec * dTij)
                        - (dVij + M.getJVBiasg() * dBgi +
                           M.getJVBiasa() * dBai);   //this line includes correction term of bias change
        // residual error of Delta Rotation measurement
        Sophus::SO3d dR_dbg = Sophus::SO3d::exp(M.getJRBiasg() * dBgi);
        Sophus::SO3d rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
        Vector3d rPhiij = rRij.log();

        // residual error of Gyroscope's bias, Forster 15'RSS
        Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                          - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

        // residual error of Accelerometer's bias, Forster 15'RSS
        Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                          - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

        Vector15d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=15
        err.setZero();
        // 15-Dim error vector order:
        // position-velocity-rotation-deltabiasGyr_i-deltabiasAcc_i
        // rPij - rVij - rPhiij - rBiasGi - rBiasAi
        err.segment<3>(0) = rPij;       // position error
        err.segment<3>(3) = rVij;       // velocity error
        err.segment<3>(6) = rPhiij;     // rotation phi error
        err.segment<3>(9) = rBiasG;     // bias gyro error
        err.segment<3>(12) = rBiasA;    // bias acc error

        _error = err;
    }

/**
 * @brief EdgeNavState::linearizeOplus
 * In g2o, linearizeOplus() is called in buildSystem(), after computeError()
 * So variable #_error# is computed and can be used here
 *
 * Reference:
 * [1]  Supplementary Material to :
 *      IMU Preintegration on Manifold for E cient Visual-Inertial Maximum-a-Posteriori Estimation IMU Preintegration
 *      Technical Report GT-IRIM-CP&R-2015-001
 *      Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza
 *      May 30, 2015
 * [2]  IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation
 *      Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza
 *      RSS, 2015
 */
    void EdgeNavState::linearizeOplus() {
        const VertexNavState *vi = static_cast<const VertexNavState *>(_vertices[0]);
        const VertexNavState *vj = static_cast<const VertexNavState *>(_vertices[1]);

        // terms need to computer error in vertex i, except for bias error
        const NavState &NSi = vi->estimate();
        Vector3d Pi = NSi.Get_P();
        Vector3d Vi = NSi.Get_V();
        Matrix3d Ri = NSi.Get_RotMatrix();
        Vector3d dBgi = NSi.Get_dBias_Gyr();

        // terms need to computer error in vertex j, except for bias error
        const NavState &NSj = vj->estimate();
        Vector3d Pj = NSj.Get_P();
        Vector3d Vj = NSj.Get_V();
        Matrix3d Rj = NSj.Get_RotMatrix();

        // IMU Preintegration measurement
        const IMUPreintegrator &M = _measurement;
        double dTij = M.getDeltaTime();   // Delta Time
        double dT2 = dTij * dTij;

        // some temp variable
        Matrix3d I3x3 = Matrix3d::Identity();   // I_3x3
        Matrix3d O3x3 = Matrix3d::Zero();       // 0_3x3
        Matrix3d RiT = Ri.transpose();          // Ri^T
        Matrix3d RjT = Rj.transpose();          // Rj^T
        Vector3d rPhiij = _error.segment<3>(6); // residual of rotation, rPhiij
        Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);    // inverse right jacobian of so3 term #rPhiij#
        Matrix3d J_rPhi_dbg = M.getJRBiasg();              // jacobian of preintegrated rotation-angle to gyro bias i

        // 1.
        // increment is the same as Forster 15'RSS
        // pi = pi + Ri*dpi,    pj = pj + Rj*dpj
        // vi = vi + dvi,       vj = vj + dvj
        // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
        //      Note: the optimized bias term is the 'delta bias'
        // dBgi = dBgi + dbgi_update,    dBgj = dBgj + dbgj_update
        // dBai = dBai + dbai_update,    dBaj = dBaj + dbaj_update

        // 2.
        // 15-Dim error vector order:
        // position-velocity-rotation-deltabiasGyr_i-deltabiasAcc_i
        // rPij - rVij - rPhiij - rBiasGi - rBiasAi
        //      Jacobian row order:
        // J_rPij_xxx
        // J_rVij_xxx
        // J_rPhiij_xxx
        // J_rBiasGi_xxx
        // J_rBiasAi_xxx

        // 3.
        // order in 'update_'
        // Vertex_i : dPi, dVi, dPhi_i, dBiasGyr_i, dBiasAcc_i,
        // Vertex_j : dPj, dVj, dPhi_j, dBiasGyr_j, dBiasAcc_j
        //      Jacobian column order:
        // J_xxx_dPi, J_xxx_dVi, J_xxx_dPhi_i, J_xxx_dBiasGyr_i, J_xxx_dBiasAcc_i
        // J_xxx_dPj, J_xxx_dVj, J_xxx_dPhi_j, J_xxx_dBiasGyr_j, J_xxx_dBiasAcc_j

        // 4.
        // For Vertex_i
        _jacobianOplusXi.setZero();

        // 4.1
        // J_rPij_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(0, 0) = -I3x3;      //J_rP_dpi
        _jacobianOplusXi.block<3, 3>(0, 3) = -RiT * dTij;  //J_rP_dvi
        _jacobianOplusXi.block<3, 3>(0, 6) = Sophus::SO3::hat(
                RiT * (Pj - Pi - Vi * dTij - 0.5 * GravityVec * dT2));    //J_rP_dPhi_i
        _jacobianOplusXi.block<3, 3>(0, 9) = -M.getJPBiasg();     //J_rP_dbgi
        _jacobianOplusXi.block<3, 3>(0, 12) = -M.getJPBiasa();     //J_rP_dbai

        // 4.2
        // J_rVij_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(3, 0) = O3x3;    //dpi
        _jacobianOplusXi.block<3, 3>(3, 3) = -RiT;    //dvi
        _jacobianOplusXi.block<3, 3>(3, 6) = Sophus::SO3::hat(RiT * (Vj - Vi - GravityVec * dTij));    //dphi_i
        _jacobianOplusXi.block<3, 3>(3, 9) = -M.getJVBiasg();    //dbg_i
        _jacobianOplusXi.block<3, 3>(3, 12) = -M.getJVBiasa();    //dba_i

        // 4.3
        // J_rPhiij_xxx_i for Vertex_i
        Matrix3d ExprPhiijTrans = Sophus::SO3::exp(rPhiij).inverse().matrix();
        Matrix3d JrBiasGCorr = Sophus::SO3::JacobianR(J_rPhi_dbg * dBgi);
        _jacobianOplusXi.block<3, 3>(6, 0) = O3x3;    //dpi
        _jacobianOplusXi.block<3, 3>(6, 3) = O3x3;    //dvi
        _jacobianOplusXi.block<3, 3>(6, 6) = -JrInv_rPhi * RjT * Ri;    //dphi_i
        _jacobianOplusXi.block<3, 3>(6, 9) = -JrInv_rPhi * ExprPhiijTrans * JrBiasGCorr * J_rPhi_dbg;    //dbg_i
        _jacobianOplusXi.block<3, 3>(6, 12) = O3x3;    //dba_i

        // 4.4
        // J_rBiasGi_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(9, 0) = O3x3;    //dpi
        _jacobianOplusXi.block<3, 3>(9, 3) = O3x3;    //dvi
        _jacobianOplusXi.block<3, 3>(9, 6) = O3x3;    //dphi_i
        _jacobianOplusXi.block<3, 3>(9, 9) = -I3x3;    //dbg_i
        _jacobianOplusXi.block<3, 3>(9, 12) = O3x3;    //dba_i

        // 4.5
        // J_rBiasAi_xxx_i for Vertex_i
        _jacobianOplusXi.block<3, 3>(12, 0) = O3x3;     //dpi
        _jacobianOplusXi.block<3, 3>(12, 3) = O3x3;     //dvi
        _jacobianOplusXi.block<3, 3>(12, 6) = O3x3;     //dphi_i
        _jacobianOplusXi.block<3, 3>(12, 9) = O3x3;    //dbg_i
        _jacobianOplusXi.block<3, 3>(12, 12) = -I3x3;     //dba_i

        // 5.
        // For Vertex_j
        _jacobianOplusXj.setZero();

        // 5.1
        // J_rPij_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(0, 0) = RiT * Rj;  //dpj
        _jacobianOplusXj.block<3, 3>(0, 3) = O3x3;    //dvj
        _jacobianOplusXj.block<3, 3>(0, 6) = O3x3;    //dphi_j
        _jacobianOplusXj.block<3, 3>(0, 9) = O3x3;    //dbg_j, all zero
        _jacobianOplusXj.block<3, 3>(0, 12) = O3x3;    //dba_j, all zero

        // 5.2
        // J_rVij_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(3, 0) = O3x3;    //dpj
        _jacobianOplusXj.block<3, 3>(3, 3) = RiT;    //dvj
        _jacobianOplusXj.block<3, 3>(3, 6) = O3x3;    //dphi_j
        _jacobianOplusXj.block<3, 3>(3, 9) = O3x3;    //dbg_j, all zero
        _jacobianOplusXj.block<3, 3>(3, 12) = O3x3;    //dba_j, all zero

        // 5.3
        // J_rPhiij_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(6, 0) = O3x3;    //dpj
        _jacobianOplusXj.block<3, 3>(6, 3) = O3x3;    //dvj
        _jacobianOplusXj.block<3, 3>(6, 6) = JrInv_rPhi;    //dphi_j
        _jacobianOplusXj.block<3, 3>(6, 9) = O3x3;    //dbg_j, all zero
        _jacobianOplusXj.block<3, 3>(6, 12) = O3x3;    //dba_j, all zero

        // 5.4
        // J_rBiasGi_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(9, 0) = O3x3;     //dpj
        _jacobianOplusXj.block<3, 3>(9, 3) = O3x3;     //dvj
        _jacobianOplusXj.block<3, 3>(9, 6) = O3x3;     //dphi_j
        _jacobianOplusXj.block<3, 3>(9, 9) = I3x3;    //dbg_j
        _jacobianOplusXj.block<3, 3>(9, 12) = O3x3;     //dba_j

        // 5.5
        // J_rBiasAi_xxx_j for Vertex_j
        _jacobianOplusXj.block<3, 3>(12, 0) = O3x3;    //dpj
        _jacobianOplusXj.block<3, 3>(12, 3) = O3x3;    //dvj
        _jacobianOplusXj.block<3, 3>(12, 6) = O3x3;    //dphi_j
        _jacobianOplusXj.block<3, 3>(12, 9) = O3x3;    //dbg_j
        _jacobianOplusXj.block<3, 3>(12, 12) = I3x3;    //dba_j

    }

/**
 * @brief EdgeNavStatePointXYZ::EdgeNavStatePointXYZ
 */
    EdgeNavStatePointXYZ::EdgeNavStatePointXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavState>() {
    }

// Todo
    bool EdgeNavStatePointXYZ::read(std::istream &is) { return true; }

    bool EdgeNavStatePointXYZ::write(std::ostream &os) const { return true; }

    void EdgeNavStatePointXYZ::linearizeOplus() {
        const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
        const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[1]);

        const NavState &ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d &Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // Jacobian of camera projection
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - pi( Pc )
        // Pw <- Pw + dPw,          for Point3D
        // Rwb <- Rwb*exp(dtheta),  for NavState.R
        // Pwb <- Pwb + Rwb*dPwb,   for NavState.P

        // Jacobian of error w.r.t Pw
        _jacobianOplusXi = -Jpi * Rcb * Rwb.transpose();

        // Jacobian of Pc/error w.r.t dPwb
        //Matrix3d J_Pc_dPwb = -Rcb;
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb);
        // Jacobian of Pc/error w.r.t dRwb
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (SO3::hat(Paux) * Rcb);

        // Jacobian of Pc w.r.t NavState
        // order in 'update_': dP, dV, dPhi, dBiasGyr, dBiasAcc
        Matrix<double, 2, 15> JNavState = Matrix<double, 2, 15>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        // Jacobian of error w.r.t NavState
        _jacobianOplusXj = JNavState;
    }

    void EdgeNavStatePointXYZOnlyPose::linearizeOplus() {
        const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[0]);

        const NavState &ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        //const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];

        // Jacobian of camera projection
        Matrix<double, 2, 3> Maux;
        Maux.setZero();
        Maux(0, 0) = fx;
        Maux(0, 1) = 0;
        Maux(0, 2) = -x / z * fx;
        Maux(1, 0) = 0;
        Maux(1, 1) = fy;
        Maux(1, 2) = -y / z * fy;
        Matrix<double, 2, 3> Jpi = Maux / z;

        // error = obs - pi( Pc )
        // Pw <- Pw + dPw,          for Point3D
        // Rwb <- Rwb*exp(dtheta),  for NavState.R
        // Pwb <- Pwb + Rwb*dPwb,   for NavState.P

        // Jacobian of Pc/error w.r.t dPwb
        Matrix<double, 2, 3> JdPwb = -Jpi * (-Rcb);
        Vector3d Paux = Rcb * Rwb.transpose() * (Pw - Pwb);
        Matrix<double, 2, 3> JdRwb = -Jpi * (Sophus::SO3::hat(Paux) * Rcb);

        // Jacobian of Pc w.r.t NavState
        // order in 'update_': dP, dV, dPhi, dBiasGyr, dBiasAcc
        Matrix<double, 2, 15> JNavState = Matrix<double, 2, 15>::Zero();
        JNavState.block<2, 3>(0, 0) = JdPwb;
        JNavState.block<2, 3>(0, 6) = JdRwb;

        _jacobianOplusXi = JNavState;
    }

/**
 * @brief VertexGyrBias::VertexGyrBias
 */
    VertexGyrBias::VertexGyrBias() : BaseVertex<3, Vector3d>() {
    }

    bool VertexGyrBias::read(std::istream &is) {
        Vector3d est;
        for (int i = 0; i < 3; i++)
            is >> est[i];
        setEstimate(est);
        return true;
    }

    bool VertexGyrBias::write(std::ostream &os) const {
        Vector3d est(estimate());
        for (int i = 0; i < 3; i++)
            os << est[i] << " ";
        return os.good();
    }

    void VertexGyrBias::oplusImpl(const double *update_) {
        Eigen::Map<const Vector3d> update(update_);
        _estimate += update;
    }

/**
 * @brief EdgeGyrBias::EdgeGyrBias
 */
    EdgeGyrBias::EdgeGyrBias() : BaseUnaryEdge<3, Vector3d, VertexGyrBias>() {
    }


    bool EdgeGyrBias::read(std::istream &is) {
        return true;
    }

    bool EdgeGyrBias::write(std::ostream &os) const {
        return true;
    }

    void EdgeGyrBias::computeError() {
        const VertexGyrBias *v = static_cast<const VertexGyrBias *>(_vertices[0]);
        Vector3d bg = v->estimate();
        Matrix3d dRbg = Sophus::SO3::exp(J_dR_bg * bg).matrix();
        Sophus::SO3d errR((dRbij * dRbg).transpose() * Rwbi.transpose() * Rwbj); // dRij^T * Riw * Rwj
        _error = errR.log();
    }


    void EdgeGyrBias::linearizeOplus() {
        Sophus::SO3d errR(dRbij.transpose() * Rwbi.transpose() * Rwbj); // dRij^T * Riw * Rwj
        Matrix3d Jlinv = Sophus::SO3::JacobianLInv(errR.log());
        _jacobianOplusXi = -Jlinv * J_dR_bg;
    }
    
    


// void EdgeNavStateOnlyPose::setTlb(SE3d& Tlb_)
// {
//   Tlb = Tlb_;
// 
// }

/*
bool EdgeNavStateOnlyPose::read(std::istream& is)
{
    return true;
}

bool EdgeNavStateOnlyPose::write(std::ostream& os) const
{
  return os.good();
}*/

EdgeNavStateICP::EdgeNavStateICP():BaseBinaryEdge<6, SE3d, VertexNavStatePVR, VertexNavStatePVR>()
{
  
}

void EdgeNavStateICP::computeError()
{
    VertexNavStatePVR *from = static_cast<VertexNavStatePVR*>(_vertices[0]);
    VertexNavStatePVR *to   = static_cast<VertexNavStatePVR*>(_vertices[1]);
    
    SE3d from_se3, to_se3;
    from_se3.setRotationMatrix(from->estimate().Get_RotMatrix());
    from_se3.translation() = from->estimate().Get_P();
    from_se3 = from_se3 ;
    to_se3.setRotationMatrix(to->estimate().Get_RotMatrix());
    to_se3.translation() = to->estimate().Get_P();
    to_se3 = to_se3 ;
    
//     std::cerr<<(from_se3.inverse()* to_se3).matrix()<<std::endl;
    SE3d delta=_inverseMeasurement * (from_se3.inverse()* to_se3);
//     Matrix3d dRij = _measurement.rotationMatrix();
//     std::cout << delta.log().transpose() << std::endl;
    Vector3d rPhiij = SO3d(delta.rotationMatrix()).log();
    Vector3d rPij = delta.translation();
    Vector6d err;
    err.setZero();
    err.segment<3>(0) = rPij;
    err.segment<3>(3) = rPhiij;
//     std::cout << err.transpose() << std::endl;
    _error = err;
}


// void EdgeNavStateICP::linearizeOplus()
// {
//     VertexNavStatePVR *from = static_cast<VertexNavStatePVR*>(_vertices[0]);
//     VertexNavStatePVR *to   = static_cast<VertexNavStatePVR*>(_vertices[1]);
//     
//     Matrix3d Ri = from->estimate().Get_RotMatrix();
//     Matrix3d RiT = Ri.transpose();  
//     Vector3d Pi = from->estimate().Get_P();
//     
//     Matrix3d Rj = to->estimate().Get_RotMatrix();
//     Matrix3d RjT = Rj.transpose();
//     Vector3d Pj = to->estimate().Get_P();
//     
//     Vector3d rPhiij = _error.segment<3>(3);
//     Matrix3d JrInv_rPhi = Sophus::SO3::JacobianRInv(rPhiij);
//     
// //     std::cerr<<JrInv_rPhi<<std::endl;
//     Matrix3d measurement_R  = _inverseMeasurement.rotationMatrix();
//     
//     Matrix3d I3x3 = Matrix3d::Identity();
//     
//     _jacobianOplusXi.setZero();
//     _jacobianOplusXi.block<3, 3>(0, 0) = -measurement_R * RiT;
//     _jacobianOplusXi.block<3, 3>(0, 6) = measurement_R * RiT * Sophus::SO3::hat(Pj-Pi) * Ri;
//     
// //     std::cerr<<-measurement_R * RiT<<endl<<(measurement_R * RiT * Sophus::SO3::hat(Pj-Pi) * Ri)<<std::endl;
//     
//     _jacobianOplusXi.block<3, 3>(3, 6) = -JrInv_rPhi * RjT * Ri;
//     
// //     std::cerr<<-JrInv_rPhi * RjT * Ri<<std::endl;
//     _jacobianOplusXj.setZero();
//     _jacobianOplusXj.block<3,3>(0,0) = measurement_R * RiT;
// //     std::cerr<<measurement_R * RiT<<std::endl;
//     _jacobianOplusXj.block<3,3>(3,6) = JrInv_rPhi;
// //     std::cerr<<JrInv_rPhi<<std::endl;
//     
//     
//     
// }



void EdgeNavStateOnlyPose::cmputeError()
{
    VertexNavStatePVR *from = static_cast<VertexNavStatePVR*>(_vertices[0]);
    VertexNavStatePVR *to   = static_cast<VertexNavStatePVR*>(_vertices[1]);
    
    SE3d from_se3, to_se3;
    from_se3.setRotationMatrix(from->estimate().Get_RotMatrix());
    from_se3.translation() = from->estimate().Get_P();
    from_se3 = from_se3 * Tlb;
    to_se3.setRotationMatrix(to->estimate().Get_RotMatrix());
    to_se3.translation() = to->estimate().Get_P();
    to_se3 = to_se3 * Tlb;
    SE3d delta=_inverseMeasurement * (from_se3.inverse()* to_se3);
//     cout << delta.log().transpose() << endl;
    _error= delta.log();
}

VertexSO3PitchRoll::VertexSO3PitchRoll() : BaseVertex<2, Vector2d>() {
}
    /**
     * @brief VertexCoarseInitialization pitch and roll
     * yaw pitch roll  Z  Y  X
     */
    
    
void VertexSO3PitchRoll::oplusImpl(const double* update_)
{
  Eigen::Map<const Vector2d> update(update_);
//   Vector2d update3 = Vector2d::Zero();
  _estimate = _estimate + update;
  
  while(_estimate[0] > 3.1416){
    _estimate[0] = _estimate[0] - 2 * 3.1416;
  }
  
  while(_estimate[1] > 3.1416){
    _estimate[1] = _estimate[1] - 2 * 3.1416;
  }
  
  while(_estimate[0] < -3.1416){
    _estimate[0] = _estimate[0] + 2 * 3.1416;
  }
  
  while(_estimate[1] < -3.1416){
    _estimate[1] = _estimate[1] + 2 * 3.1416;
  }
   
}





// VertexBa::VertexBa() : BaseVertex<3, Vector3d>() {
// }

void VertexBa::oplusImpl(const double* update_)
{
  Eigen::Map<const Vector3d> update(update_);
  _estimate = _estimate + update;

}


EdgeCoarsePitchRollBa::EdgeCoarsePitchRollBa() : BaseBinaryEdge<3, Vector3d, VertexSO3PitchRoll, VertexBa>()
{
	gn = Vector3d();
	gn(0) = 0;
	gn(1) = 0;
	gn(2) = 9.81;
}




void EdgeCoarsePitchRollBa::computeError()
{
  VertexSO3PitchRoll *pitchroll = static_cast<VertexSO3PitchRoll*>(_vertices[0]);
  VertexBa *ba = static_cast<VertexBa*>(_vertices[1]);

  Matrix3d rotation = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) 
		    * Eigen::AngleAxisd(pitchroll->estimate()[0], Eigen::Vector3d::UnitY())
		    * Eigen::AngleAxisd(pitchroll->estimate()[1], Eigen::Vector3d::UnitX())).toRotationMatrix();
// 		    
//  Matrix3d rotation = fromEuler(Vector3d(pitchroll->estimate()[0], pitchroll->estimate()[1],0)); //roll pitch yaw
  
  Vector3d estimate = rotation * gn + ba->estimate();
  
//   std::cout<<rotation<<std::endl<<gn.transpose()<<std::endl;
//   std::cout<<_measurement.transpose()<<" | "<<estimate.transpose() <<" | " << (rotation * gn).transpose()<<" | "<<ba->estimate().transpose()<<std::endl;
  _error = _measurement - estimate;
}

EdgeBiasa::EdgeBiasa()
{
  priori = Vector3d(0,0,0);
}


void EdgeBiasa::computeError()
{
  VertexBa*  vba = static_cast<VertexBa*>(_vertices[0]);
  
  _error = priori - vba->estimate() ;

}


// bool EdgeNavStateOnlyPose::setMeasurementFromState()
// {
//     VertexNavStatePVR *from = static_cast<VertexNavStatePVR*>(_vertices[0]);
//     VertexNavStatePVR *to   = static_cast<VertexNavStatePVR*>(_vertices[1]);
//     SE3d from_se3, to_se3;
//     from_se3.setRotationMatrix(from->estimate().Get_RotMatrix());
//     from_se3.translation() = from->estimate().Get_P();
//     from_se3 = from_se3 * Tlb;
//     to_se3.setRotationMatrix(to->estimate().Get_RotMatrix());
//     to_se3.translation() = to->estimate().Get_P();
//     to_se3 = to_se3 * Tlb;
//     SE3d delta = from_se3.inverse() * to_se3;
//     
//     setMeasurement(delta);
//     return true;
// }

// void EdgeNavStateOnlyPose::initialEstimate(const HyperGraph::VertexSet& from, OptimizableGraph::Vertex* to)
// {
// //     VertexNavStatePVR *from = static_cast<VertexNavStatePVR*>(_vertices[0]);
// //     VertexNavStatePVR *to   = static_cast<VertexNavStatePVR*>(_vertices[1]);
// // 
// //     SE3d from_se3, to_se3;
// //     from_se3.setRotationMatrix(from->estimate().Get_RotMatrix());
// //     from_se3.translation() = from->estimate().Get_P();
// //     to_se3.setRotationMatrix(to->estimate().Get_RotMatrix());
// //     to_se3.translation() = to->estimate().Get_P();
// //     
// //     NavState from_nav;
// //     if (from.count(from) > 0) {
// //       to->setEstimate(from_se3 * _measurement);
// //     } else
// //       from->setEstimate(to_se3 * _measurement.inverse());
//     
//     
// }

// void EdgeNavStateOnlyPose::linearizeOplus()
// {
// //     VertexNavStatePVR *from = static_cast<VertexNavStatePVR*>(_vertices[0]);
// //     VertexNavStatePVR *to   = static_cast<VertexNavStatePVR*>(_vertices[1]);
// //     
// //    SE3d E;
// //    const SE3d& Xi = from->estimate();
// //    const SE3d& Xj = to->estimate();
// //    const SE3d& Z = _measurement;
// //    
//    
// }


}
