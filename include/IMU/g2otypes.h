#ifndef G2OTYPES_H_
#define G2OTYPES_H_

#include "IMU/NavState.h"
#include "IMUPreintegrator.h"

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
// #include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_cholmod.h"

using namespace vill;

#define TEST_STEREO
// #define TEST_EXTRINSIC_CALIB
// #define TEST_STEREO_CALIB

// the vertecies and edges used when doing BA with IMU measurements

namespace g2o {

    ////////////////////////////////////////////////////////////////////////////////////////
    // pang pang optimization edges
    class EdgeXYZLaser: public BaseUnaryEdge<3, Vector3d, VertexSBAPointXYZ>{
    public:
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    EdgeXYZLaser(){}
	    bool read(std::istream& is);
	    bool write(std::ostream& os) const;
	    
	    void computeError()  {
		    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
		    Vector3d obs(_measurement);
		    _error = obs-v1->estimate();
	    }

    // 	virtual void linearizeOplus();
    };
    
//     class EdgePoseSE3: public BaseUnaryEdge<6, NavState, VertexNavStatePVR>{
//     public:
// 	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 	    EdgePoseSE3(){}
// 	    bool read(std::istream& is);
// 	    bool write(std::ostream& os) const;
// 	    
// 	  virtual void setMeasurement(const NavState& m){
// 	    _measurement = m;
//     //         _inverseMeasurement = m.inverse();
// 	  }
// 	    
// 	    void computeError()  {
// 		    VertexNavStatePVR *from = static_cast<VertexNavStatePVR*>(_vertices[0]);
//     // 		Isometry3D delta=_inverseMeasurement * from->estimate().inverse() * to->estimate();
// 		    NavState delta =  _measurement * from->estimate().inverse();
// 		    Vector6d v;
// // 		    Eigen::Quaterniond r = delta.Get_R();
// 		    SO3d so3d = delta.Get_R();
// 		    Eigen::Quaterniond r = so3d.unit_quaternion();
// 		    r.normalize();
// 		    if(r.w()<0){
// 			    r.coeffs() *= -1;
// 		    }
// 		    v.block<3,1>(3,0) = r.coeffs().head<3>();
// 		    v.block<3,1>(0,0) = delta.Get_P();
// 		    
// 		    _error = v;
// 		    
// 	    }
// 	    
// 	    void linearizeOplus();
// 	    
//     // protected:
//     // //       VertexSE3Expmap _inverseMeasurement;
// 
//     // 	virtual void linearizeOplus();
//     };
    
    class EdgePointToPlain: public BaseUnaryEdge<6,Vector6d, VertexSBAPointXYZ>{//前三个是点，后三个是点的法向量
    public:
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    EdgePointToPlain(){}
	    bool read(std::istream& is);
	    bool write(std::ostream& os) const;
	    
	    void computeError()  {
		    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
		    Vector3d b = v1->estimate();
		    Vector3d a;	a<<_measurement(0),_measurement(1),_measurement(2);
		    Vector3d normal;	normal<<_measurement(3), _measurement(4), _measurement(5);
		    Vector3d ab = b-a;
		    Vector3d pb = normal * (ab(0)*normal(0) + ab(1)*normal(1) +ab(2)*normal(2));
		    _error << pb(0), pb(1), pb(2),0 ,0 ,0;
	    }
    };
    
    
    ////////////////////////////////////////////////////////////////////////////////////////
  
    /**
     * @brief The VertexNavStatePVR class
     */
    class VertexNavStatePVR : public BaseVertex<9, NavState> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexNavStatePVR() : BaseVertex<9, NavState>() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        virtual void setToOriginImpl() {
            _estimate = NavState();
        }

        virtual void oplusImpl(const double *update_) {
            Eigen::Map<const Vector9d> update(update_);
            _estimate.IncSmallPVR(update);
        }

    };

    class VertexNavStateBias : public BaseVertex<6, NavState> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexNavStateBias() : BaseVertex<6, NavState>() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        virtual void setToOriginImpl() {
            _estimate = NavState();
        }

        virtual void oplusImpl(const double *update_) {
            Eigen::Map<const Vector6d> update(update_);
            _estimate.IncSmallBias(update);
        }

    };
    
    class VertexExCalib : public BaseVertex<6, SE3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexExCalib() : BaseVertex<6, SE3d>() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        virtual void setToOriginImpl() {
            _estimate = SE3d();
        }

        virtual void oplusImpl(const double *update_) {
            Eigen::Map<const Vector6d> update(update_);
//             _estimate.IncSmallBias(update);
	    _estimate *= Sophus::SE3d::exp(update);
        }
        
    };

    class EdgeNavStatePVR : public BaseMultiEdge<9, IMUPreintegrator> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStatePVR() : BaseMultiEdge<9, IMUPreintegrator>() {
            resize(3);
        }

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError();

        virtual void linearizeOplus();

        void SetParams(const Vector3d &gw) {
            GravityVec = gw;
        }

    protected:
        // Gravity vector in 'world' frame
        Vector3d GravityVec;
    };

    // mono extrinsic calibration edges
    class EdgeExtrinsicCalibration : public BaseMultiEdge<2, Vector2d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeExtrinsicCalibration() : BaseMultiEdge<2, Vector2d>() {resize(3);}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError() {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);

            _error = obs - cam_project(Pc);
        }

        bool isDepthPositive() {
            Vector3d Pc = computePc();
            return Pc(2) > 0.0;
        }

        Vector3d computePc() {
            const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            const VertexNavStatePVR *vNavState = static_cast<const VertexNavStatePVR *>(_vertices[1]);
	    const VertexExCalib *vExCalib = static_cast<const VertexExCalib *>(_vertices[2]);
	    
	    SE3d Tbc = vExCalib->estimate();
	    Matrix3d Rbc = Tbc.rotationMatrix();
	    Vector3d Pbc = Tbc.translation();

            const NavState &ns = vNavState->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            const Vector3d &Pw = vPoint->estimate();

            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

            return Pc;
        }

        inline Vector2d project2d(const Vector3d &v) const {
            Vector2d res;
            res(0) = v(0) / v(2);
            res(1) = v(1) / v(2);
            return res;
        }

        Vector2d cam_project(const Vector3d &trans_xyz) const {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0] * fx + cx;
            res[1] = proj[1] * fy + cy;
            return res;
        }

        //TODO using numerical method, need to change to closed form.
//         virtual void linearizeOplus();

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
        }

    protected:
        // Camera intrinsics
        double fx, fy, cx, cy;
    };
    
    // stereo extrinsic calibration edges
    class EdgeStereoExtrinsicCalibration : public BaseMultiEdge<3, Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeStereoExtrinsicCalibration() : BaseMultiEdge<3, Vector3d>() {resize(3);}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError() {
            Vector3d Pc = computePc();
            Vector3d obs(_measurement);

            _error = obs - cam_project(Pc,bf);
        }

        bool isDepthPositive() {
            Vector3d Pc = computePc();
            return Pc(2) > 0.0;
        }

        Vector3d computePc() {
            const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            const VertexNavStatePVR *vNavState = static_cast<const VertexNavStatePVR *>(_vertices[1]);
	    const VertexExCalib *vExCalib = static_cast<const VertexExCalib *>(_vertices[2]);
	    
	    SE3d Tbc = vExCalib->estimate();
	    Matrix3d Rbc = Tbc.rotationMatrix();
	    Vector3d Pbc = Tbc.translation();

            const NavState &ns = vNavState->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            const Vector3d &Pw = vPoint->estimate();

            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

            return Pc;
        }

        inline Vector2d project2d(const Vector3d &v) const {
            Vector2d res;
            res(0) = v(0) / v(2);
            res(1) = v(1) / v(2);
            return res;
        }

        Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const{
	  const float invz = 1.0f/trans_xyz[2];
	  Vector3d res;
	  res[0] = trans_xyz[0]*invz*fx + cx;
	  res[1] = trans_xyz[1]*invz*fy + cy;
	  res[2] = res[0] - bf*invz;
	  return res;
	}

        //TODO using numerical method, need to change to closed form.
//         virtual void linearizeOplus();

        void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,const double &bf_/*,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_*/) {
	    fx = fx_;
	    fy = fy_;
	    cx = cx_;
	    cy = cy_;
	    bf = bf_;
// 	    Rbc = Rbc_;
// 	    Pbc = Pbc_;
	}

protected:
	// Camera intrinsics
	double fx, fy, cx, cy,bf;
// 	// Camera-IMU extrinsics
// 	Matrix3d Rbc;
// 	Vector3d Pbc;
    };
    
    class EdgeNavStateBias : public BaseBinaryEdge<6, IMUPreintegrator, VertexNavStateBias, VertexNavStateBias> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStateBias() : BaseBinaryEdge<6, IMUPreintegrator, VertexNavStateBias, VertexNavStateBias>() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError();

        virtual void linearizeOplus();

    };
    


#ifdef TEST_STEREO
class EdgeStereoNavStatePVRPointXYZ : public BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexNavStatePVR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoNavStatePVRPointXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexNavStatePVR>() {}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    void computeError() {
        Vector3d Pc = computePc();
        Vector3d obs(_measurement);

        _error = obs - cam_project(Pc,bf);//
    }

    bool isDepthPositive() {
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc() {
        const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexNavStatePVR* vNavState = static_cast<const VertexNavStatePVR*>(_vertices[1]);

        const NavState& ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        return Pc;
        //Vector3d Pwc = Rwb*Pbc + Pwb;
        //Matrix3d Rcw = (Rwb*Rbc).transpose();
        //Vector3d Pcw = -Rcw*Pwc;
        //Vector3d Pc = Rcw*Pw + Pcw;
    }
    inline Vector2d project2d(const Vector3d& v) const {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
    // Vector2d cam_project(const Vector3d & trans_xyz) const {
    //     Vector2d proj = project2d(trans_xyz);
    //     Vector2d res;
    //     res[0] = proj[0]*fx + cx;
    //     res[1] = proj[1]*fy + cy;
    //     return res;
    // }

Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}

    //
    virtual void linearizeOplus();
//TODO   新加了bf，所以用该该函数的地方需要调整
    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,const double &bf_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        bf = bf_;
        Rbc = Rbc_;
        Pbc = Pbc_;
    }

protected:
    // Camera intrinsics
    double fx, fy, cx, cy,bf;
    // Camera-IMU extrinsics
    Matrix3d Rbc;
    Vector3d Pbc;
};


class EdgePrioriNavStatePVRBias: public BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
	EdgePrioriNavStatePVRBias() : BaseBinaryEdge< 15, NavState, VertexNavStatePVR, VertexNavStateBias>(){};
	
    bool read(std::istream& is){return true;}

    bool write(std::ostream& os) const{return true;}
    
    void computeError();
	
	virtual void linearizeOplus();
};

class EdgeStereoNavStatePVRPointXYZOnlyPose : public BaseUnaryEdge<3, Vector3d, VertexNavStatePVR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //EdgeStereoNavStatePVRPointXYZOnlyPose(){}
    EdgeStereoNavStatePVRPointXYZOnlyPose() : BaseUnaryEdge<3, Vector3d, VertexNavStatePVR>(){};

    bool read(std::istream& is){return true;}

    bool write(std::ostream& os) const{return true;}

    void computeError() {
        Vector3d Pc = computePc();
        Vector3d obs(_measurement);

        _error = obs - cam_project(Pc,bf);
    }

    bool isDepthPositive() {
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc() {
        const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);

        const NavState& ns = vNSPVR->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        //const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbc.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

        return Pc;
        //Vector3d Pwc = Rwb*Pbc + Pwb;
        //Matrix3d Rcw = (Rwb*Rbc).transpose();
        //Vector3d Pcw = -Rcw*Pwc;
        //Vector3d Pc = Rcw*Pw + Pcw;
    }


    inline Vector2d project2d(const Vector3d& v) const {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}

    //
    virtual void linearizeOplus();
//TODO  这里加了bf
    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,const double &bf_,
                   const Matrix3d& Rbc_, const Vector3d& Pbc_, const Vector3d& Pw_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
        bf = bf_;
        Rbc = Rbc_;
        Pbc = Pbc_;
        Pw = Pw_;
    }

protected:
    // Camera intrinsics
    double fx, fy, cx, cy, bf;
    // Camera-IMU extrinsics
    Matrix3d Rbc;
    Vector3d Pbc;
    // Point position in world frame
    Vector3d Pw;
};
#endif
    
    class EdgeNavStatePVRPointXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePVR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStatePVRPointXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavStatePVR>() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError() {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);

            _error = obs - cam_project(Pc);
        }

        bool isDepthPositive() {
            Vector3d Pc = computePc();
            return Pc(2) > 0.0;
        }

        Vector3d computePc() {
            const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            const VertexNavStatePVR *vNavState = static_cast<const VertexNavStatePVR *>(_vertices[1]);

            const NavState &ns = vNavState->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            const Vector3d &Pw = vPoint->estimate();

            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

            return Pc;
        }

        inline Vector2d project2d(const Vector3d &v) const {
            Vector2d res;
            res(0) = v(0) / v(2);
            res(1) = v(1) / v(2);
            return res;
        }

        Vector2d cam_project(const Vector3d &trans_xyz) const {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0] * fx + cx;
            res[1] = proj[1] * fy + cy;
            return res;
        }

        //
        virtual void linearizeOplus();

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const Matrix3d &Rbc_, const Vector3d &Pbc_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_;
            Pbc = Pbc_;
        }

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const SO3d &Rbc_, const Vector3d &Pbc_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_.matrix();
            Pbc = Pbc_;
        }

    protected:
        // Camera intrinsics
        double fx, fy, cx, cy;
        // Camera-IMU extrinsics
        Matrix3d Rbc;
        Vector3d Pbc;
    };

    class EdgeNavStatePVRPointXYZOnlyPose : public BaseUnaryEdge<2, Vector2d, VertexNavStatePVR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStatePVRPointXYZOnlyPose() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError() {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);

            _error = obs - cam_project(Pc);
        }

        bool isDepthPositive() {
            Vector3d Pc = computePc();
            return Pc(2) > 0.0;
        }

        Vector3d computePc() {
            const VertexNavStatePVR *vNSPVR = static_cast<const VertexNavStatePVR *>(_vertices[0]);

            const NavState &ns = vNSPVR->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            //const Vector3d& Pw = vPoint->estimate();

            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

            return Pc;
        }

        inline Vector2d project2d(const Vector3d &v) const {
            Vector2d res;
            res(0) = v(0) / v(2);
            res(1) = v(1) / v(2);
            return res;
        }

        Vector2d cam_project(const Vector3d &trans_xyz) const {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0] * fx + cx;
            res[1] = proj[1] * fy + cy;
            return res;
        }

        virtual void linearizeOplus();

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const Matrix3d &Rbc_, const Vector3d &Pbc_, const Vector3d &Pw_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_;
            Pbc = Pbc_;
            Pw = Pw_;
        }

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const SO3d &Rbc_, const Vector3d &Pbc_, const Vector3d &Pw_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_.matrix();
            Pbc = Pbc_;
            Pw = Pw_;
        }

    protected:
        // Camera intrinsics
        double fx, fy, cx, cy;
        // Camera-IMU extrinsics
        Matrix3d Rbc;
        Vector3d Pbc;
        // Point position in world frame
        Vector3d Pw;
    };

    /**
     * @brief The EdgeNavStatePrior class
     */
    class EdgeNavStatePriorPVRBias : public BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStatePriorPVRBias() : BaseBinaryEdge<15, NavState, VertexNavStatePVR, VertexNavStateBias>() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError();

        virtual void linearizeOplus();

    };

    /**
     * @brief The VertexNavState class
     * Vertex of tightly-coupled Visual-Inertial optimization
     */
    class VertexNavState : public BaseVertex<15, NavState> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexNavState();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        virtual void setToOriginImpl() {
            _estimate = NavState();
        }

        virtual void oplusImpl(const double *update_);
    };

    /**
     * @brief The EdgeNavStatePrior class
     */
    class EdgeNavStatePrior : public BaseUnaryEdge<15, NavState, VertexNavState> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStatePrior() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError();

        virtual void linearizeOplus();

    };

    /**
     * @brief The VertexGravityW class
     */
    class VertexGravityW : public BaseVertex<2, Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexGravityW();

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        virtual void setToOriginImpl() {
            _estimate = Vector3d(0, 0, 9.81);
        }

        virtual void oplusImpl(const double *update_);
    };

    /**
     * @brief The EdgeNavStateGw class
     */
    class EdgeNavStateGw : public BaseMultiEdge<15, IMUPreintegrator> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStateGw();

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError();

        virtual void linearizeOplus();
    };

    /**
     * @brief The EdgeNavState class
     * Edge between NavState_i and NavState_j, vertex[0]~i, vertex[1]~j
     * Measurement~Vector15d: 9Dof-IMUPreintegrator measurement & 6Dof-IMU bias change all Zero
     */
    class EdgeNavState : public BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavState();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError();

        virtual void linearizeOplus();

        void SetParams(const Vector3d &gw) {
            GravityVec = gw;
        }

    protected:
        // Gravity vector in 'world' frame
        Vector3d GravityVec;
    };

    /**
     * @brief The EdgeNavStatePointXYZ class
     * Edge between NavState and Point3D, vertex[0]~Point3D, vertex[1]~NavState
     * Measurement~Vector2f: 2Dof image feature position
     */
    class EdgeNavStatePointXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavState> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStatePointXYZ();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError() {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);

            _error = obs - cam_project(Pc);
        }

        bool isDepthPositive() {
            Vector3d Pc = computePc();
            return Pc(2) > 0.0;
        }

        Vector3d computePc() {
            const VertexSBAPointXYZ *vPoint = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
            const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[1]);

            const NavState &ns = vNavState->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            const Vector3d &Pw = vPoint->estimate();

            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

            return Pc;
        }

        inline Vector2d project2d(const Vector3d &v) const {
            Vector2d res;
            res(0) = v(0) / v(2);
            res(1) = v(1) / v(2);
            return res;
        }

        Vector2d cam_project(const Vector3d &trans_xyz) const {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0] * fx + cx;
            res[1] = proj[1] * fy + cy;
            return res;
        }

        //
        virtual void linearizeOplus();

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const Matrix3d &Rbc_, const Vector3d &Pbc_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_;
            Pbc = Pbc_;
        }

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const SO3d &Rbc_, const Vector3d &Pbc_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_.matrix();
            Pbc = Pbc_;
        }

    protected:
        // Camera intrinsics
        double fx, fy, cx, cy;
        // Camera-IMU extrinsics
        Matrix3d Rbc;
        Vector3d Pbc;
    };

    class EdgeNavStatePointXYZOnlyPose : public BaseUnaryEdge<2, Vector2d, VertexNavState> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStatePointXYZOnlyPose() {}

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        void computeError() {
            Vector3d Pc = computePc();
            Vector2d obs(_measurement);

            _error = obs - cam_project(Pc);
        }

        bool isDepthPositive() {
            Vector3d Pc = computePc();
            return Pc(2) > 0.0;
        }

        Vector3d computePc() {
            const VertexNavState *vNavState = static_cast<const VertexNavState *>(_vertices[0]);

            const NavState &ns = vNavState->estimate();
            Matrix3d Rwb = ns.Get_RotMatrix();
            Vector3d Pwb = ns.Get_P();
            Matrix3d Rcb = Rbc.transpose();
            Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbc;

            return Pc;
        }

        inline Vector2d project2d(const Vector3d &v) const {
            Vector2d res;
            res(0) = v(0) / v(2);
            res(1) = v(1) / v(2);
            return res;
        }

        Vector2d cam_project(const Vector3d &trans_xyz) const {
            Vector2d proj = project2d(trans_xyz);
            Vector2d res;
            res[0] = proj[0] * fx + cx;
            res[1] = proj[1] * fy + cy;
            return res;
        }

        //
        virtual void linearizeOplus();

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const Matrix3d &Rbc_, const Vector3d &Pbc_, const Vector3d &Pw_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_;
            Pbc = Pbc_;
            Pw = Pw_;
        }

        void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_,
                       const SO3d &Rbc_, const Vector3d &Pbc_, const Vector3d &Pw_) {
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            Rbc = Rbc_.matrix();
            Pbc = Pbc_;
            Pw = Pw_;
        }

    protected:
        // Camera intrinsics
        double fx, fy, cx, cy;
        // Camera-IMU extrinsics
        Matrix3d Rbc;
        Vector3d Pbc;
        // Point position in world frame
        Vector3d Pw;
    };


    /**
     * @brief The VertexGyrBias class
     * For gyroscope bias compuation in Visual-Inertial initialization
     */
    class VertexGyrBias : public BaseVertex<3, Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexGyrBias();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        virtual void setToOriginImpl() {
            _estimate.setZero();
        }

        virtual void oplusImpl(const double *update_);
    };

 

    
    
    /**
     * @brief The EdgeGyrBias class
     * For gyroscope bias compuation in Visual-Inertial initialization
     */
    class EdgeGyrBias : public BaseUnaryEdge<3, Vector3d, VertexGyrBias> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeGyrBias();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        Matrix3d dRbij;
        Matrix3d J_dR_bg;
        Matrix3d Rwbi;
        Matrix3d Rwbj;

        void computeError();

        virtual void linearizeOplus();
    };
    
    class EdgeNavStateICP : public BaseBinaryEdge<6, SE3d, VertexNavStatePVR, VertexNavStatePVR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeNavStateICP();

        bool read(std::istream &is){return true;}

        bool write(std::ostream &os) const{return true;}

        void computeError();
	
// 	virtual void linearizeOplus();
	
	void setTbl(const SE3d& Tbl_) {Tbl = Tbl_;}
	
	void setMeasurement(const SE3d& m){
	  _measurement = Tbl * m * Tbl.inverse();
	  _inverseMeasurement = m.inverse();
	}

//         virtual void linearizeOplus();

//         void SetParams(const Vector3d &gw) {
//             GravityVec = gw;
//         }

//     protected:
        // Gravity vector in 'world' frame
//         Vector3d GravityVec;
protected:
      SE3d _inverseMeasurement;
      SE3d Tbl;
    };
    
class EdgeNavStateOnlyPose: public BaseBinaryEdge<6, SE3d, VertexNavStatePVR, VertexNavStatePVR>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    EdgeNavStateOnlyPose(): BaseBinaryEdge< 6, Sophus::SE3d, g2o::VertexNavStatePVR, g2o::VertexNavStatePVR >(){}
  
    void setTlb(SE3d& Tlb_) {Tlb = Tlb_;}
    
    bool read(std::istream &is){return true;}

    bool write(std::ostream &os) const {return true;}
    
    void cmputeError();
    
    void setMeasurement(const SE3d& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }
  
//       void linearizeOplus();

//       virtual int measurementDimension() const {return 7;}
  
//       virtual bool setMeasurementFromState();
// /*
// //       virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
// //           OptimizableGraph::Vertex* /*to*/) { 
// //         return 1.;
// //       }
// // 	*/
// // 	virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
// 	
protected:
  SE3d _inverseMeasurement;
  SE3d Tlb;
};


   /**
     * @brief Vertex to estimate the pitch and roll 
     */  
    class VertexSO3PitchRoll : public BaseVertex<2, Vector2d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSO3PitchRoll();

        bool read(std::istream &is){return true;}

        bool write(std::ostream &os) const {return true;}

        virtual void setToOriginImpl() {
            _estimate.setZero();
        }

        virtual void oplusImpl(const double *update_);
    };


    /**
     * @brief Vertex to estimate the initial ba
     */  
    class VertexBa : public BaseVertex<3, Vector3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexBa() : BaseVertex< 3, Vector3d >() {}

        bool read(std::istream &is){return true;}

        bool write(std::ostream &os) const{return true;}

        virtual void setToOriginImpl() {
            _estimate.setZero();
        }

        virtual void oplusImpl(const double *update_);
    };
    /**
     * @brief The edge to estimate the pitch, roll and ba
     */    

    
    class EdgeCoarsePitchRollBa : public BaseBinaryEdge<3, Vector3d, VertexSO3PitchRoll, VertexBa> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeCoarsePitchRollBa();

        bool read(std::istream &is){return true;}

        bool write(std::ostream &os) const{return true;}


        void computeError();
	
	private:
	  Vector3d gn;
    }; 
    
    class EdgeBiasa : public BaseUnaryEdge<3, Vector3d, VertexBa>{
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeBiasa();
	
	bool read(std::istream &is){return true;}

        bool write(std::ostream &os) const{return true;}
        
        void computeError();
	
	private:
	  Vector3d priori;
	
    };
//     
}

#endif // G2OTYPES_H
