// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Modified by Raúl Mur Artal (2014)
// Added EdgeSE3ProjectXYZ (project using focal_length in x,y directions)
// Modified by Raúl Mur Artal (2016)
// Added EdgeStereoSE3ProjectXYZ (project using focal_length in x,y directions)
// Added EdgeSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)
// Added EdgeStereoSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)

#ifndef G2O_SIX_DOF_TYPES_EXPMAP
#define G2O_SIX_DOF_TYPES_EXPMAP

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"

#include "types_sba.h"
#include <Eigen/Geometry>

#include "sophus/se3.hpp"

using Sophus::SE3d;


namespace g2o
{
namespace types_six_dof_expmap
{
void init();
}

using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;


/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class  VertexSE3Expmap : public BaseVertex<6, SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSE3Expmap() {}

    bool read ( std::istream& is );

    bool write ( std::ostream& os ) const;

    virtual void setToOriginImpl() {
        _estimate = SE3d();
    }

    virtual void oplusImpl ( const double* update_ )  {
        Vector6d update;
        update<<update_[3], update_[4],update_[5],update_[0],update_[1],update_[2];
        setEstimate ( SE3d::exp ( update ) *estimate() );
    }
};


class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ() {}

    bool read ( std::istream& is );

    bool write ( std::ostream& os ) const;

    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
        Vector2d obs ( _measurement );
        _error = obs-cam_project ( v1->estimate() * ( v2->estimate() ) );
    }

    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
        return ( v1->estimate() * ( v2->estimate() ) ) ( 2 ) >0.0;
    }


    virtual void linearizeOplus();

    Vector2d cam_project ( const Vector3d & trans_xyz ) const;

    double fx, fy, cx, cy;
};


class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoSE3ProjectXYZ();

    bool read ( std::istream& is );

    bool write ( std::ostream& os ) const;

    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
        Vector3d obs ( _measurement );
        _error = obs - cam_project ( v1->estimate() * ( v2->estimate() ),bf );
    }

    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[1] );
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*> ( _vertices[0] );
        return ( v1->estimate() * ( v2->estimate() ) ) ( 2 ) >0.0;
    }


    virtual void linearizeOplus();

    Vector3d cam_project ( const Vector3d & trans_xyz, const float &bf ) const;

    double fx, fy, cx, cy, bf;
};

class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose() {}

    bool read ( std::istream& is );

    bool write ( std::ostream& os ) const;

    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        Vector2d obs ( _measurement );
        _error = obs-cam_project ( v1->estimate() * ( Xw ) );
    }

    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        return ( v1->estimate() * ( Xw ) ) ( 2 ) >0.0;
    }


    virtual void linearizeOplus();

    Vector2d cam_project ( const Vector3d & trans_xyz ) const;

    Vector3d Xw;
    double fx, fy, cx, cy;
};


class  EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoSE3ProjectXYZOnlyPose() {}

    bool read ( std::istream& is );

    bool write ( std::ostream& os ) const;

    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        Vector3d obs ( _measurement );
        _error = obs - cam_project ( v1->estimate() * ( Xw ) );
    }

    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        return ( v1->estimate() * ( Xw ) ) ( 2 ) >0.0;
    }


    virtual void linearizeOplus();

    Vector3d cam_project ( const Vector3d & trans_xyz ) const;

    Vector3d Xw;
    double fx, fy, cx, cy, bf;
};

 class  EdgeSE3 : public BaseBinaryEdge<6, SE3d, VertexSE3Expmap, VertexSE3Expmap> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      virtual void setMeasurement(const SE3d& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

//       virtual bool setMeasurementData(const double* d){
//         Eigen::Map<const Vector7d> v(d);
//         setMeasurement(internal::fromVectorQT(v));
//         return true;
//       }
// 
//       virtual bool getMeasurementData(double* d) const{
//         Eigen::Map<Vector7d> v(d);
//         v = internal::toVectorQT(_measurement);
//         return true;
//       }

//       void linearizeOplus();

      virtual int measurementDimension() const {return 7;}

      virtual bool setMeasurementFromState() ;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
          OptimizableGraph::Vertex* /*to*/) { 
        return 1.;
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    protected:
      SE3d _inverseMeasurement;
  };

} // end namespace

#endif
