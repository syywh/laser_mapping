#ifndef YGZ_CONFIGPARAM_H_
#define YGZ_CONFIGPARAM_H_

#include <Eigen/Core>
#include <Thirdparty/sophus/sophus/se3.hpp>
#include <Thirdparty/sophus/sophus/so3.hpp>
using namespace Eigen;
using namespace Sophus;
namespace vill {

    class ConfigParam {
    public:
	static void setTbl(const SE3d &nTbl);
	
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ConfigParam(string extrinsicTLCfile, string refinefile, string extrinsicTBCfile);
	
        static SE3d GetSE3Tbl() { return _SE3Tbl; }

        static Matrix4d GetEigTbl();

//         static Mat GetMatTbc();

        static Matrix4d GetEigT_lb();

//         static Mat GetMatT_cb();


        static bool GetAccMultiply9p8();

        static double GetG() { return _g; }

        std::string _bagfile;
        std::string _imuTopic;

	static int _verbose;

    private:
        static SE3d _SE3Tbl;
        static Matrix4d _EigTbl;
//         static Mat _MatTbc;
        static Matrix4d _EigTlb;
//         static Mat _MatTcb;
        static int _LocalWindowSize;
	static int _InitWindowSize;
        static bool _bAccMultiply9p8;

        static double _g;
        static double _nVINSInitTime;

    };

    

}

#endif // CONFIGPARAM_H
