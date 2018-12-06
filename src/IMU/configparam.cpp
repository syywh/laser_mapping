#include "IMU/configparam.h"
#include <iostream>
#include <fstream>
#include "glog/logging.h"
namespace vill {
    double ConfigParam::_g = 9.810;

    Sophus::SE3d ConfigParam::_SE3Tbl = Sophus::SE3d();
    Eigen::Matrix4d ConfigParam::_EigTbl = Eigen::Matrix4d::Identity();
//     cv::Mat ConfigParam::_MatTbc = cv::Mat::eye(4, 4, CV_32F);
    Eigen::Matrix4d ConfigParam::_EigTlb = Eigen::Matrix4d::Identity();
//     cv::Mat ConfigParam::_MatTcb = cv::Mat::eye(4, 4, CV_32F);

    bool ConfigParam::_bAccMultiply9p8 = false;

	int ConfigParam::_verbose = 1;
	

    ConfigParam::ConfigParam(string extrinsicTLCfile, string refinefile, string extrinsicTBCfile)
    {
      fstream f(extrinsicTLCfile);
      fstream frefine(refinefile);
      fstream ftbc(extrinsicTBCfile);
	  Eigen::Matrix4d Tvelo_cam = Eigen::Matrix4d::Identity();
	  Eigen::Matrix4d Trefine = Eigen::Matrix4d::Identity();
	  Eigen::Matrix4d Tbc;
	  if(f.is_open() && frefine.is_open() && ftbc.is_open()){
	    for(int i = 0; i <3; i++){
	      for(int j = 0; j < 4; j ++){
		f >> Tvelo_cam(i,j);
		frefine >> Trefine(i,j);
// 		cout<<Tvelo_cam(i,j);
	      }
// 	      cout<<endl;
	    }
	    
	    Tvelo_cam = Tvelo_cam.inverse() * Trefine;
	    cout<<"Tvelo_cam load\n"<<Tvelo_cam<<endl;
	    
	    for(int i = 0; i <4; i++){
	      for(int j = 0; j < 4; j ++){
		ftbc>> Tbc(i,j);

	      }
// 	      cout<<endl;
	    }
	     cout<<"tbc load\n"<<Tbc<<endl;
	  }else{
		Tvelo_cam<<   -0.0057,   -0.1104 ,   0.9939    ,0.1387,
		-1.0000 ,   0.0003  , -0.0057   , 0.2613,
		0.0006 ,  -0.9938  , -0.1107   ,-0.0840,
		0    ,     0    ,     0  ,  1.0000;
		
		Tbc<<0.0281, -0.0194, 0.9994, 0.1461,
		-1.0041, -0.0129, 0.0278, 0.2614,
		0.0123, -0.9998, -0.0199, -0.0743,
		0, 0, 0, 1.0000;
	  }
	  

	  
	  Eigen::Quaterniond qr(Tbc.block<3,3>(0,0));
	  Tbc.block<3,3>(0,0) = qr.normalized().toRotationMatrix();
	  
	  _EigTbl = Tbc * Tvelo_cam.inverse();
	  _EigTlb = _EigTbl.inverse();
	  _SE3Tbl = Sophus::SE3d(_EigTbl);
	  LOG(INFO)<<"Tbl\n"<<_EigTbl;

    }

    void ConfigParam::setTbl(const SE3d& nTbl)
    {
	_SE3Tbl = nTbl;
	Matrix3d R = nTbl.rotationMatrix();
	Vector3d t = nTbl.translation();
	_EigTbl.block<3, 3>(0, 0) = R;
	_EigTbl.block<3, 1>(0, 3) = t;

	_EigTlb = Eigen::Matrix4d::Identity();
	_EigTlb.block<3, 3>(0, 0) = R.transpose();
	_EigTlb.block<3, 1>(0, 3) = -R.transpose() * t;
// 	for (int i = 0; i < 4; i++)
// 	    for (int j = 0; j < 4; j++)
// 		_MatTcb.at<float>(i, j) = _EigTcb(i, j);
    }
    

    Eigen::Matrix4d ConfigParam::GetEigTbl() {
        return _EigTbl;
    }


    Eigen::Matrix4d ConfigParam::GetEigT_lb() {
        return _EigTlb;
    }


    bool ConfigParam::GetAccMultiply9p8() {
        return _bAccMultiply9p8;
    }


}
