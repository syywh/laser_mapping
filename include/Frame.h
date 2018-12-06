#ifndef FRAME_H
#define FRAME_H

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "ros/ros.h"
#include "GPS.h"
#include "IMU/IMUPreintegrator.h"
#include "imu_preintegration.h"
#include "IMU/NavState.h"
#include "Thirdparty/sophus/sophus/se3.hpp"
// #include "ros/console.h"
/*
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"*/

// #include "ros/package.h"




using namespace std;

namespace Velodyne_SLAM
{
class Frame
{
public:
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	Frame();
	Frame(const Frame &frame);
// 	Frame(const ros::Time& stamp, DP* frameMapPoints, PM::TransformationParameters pLocalTicp, const DP*pMapPoints, bool iskeyframe);
	Frame(const ros::Time& stamp, DP* frameMapPoints, const PM::TransformationParameters pLocalTicp, bool iskeyframe);

	 bool SetKF();
	 bool IsKF();
	 void SetRelatedKFId(long unsigned int pRelatedKFId);

	DP* mFrameMapPoints;	//Points in current frame
	const DP* mMapPoints;	//MapPoints in LocalFrame
	DP mMapPoints_no_overlap;
	DP mMapPoints_overlap;
	
	ros::Time mstamp;
	long unsigned int mnId;
	static long unsigned int nNextId;
	long unsigned int mnRelatedKFId;
	
	PM::TransformationParameters mLocalTicp;		//当前Frame的位姿
	
	//not safe, only use for initialization
	PM::TransformationParameters mTicp;
	void SetTicp(PM::TransformationParameters pTicp);
	PM::TransformationParameters GetGlobalPose();
	 void ComputePreInt(Vector3d& bg);
	 void ComputePreInt();
	 Frame* previousF = nullptr;
	Frame* previousF_KF = nullptr;
	//imu preintegration
	boost::mutex mMutexImuPreint;    
	vill::IMUPreintegrator mIMUPreint;
	vector<vill::IMUData> mvIMUdatas;
	void SetIMUPreint(vill::IMUPreintegrator& pIMUpreint);
	void SetIMUdata(vector<vill::IMUData>& mIMUdata);
	vill::IMUPreintegrator GetIMUPreint();
	
	void UpdateNavStatePVRFromTwc(const SE3d &Twc, const SE3d &Tbc);
	const vill::NavState &GetNavState(void);
	
	
        void SetNavStateVel(const Vector3d &vel);

        void SetNavStatePos(const Vector3d &pos);

        void SetNavStateRot(const Matrix3d &rot);

        void SetNavStateRot(const SO3d & rot);

        void SetNavStateBiasGyr(const Vector3d &bg);

        void SetNavStateBiasAcc(const Vector3d &ba);

        void SetNavStateDeltaBg(const Vector3d &dbg);

        void SetNavStateDeltaBa(const Vector3d &dba);
	
	void UpdatePoseFromNS(const SE3d &Tbc);
	
	void SetInitialNavStateAndBias(const vill::NavState &ns);
	
	bool isProcessed();//is ok to delete
	
	void setProcessed(bool flag);
	
	void UpdateNavState(const vill::IMUPreintegrator &imupreint, const Vector3d &gw);
	
	int previousFId;
	
	GPS gps;
	
	void setGPS(GPS pgps );
protected:
	 bool isKeyFrame;
        // P, V, R, bg, ba, delta_bg, delta_ba (delta_bx is for optimization update)
        boost::mutex mMutexNavState;
        vill::NavState mNavState;
	
	boost::mutex mMutexProcessed;
	bool processed;
	
};
}//end of namespace
#endif