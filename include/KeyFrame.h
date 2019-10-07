#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "ros/package.h"

#include "bits/unique_ptr.h"
#include "boost/thread/future.hpp"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

// Services
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "map_msgs/GetPointMap.h"
#include "map_msgs/SaveMap.h"
// #include "ethzasl_icp_mapper/LoadMap.h"
// #include "ethzasl_icp_mapper/CorrectPose.h"
// #include "ethzasl_icp_mapper/SetMode.h"
// #include "ethzasl_icp_mapper/GetMode.h"
// #include "ethzasl_icp_mapper/GetBoundedMap.h" // FIXME: should that be moved to map_msgs?
#include "laser_mapping/LoadMap.h"
#include "laser_mapping/CorrectPose.h"
#include "laser_mapping/SetMode.h"
#include "laser_mapping/GetMode.h"
#include "laser_mapping/GetBoundedMap.h" 
#include "GPS.h"


using namespace std;

namespace Velodyne_SLAM
{
// class Frame;
class KeyFrame
{
public:
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	KeyFrame();
	//KeyFrame(const KeyFrame &keyframe);
// 	KeyFrame(const ros::Time& stamp, DP* frameMapPoints, PM::TransformationParameters pTicp,  DP*pMapPoints, long unsigned int pFrameId);
	KeyFrame(const ros::Time& stamp, PM::TransformationParameters pTicp,DP*pMapPoints, long unsigned int pFrameId);
	KeyFrame(const ros::Time& stamp, PM::TransformationParameters pTicp, long unsigned int pFrameId );
// 	KeyFrame(KeyFrame* oldKF, long unsigned int newmnId);

	void SetPose(PM::TransformationParameters mT);
	void OptimizedPose(PM::TransformationParameters mT);
	PM::TransformationParameters getPose();
	PM::TransformationParameters getUnPose();
	DP* getFrameDataPoints();
	bool getGPS(GPS& gps);
	bool getGeoXYZ( geoXYZ& pGeoXYZ );
	void setGPS(GPS pgps);
	void setGPSENA(double easting, double northing, double altitude, int status);
	void setGeoXYZ(Eigen::Matrix3d oriRot , Eigen::Vector3d oriTrans);
	void setGeoXYZ(Eigen::Vector2d oriTrans);
	void setGeoENA(Eigen::Vector3d OriENA);
	bool isNewNode();
	void setNewNode(bool flag);
	static void setNextId(const long unsigned int id);
	bool isPoseSetted();
	
	DP* mFrameMapPoints;
	DP* mLocalMapPoints;
	//DP mMapPoints_no_overlap;
	//DP mMapPoints_overlap;
	
	ros::Time mstamp;
	long unsigned int mnId;
// 	long unsigned int TrimmedmnId;	//指向代替该KeyFrame的Node，若等于mnId，说明自身是TrimmedNode
	long unsigned int mnFrameId;
// 	long unsigned int oldmnId;
// 	long unsigned int newmnId;
	static long unsigned int nNextId;
	
	bool Optimized;
	 boost::mutex mMutexPose;
	 PM::TransformationParameters mRelative_Pose;
	 long unsigned int preKFId;
	 
	 map<long unsigned int, PM::TransformationParameters> neighbours;	//默认第一个是icp时候的前一个
	map<long unsigned int, bool> neighbours_isReachable;
	 
	 map<long unsigned int, PM::TransformationParameters> neighboursforTrimmedKF;	//默认第一个是icp时候的前一个
	map<long unsigned int, bool> neighbours_isReachableforTrimmedKF;
	GPS gps;

protected:
	PM::TransformationParameters mTicp;		//当前KeyFrame的位姿
	PM::TransformationParameters mTicp_Unoptimized;		//优化后的Keyrame的位姿
	bool hasGPS;
	bool hasGeoXYZ;
// 	struct{
// 		double  longitute;	//经度
// 		double latitude;	//纬度
// 		
// 	}mGPS;
	GPS mGPS;
	
	geoXYZ mgeoxyz;
	bool NewNode;
	bool PoseSetted;
	
};
}//end of namespace
#endif
