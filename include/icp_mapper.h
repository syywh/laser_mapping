#ifndef ICP_MAPPER_H
#define ICP_MAPPER_H

#include "Frame.h"
#include "preintegration_opt.h"
#include "IMU/imudata.h"
#include "imu_preintegration.h"
#include "odom_preintegration.h"
#include "LocalMapping.h"
#include "laser_mapping/Localizing_Mode.h"
// #include "IMU/IMUPreintegrator.h"

#include "fstream"
#include "glog/logging.h"

#include "bits/unique_ptr.h"
#include "bits/stl_queue.h"
#include "boost/thread/future.hpp"
#include <boost/graph/graph_concepts.hpp>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "eigen_conversions/eigen_msg.h"

// Services
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

//msg
#include "nav_msgs/Odometry.h"
#include <rtk_gps/Position.h>
#include <rtk_gps/Orientation.h>
#include <gps_common/GPSFix.h>
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/Imu.h>


#include <message_filters/subscriber.h>
#include <message_filters/connection.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace std;
using namespace LIV;

namespace Velodyne_SLAM
{
class Frame;
class LocalMapping;
class Map;
class preintegration_opt;


class ICPMapper
{

	
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	typedef typename NNS::SearchType NNSearchType;
	
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
		rtk_gps::Position,
		rtk_gps::Orientation> PointCloudwithGPS;
		
	ros::NodeHandle& n;

	
	// Subscribers
	ros::Subscriber scanSub;
	ros::Subscriber cloudSub;
	ros::Subscriber gps_position;
	ros::Subscriber gps_orientation;
	ros::Subscriber gps_fix;
	ros::Subscriber imuSub;
	ros::Subscriber stateSub;
	
	message_filters::Subscriber<sensor_msgs::PointCloud2> message_filter_cloud;
	message_filters::Subscriber<rtk_gps::Position> message_filter_gps_position;
	message_filters::Subscriber<rtk_gps::Orientation> message_filter_gps_orientation;
	
	message_filters::Synchronizer<PointCloudwithGPS> synchronizer_;
	message_filters::Connection connection;

	
// 	
	
	// Publishers
	ros::Publisher mapPub;	//Publishers "point_map"
	ros::Publisher outlierPub;	//Publishers "outliers"
	ros::Publisher odomPub;	//Publishers "icp_odom"
	ros::Publisher odomErrorPub;//Publishers "icp_error_odom"


	// Timef
	ros::Time mapCreationTime;
	ros::Time lastPoinCloudTime;
	
	// libpointmatcher
	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPreFilters;
	PM::DataPointsFilters mapPostFilters;
	PM::DataPoints *mapPointCloud;
	PM::DataPoints MapPointCloud;
	PM::ICPSequence icp;
	PM::ICPSequence icpInit;
	unique_ptr<PM::Transformation> transformation;
	
	#if BOOST_VERSION >= 104100
	typedef boost::packaged_task<PM::DataPoints*> MapBuildingTask;
	typedef boost::unique_future<PM::DataPoints*> MapBuildingFuture;
	boost::thread mapBuildingThread;
	MapBuildingTask mapBuildingTask;
	MapBuildingFuture mapBuildingFuture;
	bool mapBuildingInProgress;
	#endif // BOOST_VERSION >= 104100
	bool processingNewCloud; 

	// Parameters
	bool publishMapTf; 
	bool useConstMotionModel; 
	bool localizing;	//是否定位的标志位
	bool mapping;	//是否建图的标志位
	int minReadingPointCount;
	int minMapPointCount;
	int inputQueueSize; 
	double minOverlap;
	double maxOverlapToMerge;
	double tfRefreshPeriod;  //!< if set to zero, tf will be publish at the rate of the incoming point cloud messages 
	string laserFrame;  //launch中param  odom_frame
	string imuFrame;
	string mapFrame;
	string vtkFinalMapName; //!< name of the final vtk map

	const double mapElevation; // initial correction on z-axis //FIXME: handle the full matrix
	
	// Parameters for dynamic filtering
	const float priorStatic; //!< ratio. Prior to be static when a new point is added
	const float priorDyn; //!< ratio. Prior to be dynamic when a new point is added
// 	const float maxAngle; //!< in rad. Openning angle of a laser beam
// 	const float eps_a; //!< ratio. Error proportional to the laser distance
// 	const float eps_d; //!< in meter. Fix error on the laser distance
// 	const float alpha; //!< ratio. Propability of staying static given that the point was dynamic
// 	const float beta; //!< ratio. Propability of staying dynamic given that the point was static
// 	const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic
// 	const float maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map.
// 	const float sensorMaxRange;//sensor max range


	PM::TransformationParameters TimuToMap, TLaserToMap, TodomToMap;
	PM::TransformationParameters TLaserToLocalMap;
	PM::TransformationParameters TimuToLaser, TodomToLaser;
	boost::thread publishThread;
	boost::mutex publishLock;
	ros::Time publishStamp;
	
	tf::TransformListener tfListener;	//用于laserscan回调函数中
	tf::TransformBroadcaster tfBroadcaster;
	
	bool Debug_ICP;
	//edit
	int Index;
	float halfRoadWidth;
	int MaxLocalMappoints;
	float KFthreOverlap;
	float maxAngle;
	int maxLocalMapLength;
	
    enum eMappingState{
        NOT_INITIALIZED=0,
        INITIALIZING=1,
        WORKING=2,
        LOST=3,
        NOT_LOCALIZEING=4,
	LOCALIZEING = 5
    };
	
	boost::mutex mMutexState;
	eMappingState mCurrentState;
	eMappingState mLastState;
	 
	
	boost::mutex mMutexLocalMap;    
	
	const float eps;
	
// 	vector<PM::TransformationParameters> mTemp_FramePose_Record;//TEST
	vector<Frame*> mTemp_Frame_Record;//TEST used for record frame id for mengmeng

	struct BoolSetter
	{
	public:
		bool toSetValue;
		BoolSetter(bool& target, bool toSetValue):
			toSetValue(toSetValue),
			target(target)
		{}
		~BoolSetter()
		{
			target = toSetValue;
		}
	protected:
		bool& target;
	};

	bool buse_odom, buse_imu;
	
	OdomPreintegrator previousOdom;
	
public:
	ICPMapper(ros::NodeHandle& nh, Map* mMap, bool use_odom, bool use_imu);
	~ICPMapper();
	
	void SetLocalMapper(LocalMapping* pLocalMapper);
	bool isOKforCreateNewKeyFrame();
	void SetStopCreateNewKeyFrame();
	void ResetStopCreateNewKeyFrame();
	void OkForProcessNewFrame();
	void StopProcessNewFrame();
	bool isOkForProcessNewFrame();
	void startSynchronize();
	
	void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
	void gotCloudSlidingMap(const sensor_msgs::PointCloud2& cloudMsgIn);
// 	void SetMapper(LocalMapping* pLocalMapper);
	
	int getPreviousFrameId();
	
	void Run();
	void RunSlidingMap();
	void setOdoPre(LIV::odom_preintegration* podoPre);
	void setImuPre(LIV::imu_preintegration* pimuPre);
	void setPreintegrationOpt(LIV::preintegration_opt* ppreintegration_opt);
	ofstream recordKF;

	bool initializationforReSLAM();

	string getCloudName(){
		return cloud_in;
	}
	void gotGPSNavSatFix(const sensor_msgs::NavSatFix& gpsMsgIn);
	void gotGPSFix(const gps_common::GPSFix& gpsMsgIn, double yaw);
	void stop();
	
protected:
	void gotScan(const sensor_msgs::LaserScan& scanMsgIn);
	
	void gotGPSPosition(const rtk_gps::Position& gpsPositionMsgIn);
	void gotGPSOrientation(const rtk_gps::Orientation& gpsOrientationMsgIn);
	void gotGPSFix(const gps_common::GPSFix& gpsMsgIn);
	
	void gotCloudandGPS(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn, const rtk_gps::PositionConstPtr& gpsMsgInP, const rtk_gps::OrientationConstPtr& gpsMsgInO);
	void gotIMU( const sensor_msgs::ImuConstPtr& msg);
	void gotState(const laser_mapping::Localizing_Mode& msg);
// 	void processCloud(unique_ptr<DP> cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processCloud_imu(DP* cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processCloud(DP* cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processCloud_odom(DP* cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processCloudSlidingMap_imu(DP* cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processCloudSlidingMap(DP* cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
	void processCloudandGPS(DP* cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq, GPS gps);
	void processNewMapIfAvailable();
	void setMap(DP* newPointCloud);
	DP* updateMap(DP* newPointCloud, const PM::TransformationParameters Ticp, bool updateExisting);
	void waitForMapBuildingCompleted();
	
	void publishLoop(double publishPeriod);
	void publishTransform();
	
	// Services
// 	bool getPointMap(map_msgs::GetPointMap::Request &req, map_msgs::GetPointMap::Response &res);
// 	bool saveMap(map_msgs::SaveMap::Request &req, map_msgs::SaveMap::Response &res);
// 	bool loadMap(ethzasl_icp_mapper::LoadMap::Request &req, ethzasl_icp_mapper::LoadMap::Response &res);
// 	bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
// 	bool correctPose(ethzasl_icp_mapper::CorrectPose::Request &req, ethzasl_icp_mapper::CorrectPose::Response &res);
// 	bool setMode(ethzasl_icp_mapper::SetMode::Request &req, ethzasl_icp_mapper::SetMode::Response &res);
// 	bool getMode(ethzasl_icp_mapper::GetMode::Request &req, ethzasl_icp_mapper::GetMode::Response &res);
// 	bool getBoundedMap(ethzasl_icp_mapper::GetBoundedMap::Request &req, ethzasl_icp_mapper::GetBoundedMap::Response &res);
	
	bool loadPosetxt(string posetxt, PM::TransformationParameters& pose);
	//edit
	boost::mutex mMutexCreateNewKeyFrame;    
	boost::mutex mMutexCreateNewFrame;   
	LocalMapping* mpLocalMapper;
	Frame mcurrentFrame;
	Map* mpMap;
	LIV::odom_preintegration* modom_preintegration;
	LIV::imu_preintegration* mimu_preintegration;
	LIV::preintegration_opt* mpreintegration_optimizer;
	
	
	boost::mutex mMutexPreviousFrame;
	
	Frame* previousFrame;
	
	bool OKForCreateNewKeyFrame;
	bool OKProcessNewFrame;
	
	long unsigned int lastProcessedKeyFrame;
	int checkLocalizing;
	DP DPforLocalizing;
	
	bool bAccMultiply98 ;
	
	PM::TransformationParameters Ticplocal;
	PM::TransformationParameters firstICP;
	
	GPS mappergps;
	ros::Time gps_position_time;
	ros::Time gps_orientation_time;
	ros::Time gps_time;
	
	//message name
	string cloud_in;
	string imu_in;
	
	vector<vill::IMUData> deltaIMUdata;
	
	//data
	std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;

	int cnt_for_lost_imu;
	bool reInitialized;
};
}//end of namespace
#endif
