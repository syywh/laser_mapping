#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "Frame.h"
#include "icp_mapper.h"
#include "Map.h"

// #include "pointmatcher/PointMatcher.h"
// #include "nabo/nabo.h"
// #include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "KeyFrame.h"
#include "LoopClosing.h"

#include <Eigen/StdVector>
#include <Eigen/Core>
#include "IMU/configparam.h"
#include "IMU/IMUPreintegrator.h"

namespace Velodyne_SLAM
{
class LoopClosing;
class ICPMapper;
class Frame;
class LocalMapping{
public:
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	typedef typename NNS::SearchType NNSearchType;
	
	LocalMapping(Map* wMap,ros::NodeHandle& np );
	~LocalMapping();
	
	void Run();
	void RunSlidingMap();
	void InsertFrame(Frame* pFrame);
	bool AcceptFrames();
	void SetAcceptFrames(bool flag);
	bool ProcessLocalMap();
	void ProcessSlidingMap();
	void DivideCloud(Frame* pFrame);
	int getLocalMapPointsNumber();
	void getLocalMapPointsNew(DP& copyLocalMapPoints);
	PM::TransformationParameters getLocalTLocalICP();
	void getLocalMapTime(ros::Time& time);
	
// 	void UPdateLocalMapList();
	void SetOkForCutList();
	
	void SetIcpMapper(ICPMapper* pICPMapper);	//used for relocalization
	void SetLoopClosing(LoopClosing* pLoopClosing);
	
	void RequstStop();
	void GoOn();

	void StopInsertNewFrame();
	void OKInsertNewFrame();
	bool isOKForInsertNewFrame();
	
	void UPdateTLocalICP(int KFid);
	void Stopping();
	
	bool initializationforReSLAM();


	
	Map* mpMap;
	
	bool LocalMapUpdate;
	bool LocalMapUpdateForVisualisation;
	bool Initialized;

	ICPMapper* mpTracker;
	LoopClosing *mpLoopMapper;
	
	boost::mutex mMutexNewLocalMapPoints;    //for mLocalMappoints
	
	
	unique_ptr<PM::Transformation> transformation;
	KeyFrame* mCurrentKeyFrame;
	
	ros::NodeHandle& n;
	ros::Publisher localMapPub;
	bool bSlidingMap;

	bool hasProcessedOneFrame();
	void setProcessedOneFrame(bool flag);
	
private:
	bool CheckNewFrames();
	void CreateNewKeyFrames();

	
	int NumOfList;
	std::list<Frame* > mlNewFrames;
	std::list<Frame* > mLocalFrames;
	boost::mutex mMutexNewFrames;    			//for mNewFrames
	
// 	boost::mutex mMutexTLocalICP;    //for mLocalMappoints
	
	DP* mLocalMappoints;	//in Local !
	PM::TransformationParameters TLocalicp;
	
	bool mbAcceptKeyFrames;
// 	bool InsertFrame;
	boost::mutex mMutexAccept;
	boost::mutex mMutexStop;    


	
	int CountNumInLocalMap;
	Frame* mCurrentFrame;
	bool LocalListFull;
	ros::Time mLocalstamp;
	
// 	PM::ICPSequence icp;
	PM::DataPointsFilters mapPostFilters;
	PM::DataPointsFilters maximumDistanceFilteres;
	bool mbRequestStop;
	bool OkForCutList;
	
	bool OkForInsertNewFrame;
	
	KeyFrame* KFinLoopProcessing;
	
	tf::TransformBroadcaster tfBroadcaster;
	
	ofstream fPosetxt;
	
// 	// Parameters for dynamic filtering
// 	const float priorStatic = 0.55; //!< ratio. Prior to be static when a new point is added
// 	const float priorDyn =0.45;
// 	const float maxAngle; //!< in rad. Openning angle of a laser beam
// 	const float eps_a; //!< ratio. Error proportional to the laser distance
// 	const float eps_d; //!< in meter. Fix error on the laser distance
// 	const float alpha; //!< ratio. Propability of staying static given that the point was dynamic
// 	const float beta; //!< ratio. Propability of staying dynamic given that the point was static
// 	const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic
// 	const float maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map.
// 	const float sensorMaxRange;//sensor max range
// 	 float priorStatic=0.45 ; //!< ratio. Prior to be static when a new point is added
// 	 float priorDyn =0.55; //!< ratio. Prior to be dynamic when a new point is added
// 	 float maxAngle =0.01;//< in rad. Openning angle of a laser beam0.
// 	 float eps_a=0.1; //!< ratio. Error proportional to the laser distance
// 	 float eps_d = 0.1; //!< in m11ter. Fix error on the laser distance
// 	 float alpha = 0.99; //!< ratio. Propability of staying static given that the point was dynamic
// 	 float beta = 0.9; //!< ratio. Propability of staying dynamic given that the point was static
// 	 float maxDyn = 0.9; //!< ratio. Threshold for which a point will stay dynamic
// 	 float maxDistNewPoint = pow(0.6,2); //!< in meter. Distance at which a new point will be added in the global map.
// 	 float sensorMaxRange = 80;//sensor max range
// 	 float eps = 0.0001;
	 
	 float priorStatic=0.45 ; //!< ratio. Prior to be static when a new point is added
	 float priorDyn =0.55; //!< ratio. Prior to be dynamic when a new point is added
	 float maxAngle =0.01;//< in rad. Openning angle of a laser beam0.
	 float eps_a=0.1; //!< ratio. Error proportional to the laser distance
	 float eps_d = 0.1; //!< in m11ter. Fix error on the laser distance
	 float alpha = 0.99; //!< ratio. Propability of staying static given that the point was dynamic
	 float beta = 0.9; //!< ratio. Propability of staying dynamic given that the point was static
	 float maxDyn = 0.9; //!< ratio. Threshold for which a point will stay dynamic
	 float maxDistNewPoint = pow(0.6,2); //!< in meter. Distance at which a new point will be added in the global map.
	 float sensorMaxRange = 80;//sensor max range
	 float eps = 0.0001;
	 float minAddDist = 2;
	 float robot_height = 0.7;

	 bool processedOneFrame;
	 boost::mutex mMutexprocessedOncFrame;
	
};
}


#endif
