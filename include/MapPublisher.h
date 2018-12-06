#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include "LocalMapping.h"
#include "Map.h"
// #include "pointmatcher/PointMatcher.h"
#include "LoopClosing.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "Frame.h"

#include <mutex>

namespace Velodyne_SLAM
{
class Map;
class LocalMapping;
class LoopClosing;
class MapPublisher{
public:
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	MapPublisher(Map* pMap, LocalMapping* pLocalMapping, LoopClosing* pLoopCloser);
	void Refresh();
	void showWholeMap();
	
	Map* mpMap;
	LocalMapping* mLocalMapping; 
	LoopClosing* mLoopCloser;
	
	void UpdateLocalT(PM::TransformationParameters pT);
	void Run();
	
private:
	ros::NodeHandle nh;
	ros::Publisher WholeMapPublisher;
	ros::Publisher ReferenceMapPublisher;
	ros::Publisher UpdateMapPublisher;
	ros::Publisher odomPub_Correct;	//Publishers "icp_odom"
	ros::Publisher odomPub;	//Publishers "icp_odom"
	tf::TransformBroadcaster tfBroadcaster;
	PM::TransformationParameters TMapLocal ;
	PM::TransformationParameters Ttemp ;
// 	PM::TransformationParameters Ttemp ;
	size_t mLastPublishedKeyFrameId;
	unique_ptr<PM::Transformation> transformation;
	
	std::mutex mMutexLocalT;   
	bool initialized;
	int beginID;
	
};
}


#endif