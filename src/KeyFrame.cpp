#include "KeyFrame.h"
#include <boost/graph/graph_concepts.hpp>

using namespace std;
using namespace PointMatcherSupport;

namespace Velodyne_SLAM {
	
 
long unsigned int KeyFrame::nNextId = 0;
KeyFrame::KeyFrame()
{
	mLocalMapPoints = nullptr;
	Optimized = false;
	mnId = nNextId;
	PoseSetted = false;
	
}

KeyFrame::KeyFrame(const ros::Time& stamp, PointMatcher< float >::TransformationParameters pTicp, KeyFrame::DP* pMapPoints, long unsigned int pFrameId):
mstamp(stamp), mTicp(pTicp), mLocalMapPoints(pMapPoints),mnFrameId(pFrameId)
{
	mnId = nNextId++;
	Optimized = false;
	PoseSetted = true;
// 	preKFId = 0;
// 	mRelative_Pose = PM::TransformationParameters::Identity(4, 4);
// 	mTicp = PM::TransformationParameters::Identity(4, 4);
}

KeyFrame::KeyFrame(const ros::Time& stamp, PointMatcher< float >::TransformationParameters pTicp, long unsigned int pFrameId):
mstamp(stamp), mTicp(pTicp), mnFrameId(pFrameId)
{
	mnId = nNextId++;
	Optimized = false;
	mLocalMapPoints = nullptr;
	PoseSetted = true;
}

 void KeyFrame::setNextId(const long unsigned int id)
{
	nNextId = id;

}


bool KeyFrame::getGPS(GPS& gps)
{
	boost::mutex::scoped_lock lock(mMutexPose);
	if(hasGPS){
		gps = mGPS;
		return true;
	}else{
		return false;
	}

}

KeyFrame::DP* KeyFrame::getFrameDataPoints()
{
	return mFrameMapPoints;
}

void KeyFrame:: setGPS(GPS pgps){
	gps = pgps;
}

void KeyFrame::SetPose(PointMatcher< float >::TransformationParameters mT)
{
	boost::mutex::scoped_lock lock(mMutexPose);
	mTicp = mT;
	PoseSetted = true;
}
bool KeyFrame::isPoseSetted()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return PoseSetted;
}


PointMatcher< float >::TransformationParameters KeyFrame::getPose()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return mTicp;
}

PointMatcher< float >::TransformationParameters KeyFrame::getUnPose()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	if(Optimized)
		return mTicp_Unoptimized;
	else 
		return mTicp;
}


void KeyFrame::OptimizedPose(PointMatcher< float >::TransformationParameters mT)
{
	boost::mutex::scoped_lock lock(mMutexPose);
	mTicp_Unoptimized = mTicp;
	mTicp = mT;
	Optimized = true;
}




// KeyFrame::KeyFrame(const ros::Time& stamp, DP* frameMapPoints, PointMatcher< float >::TransformationParameters pTicp, const DP* pMapPoints, long unsigned int pFrameId)
// :mstamp(stamp), mFrameMapPoints(frameMapPoints), mTicp(pTicp), mLocalMapPoints(pMapPoints),mnFrameId(pFrameId)
// {
// 	mnId=nNextId++;
// 	
// }


}//end of namespace
