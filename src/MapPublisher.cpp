#include "MapPublisher.h"

namespace Velodyne_SLAM {

MapPublisher::MapPublisher(Map* pMap, LocalMapping* pLocalMapping,LoopClosing* pLoopCloser):mpMap(pMap),mLocalMapping(pLocalMapping),mLoopCloser(pLoopCloser),transformation(PM::get().REG(Transformation).create("RigidTransformation"))
{
	WholeMapPublisher = nh.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
	ReferenceMapPublisher = nh.advertise<sensor_msgs::PointCloud2>("Reference_map",2,true);
	UpdateMapPublisher = nh.advertise<sensor_msgs::PointCloud2>("Dynamic_map",2,true);
	odomPub_Correct = nh.advertise<nav_msgs::Odometry>("LoopClosed_odom", 50000, true);
	odomPub = nh.advertise<nav_msgs::Odometry>("keyframe_odom", 5000, true);
	mLastPublishedKeyFrameId = mpMap->getKeyFrameNum() ;
	beginID = mpMap->getKeyFrameNum();
	initialized = false;
	
	TMapLocal= PM::TransformationParameters::Identity(4,4);
}

void MapPublisher::Run()
{
  ros::Rate r(1);
  while(ros::ok()){
    Refresh();
    r.sleep();
  }

}


void MapPublisher::Refresh()
{
// 	{
// 		boost::mutex::scoped_lock lock(mMutexLocalT);
// 		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TMapLocal, "/map", "/Local", ros::Time::now()));
// 	}
	if(mpMap->isMapUpdated())
	{
		DP temp;
		mpMap->getWholeMapPoints(temp);
		
// 		DP mapLocalFrameCut(temp.createSimilarEmpty());
// 		DP::View viewOnProbabilityDynamic = temp.getDescriptorViewByName("probabilityDynamic");
		
// 		int localMapSize  = temp.features.cols();
// 		for(size_t i = 0; i<localMapSize; i++)
// 		{
// // 			if(viewOnProbabilityDynamic(0,i) < )
// 		}
		
		WholeMapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(temp, "/map", ros::Time::now()));
		vector<KeyFrame*> tvpKF;
		mpMap->getAllKeyFrames(tvpKF);
		for(; mLastPublishedKeyFrameId<tvpKF.size(); mLastPublishedKeyFrameId++)
		{
			KeyFrame* pKF = tvpKF[mLastPublishedKeyFrameId];
			odomPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(pKF->getPose(), "/map", ros::Time::now()));
		}
		mpMap->ResetUpdated();
		initialized=true;
	}else
	  if(initialized)
	{
		DP temp;
		mpMap->getWholeMapPoints(temp);
		WholeMapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(temp, "/map", ros::Time::now()));

	}
// 	
	if(mLocalMapping->LocalMapUpdateForVisualisation) 
	{	
		DP temp;
		PM::TransformationParameters Ttemp = mLocalMapping->getLocalTLocalICP();		
		mLocalMapping->getLocalMapPointsNew(temp);
		ros::Time LocalTime;
		mLocalMapping->getLocalMapTime(LocalTime);

		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ttemp, "/map", "/Local", ros::Time::now()));
		ReferenceMapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(temp, "/Local", ros::Time::now()));//need to br tf
		mLocalMapping->LocalMapUpdateForVisualisation = false;
// 
// 		
	}
	
	if(mLoopCloser->isUpdate)
	{
		vector<KeyFrame*> tvpKF;
		mpMap->getAllKeyFrames(tvpKF);
		for(size_t t = 0; t<tvpKF.size(); t++)
		{
			KeyFrame* pKF = tvpKF[t];
			odomPub_Correct.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(pKF->getPose(), "/map", ros::Time::now()));
		}
// 		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(tvpKF.back()->getPose(), "/map", "/Local", ros::Time::now()));
// 		UpdateMapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*(tvpKF.back()->mLocalMapPoints), "/Local", ros::Time::now()));//need to br tf
		mLoopCloser->isUpdate = false;
		mpMap->updateMap(tvpKF);
		mLastPublishedKeyFrameId = beginID;
	
	}
	

}

// void MapPublisher::UpdateLocalT(PointMatcher< float >::TransformationParameters pT)
// {
// // 	boost::mutex::scoped_lock lock(mMutexLocalT);
// // 	TMapLocal = pT;
// }
void MapPublisher::showWholeMap()
{
		DP temp;
		
		mpMap->getWholeMapPoints(temp);
		if(temp.features.cols()==0)
		{
			vector<KeyFrame*> tvpKF;
			mpMap->getAllKeyFrames(tvpKF);
			temp=tvpKF[0]->mLocalMapPoints->createSimilarEmpty();
			for(size_t t = 0; t<tvpKF.size(); t++)
			{
				KeyFrame* pKF = tvpKF[t];
				DP temp_DP = *(pKF->mLocalMapPoints);
				temp_DP = transformation->compute(temp_DP,pKF->getPose());
				temp.concatenate(temp_DP);
				
			}
// 			mpMap->setMapPoints(temp);
			cerr<<"Set MapPoints"<<endl;
			
		}
		WholeMapPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(temp, "/map", ros::Time::now()));

}



}