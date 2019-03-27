#include "LocalMapping.h"
// #include "LoopClosing.h"
#include "ros/package.h"
// #include "pointmatcher_ros/get_params_from_server.h"
#include <boost/graph/graph_concepts.hpp>


// using namespace PointMatcherSupport;
namespace Velodyne_SLAM {

LocalMapping::LocalMapping(Map* wMap, ros::NodeHandle& np):mpMap(wMap),n(np)
,transformation(PM::get().REG(Transformation).create("RigidTransformation")),OkForInsertNewFrame(true),
processedOneFrame(false)
// ,
// 	priorStatic(getParam<double>("priorStatic", 0.45)),
// 	priorDyn(getParam<double>("priorDyn", 0.55)),
// 	maxAngle(getParam<double>("maxAngle", 0.01)),
// 	eps_a(getParam<double>("eps_a", 0.1)),
// 	eps_d(getParam<double>("eps_d", 0.1)),
// 	alpha(getParam<double>("alpha", 0.99)),
// 	beta(getParam<double>("beta", 0.9)),
// 	maxDyn(getParam<double>("maxDyn", 0.9)),
// 	sensorMaxRange(getParam<float>("sensorMaxRange",100)),
// 	maxDistNewPoint(pow(getParam<double>("maxDistNewPoint", 0.7),2))
{
	
	LocalMapUpdate = false;
	LocalMapUpdateForVisualisation = false;
	Initialized = false;
	NumOfList = 30;
	LocalListFull = false;
	mCurrentFrame = nullptr;
	mbRequestStop = false;
	OkForCutList = false;
	bSlidingMap = false;

	string name;
	name = "priorStatic";
	if (ros::param::get(std::string("~")+name, priorStatic))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << priorStatic);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << priorStatic);

	name = "priorDyn";
	if (ros::param::get(std::string("~")+name, priorDyn))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << priorDyn);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << priorDyn);
	
	name = "maxAngle";
	if (ros::param::get(std::string("~")+name, maxAngle))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << maxAngle);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << maxAngle);
	
	name = "eps_a";
	if (ros::param::get(std::string("~")+name, eps_a))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << eps_a);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << eps_a);
	
	name = "eps_d";
	if (ros::param::get(std::string("~")+name, eps_d))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << eps_d);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << eps_d);

	name = "alpha";
	if (ros::param::get(std::string("~")+name, alpha))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << alpha);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << alpha);
	
	name = "beta";
	if (ros::param::get(std::string("~")+name, beta))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << beta);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << beta);
	
	name = "maxDyn";
	if (ros::param::get(std::string("~")+name, maxDyn))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << maxDyn);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << maxDyn);
	
	name = "sensorMaxRange";
	if (ros::param::get(std::string("~")+name, sensorMaxRange))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << sensorMaxRange);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << sensorMaxRange);
	
	name = "maxDistNewPoint";
	if (ros::param::get(std::string("~")+name, maxDistNewPoint))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << maxDistNewPoint);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << maxDistNewPoint);
	
	name = "min_add_dist";
	if (ros::param::get(std::string("~")+name, minAddDist))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << minAddDist);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << minAddDist);

	name = "robot_height";
	if (ros::param::get(std::string("~")+name, robot_height))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << robot_height);
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << robot_height);
	
	string configFileName;
	if (ros::param::get("~mapPostFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			
			mapPostFilters = PM::DataPointsFilters(ifs);
			cerr<<"[LocalMap] post filter loaded"<<endl<<endl;
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file " << configFileName);
		}
	}
	else
	{
		
		ROS_INFO_STREAM("No map post-filters config file given, $(find velodyne_slam)/cfg/map_post_filters.yaml");
		configFileName =ros::package::getPath("laser_mapping")+ "/cfg/map_post_filters.yaml";
		
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPostFilters = PM::DataPointsFilters(ifs);
			cerr<<"[Local Map] Post Filter Loaded from default yaml"<<endl;
		}else
		{
			ROS_ERROR_STREAM("Cannot load pose filter config from YAML file " << configFileName);

		}
	}
	
	if(ros::param::get("~maximumDistFiltersConfig",configFileName)){
	  ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			
			maximumDistanceFilteres = PM::DataPointsFilters(ifs);
			cerr<<"[LocalMap] maximum distance filter loaded"<<endl<<endl;
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map maximumdist-filters config from YAML file " << configFileName);
		}
	}else{
	  ROS_INFO_STREAM("No maximum distance filters config file given, $(find velodyne_slam)/cfg/maximumDistFiltering.yaml");
	  configFileName =ros::package::getPath("laser_mapping")+ "/cfg/maximumDistFiltering.yaml";
		
	  ifstream ifs(configFileName.c_str());
	  if (ifs.good())
	  {
		  maximumDistanceFilteres = PM::DataPointsFilters(ifs);
		  cerr<<"[Local Map] maximum distance Loaded from default yaml"<<endl;
	  }else
	  {
		  ROS_ERROR_STREAM("Cannot load maximum distance filter config from YAML file " << configFileName);

	  }
	}
	
	if((mpMap->getKeyFrameNum()))
	{
		int ID;
		if (ros::param::get("~BeginID", ID))
// 		if()
		{
			cout<<"Begin KeyFrame ID "<<ID<<endl;
			int idid = mpMap->getKeyFrameNum()-1;
			for(; idid>=0;idid--){
				if(mpMap->getKeyFrame(idid)->mnId == ID)
					break;
			}
			KeyFrame* beginKF = mpMap->getKeyFrame(idid);
			cout<<beginKF->mnId<<endl;
			cerr<<beginKF->getPose()<<endl;//haimei cun T
			TLocalicp = beginKF->getPose();
			mLocalMappoints = beginKF->mLocalMapPoints;
			mCurrentKeyFrame = beginKF;
			Initialized = true;
		}else
		{
			cerr<<"+++++++error error error"<<endl<<endl;
		}
	}else
	{
		TLocalicp =  PM::TransformationParameters::Identity(4,4);
	}
	

	localMapPub = n.advertise<sensor_msgs::PointCloud2>("slidingMap", 1, true);
	
}

LocalMapping::~LocalMapping()
{
	if(mLocalMappoints)
		delete mLocalMappoints;
}

void LocalMapping::SetIcpMapper(ICPMapper* pICPMapper)
{
	 mpTracker = pICPMapper;
}

void LocalMapping::SetLoopClosing(LoopClosing* pLoopClosing)
{
	mpLoopMapper = pLoopClosing;
}



void LocalMapping::Run()
{
	//Need to set rate
	//ros::Rate r(20);
	while(ros::ok())
	{
		if(CheckNewFrames())
		{
// 			SetAcceptFrames(false);//处理的时候先不接受插入，也可以不要，直接在程序里面加锁
			if(ProcessLocalMap())
			{
				CreateNewKeyFrames();
				
			}else{
			  
			}

			setProcessedOneFrame(true);
			
		}
	    clock_t   now   =   clock(); 

		while(   clock()   -   now   <   100   ); 
		
	}
}

void LocalMapping::RunSlidingMap()
{
  ros::Rate r(20);
  bSlidingMap = true;
  while(ros::ok()){
    if(CheckNewFrames()){
      ProcessSlidingMap();
    }
  }

}


bool LocalMapping::CheckNewFrames()
{
	 boost::mutex::scoped_lock lock(mMutexNewFrames);
	 return (!mlNewFrames.empty());
}


void LocalMapping::InsertFrame(Frame* pFrame)
{
	if(!isOKForInsertNewFrame())
		return;
	
	boost::mutex::scoped_lock lock(mMutexNewFrames);
	mlNewFrames.push_back(pFrame);
	
	if(mlNewFrames.size()>NumOfList)
		mlNewFrames.pop_front();
	
// 	cerr<<"[Local Mapping] Insert Frame"<<endl;

	
	

}

void LocalMapping::SetOkForCutList()
{
	boost::mutex::scoped_lock lock(mMutexStop);
	OkForCutList = true;
}


bool LocalMapping::AcceptFrames()
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    return mbAcceptKeyFrames;
}


void LocalMapping::SetAcceptFrames(bool flag)
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    mbAcceptKeyFrames=flag;

}

void LocalMapping::GoOn()
{
	 boost::mutex::scoped_lock lock(mMutexStop);
	 mbRequestStop = false;
}

void LocalMapping::RequstStop()
{
	 boost::mutex::scoped_lock lock(mMutexStop);
	 mbRequestStop = true;
}


bool LocalMapping::ProcessLocalMap()
{
// 	std::list<Frame* > mpNewFrames;
	{
		boost::mutex::scoped_lock lock(mMutexNewFrames);
		mLocalFrames.push_back(mlNewFrames.front());
		mlNewFrames.pop_front();
	}
// 	cerr<<"[Local Mapping] Process new frame"<<endl;
// 	DivideCloud(mLocalFrames.back());
// TEST  去掉LocalMap中第一帧的人
	if((mLocalFrames.size() == 1) && (!Initialized))
	{
		boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
		
		int mapCutPtsCount = 0; 
		int mapPtsCount = mLocalFrames.front()->mFrameMapPoints->features.cols();
		DP currentFramePoints = *(mLocalFrames.front()->mFrameMapPoints);
		
		
		DP mapLocalFrameCut(currentFramePoints.createSimilarEmpty());
		for(int i = 0; i<mapPtsCount; i++)
		{
			if(currentFramePoints.features.col(i).head(3).norm() > 1.0){
				mapLocalFrameCut.setColFrom(mapCutPtsCount, currentFramePoints, i);
				mapCutPtsCount++;
			}
		}

// 		cerr<<"ori mappoints "<<mapPtsCount<<" filtered points "<<mapCutPtsCount<<endl;
		mapLocalFrameCut.conservativeResize(mapCutPtsCount);
		
// 		mLocalMappoints =  new DP(mLocalFrames.front()->mFrameMapPoints->features,mLocalFrames.front()->mFrameMapPoints->featureLabels
// 			,mLocalFrames.front()->mFrameMapPoints->descriptors, mLocalFrames.front()->mFrameMapPoints->descriptorLabels);
		mLocalMappoints = new DP(mapLocalFrameCut.features, mapLocalFrameCut.featureLabels,mapLocalFrameCut.descriptors,mapLocalFrameCut.descriptorLabels);
		
// 		cout<<"???"<<mLocalMappoints->features.cols()<<endl;
		TLocalicp = mLocalFrames.front()->mLocalTicp;
// 		*mLocalMappoints = transformation->compute(*mLocalMappoints,TLocalicp );//current frame to world
		
		LocalMapUpdateForVisualisation = true;
		Initialized = true;
		mLocalFrames.back()->mMapPoints_no_overlap = *(mLocalFrames.back()->mFrameMapPoints);
		mLocalFrames.back()->mMapPoints_no_overlap.addDescriptor("probabilityStatic", PM::Matrix::Constant(1, mLocalFrames.back()->mMapPoints_no_overlap.features.cols(), priorStatic));

		mLocalFrames.back()->mMapPoints_no_overlap.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, mLocalFrames.back()->mMapPoints_no_overlap.features.cols(), priorDyn));
		mLocalFrames.back()->mMapPoints_no_overlap.addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, mLocalFrames.back()->mMapPoints_no_overlap.features.cols()));

// 		DP* tempDP = new DP(mLocalMappoints->features,mLocalMappoints->featureLabels	//new DP for KeyFrame Local Map
// 				,mLocalMappoints->descriptors, mLocalMappoints->descriptorLabels/*,
// 				mLocalMappoints->times, mLocalMappoints->timeLabels*/);
		mLocalFrames.front()->SetKF();
		KeyFrame* newKeyFrame = new KeyFrame(mLocalFrames.front()->mstamp,TLocalicp,mLocalFrames.front()->mnId);
		
		newKeyFrame->setGPS(mLocalFrames.front()->gps);
		newKeyFrame->mRelative_Pose = mLocalFrames.front()->mLocalTicp;

		
		mLocalFrames.front()->SetRelatedKFId(newKeyFrame->mnId);
		newKeyFrame->mLocalMapPoints = mLocalMappoints;
// 		mpMap->AddKeyFrame(newKeyFrame);
		mpTracker->ResetStopCreateNewKeyFrame();
		mCurrentKeyFrame = newKeyFrame;
// 		mCurrentKeyFrame->mLocalMapPoints = mLocalMappoints;

		return false;//first KeyFrame can not be passed to the loop closer
	}
	

	DivideCloud(mLocalFrames.back());
	if(mLocalFrames.back()->IsKF())
		return true;
	
	mLocalFrames.back()->SetRelatedKFId(mCurrentKeyFrame->mnId);
	
	//divide the new frame into overlap and nonoverlap
	


	//TODO when update the new local mapping method, this should be changed
// 	std::list<Frame* >::iterator mpNewFrames_it = mLocalFrames.begin();
// 	std::list<Frame* >::iterator mpNewFrames_it = mLocalFrames.end();
// 	mpNewFrames_it--;
	Frame* pCurrentF = mLocalFrames.back();

	{
		boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
// 		if (mLocalMappoints)
// 			delete mLocalMappoints;
// 		mLocalMappoints = tempDP;/////
		mLocalMappoints->concatenate((pCurrentF->mMapPoints_no_overlap));
// 		mapPostFilters.apply(*mLocalMappoints);

		mLocalstamp = mLocalFrames.back()->mstamp;
		localMapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mLocalMappoints, "/map", ros::Time::now()));



	}
// 	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TLocalicp, "/map", "/Local", ros::Time::now()));
// 	mCurrentFrame = *(mLocalFrames.end());
	if(pCurrentF->mnId != mpTracker->getPreviousFrameId())
	  pCurrentF->setProcessed(true);
	LocalMapUpdateForVisualisation = true;
	return false;
}

void LocalMapping::ProcessSlidingMap()
{
      {
	  boost::mutex::scoped_lock lock(mMutexNewFrames);
	  mLocalFrames.push_back(mlNewFrames.front());
	  mlNewFrames.pop_front();
      }
      
      if((mLocalFrames.size() == 1) && (!Initialized))
      {
	      boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
	      
	      int mapCutPtsCount = 0; int mapPtsCount = mLocalFrames.front()->mFrameMapPoints->features.cols();
	      DP currentFramePoints = *(mLocalFrames.front()->mFrameMapPoints);
	      DP mapLocalFrameCut(currentFramePoints.createSimilarEmpty());
	      for(int i = 0; i<mapPtsCount; i++)
	      {
		      if(currentFramePoints.features.col(i).head(3).norm() > 1.0){
			      mapLocalFrameCut.setColFrom(mapCutPtsCount, currentFramePoints, i);
			      mapCutPtsCount++;
		      }
	      }
// 		cerr<<"ori mappoints "<<mapPtsCount<<" filtered points "<<mapCutPtsCount<<endl;
	      mapLocalFrameCut.conservativeResize(mapCutPtsCount);
// 		mLocalMappoints =  new DP(mLocalFrames.front()->mFrameMapPoints->features,mLocalFrames.front()->mFrameMapPoints->featureLabels
// 			,mLocalFrames.front()->mFrameMapPoints->descriptors, mLocalFrames.front()->mFrameMapPoints->descriptorLabels);
	      mLocalMappoints = new DP(mapLocalFrameCut.features, mapLocalFrameCut.featureLabels,mapLocalFrameCut.descriptors,mapLocalFrameCut.descriptorLabels);
	      
	      TLocalicp = mLocalFrames.front()->mLocalTicp;
	      *mLocalMappoints = transformation->compute(*mLocalMappoints,TLocalicp );
	      
	      LocalMapUpdateForVisualisation = true;
	      Initialized = true;
	      mLocalFrames.back()->mMapPoints_no_overlap = *(mLocalFrames.back()->mFrameMapPoints);
	      mLocalFrames.back()->mMapPoints_no_overlap.addDescriptor("probabilityStatic", PM::Matrix::Constant(1, mLocalFrames.back()->mMapPoints_no_overlap.features.cols(), priorStatic));

	      mLocalFrames.back()->mMapPoints_no_overlap.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, mLocalFrames.back()->mMapPoints_no_overlap.features.cols(), priorDyn));
	      mLocalFrames.back()->mMapPoints_no_overlap.addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, mLocalFrames.back()->mMapPoints_no_overlap.features.cols()));

// 		DP* tempDP = new DP(mLocalMappoints->features,mLocalMappoints->featureLabels	//new DP for KeyFrame Local Map
// 				,mLocalMappoints->descriptors, mLocalMappoints->descriptorLabels/*,
// 				mLocalMappoints->times, mLocalMappoints->timeLabels*/);
	      mLocalFrames.front()->SetKF();
	      KeyFrame* newKeyFrame = new KeyFrame(mLocalFrames.front()->mstamp,TLocalicp,mLocalFrames.front()->mnId);
	      
	      newKeyFrame->setGPS(mLocalFrames.front()->gps);
	      newKeyFrame->mRelative_Pose = mLocalFrames.front()->mLocalTicp;

	      
	      mLocalFrames.front()->SetRelatedKFId(newKeyFrame->mnId);
		  newKeyFrame->mLocalMapPoints = mLocalMappoints;
// 	      mpMap->AddKeyFrame(newKeyFrame);
	      mpTracker->ResetStopCreateNewKeyFrame();
	      mCurrentKeyFrame = newKeyFrame;
		  mCurrentKeyFrame->mLocalMapPoints = mLocalMappoints;
	      return ;//first KeyFrame can not be passed to the loop closer
      }
      
      DivideCloud(mLocalFrames.back());
      
      Frame* pCurrentF = mLocalFrames.back();

	{
		boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
// 		if (mLocalMappoints)
// 			delete mLocalMappoints;
// 		mLocalMappoints = tempDP;/////
// 		LOG(INFO)<<"4";
		mLocalMappoints->concatenate((pCurrentF->mMapPoints_no_overlap));

		PM::TransformationParameters Ttemp = (pCurrentF->GetGlobalPose()).inverse();
		*mLocalMappoints = transformation->compute(*mLocalMappoints, Ttemp);
		maximumDistanceFilteres.apply(*mLocalMappoints);
		
		*mLocalMappoints = transformation->compute(*mLocalMappoints, pCurrentF->GetGlobalPose());

// 		LOG(INFO)<<"6";
		mLocalstamp = mLocalFrames.back()->mstamp;
		localMapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mLocalMappoints, "/map", ros::Time::now()));

	}
// 	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TLocalicp, "/map", "/Local", ros::Time::now()));
// 	mCurrentFrame = *(mLocalFrames.end());
	if(pCurrentF->mnId != mpTracker->getPreviousFrameId())
	  pCurrentF->setProcessed(true);
	LocalMapUpdateForVisualisation = true;
	mpTracker->OkForProcessNewFrame();
	return ;
}


void LocalMapping::DivideCloud(Frame* pFrame)
{
	if(mLocalFrames.size() == 1)
	{
		pFrame->mMapPoints_no_overlap = *(pFrame->mFrameMapPoints);

		cerr<<"[Divide Cloud wrong]"<<endl;
		return;
	}
	

	int tempMapPts;
	{
		boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
		tempMapPts = mLocalMappoints->features.cols();
	}
	const int readPtsCount(pFrame->mFrameMapPoints->features.cols());
	const int mapPtsCount(tempMapPts);
	
	// Build a range image of the reading point cloud (local coordinates)
	PM::Matrix radius_reading = pFrame->mFrameMapPoints->features.topRows(3).colwise().norm();//1*cols的矩阵？，每个元素是每个点的距离

	PM::Matrix angles_reading(2, readPtsCount); // 0=inclination, 1=azimuth

	// No atan in Eigen, so we are for to loop through it...
	//构造newPointloud的angles_reading
	for(int i=0; i<readPtsCount; i++)
	{
		const float ratio = pFrame->mFrameMapPoints->features(2,i)/radius_reading(0,i);//z值和距离的比
		angles_reading(0,i) = acos(ratio);
		angles_reading(1,i) = atan2(pFrame->mFrameMapPoints->features(1,i), pFrame->mFrameMapPoints->features(0,i));//newPointCloud是输入的scanner坐标系下的点云
	}

	std::shared_ptr<NNS> featureNNS;
	featureNNS.reset( NNS::create(angles_reading));

	// Transform the local map in current frame coordinates
// 	DP mapLocalFrame = transformation->compute(*mLocalMappoints, (pFrame->mTicp *TLocalicp.inv() ).inv());
	PM::TransformationParameters Ttemp = (pFrame->mLocalTicp).inverse();
	DP mapLocalFrame = transformation->compute(*mLocalMappoints, Ttemp);

	// Remove points out of sensor range
	// FIXME: this is a parameter
// 	const float sensorMaxRange = 100.0;
	
	PM::Matrix globalId(1, mapPtsCount); //地图点数量  关联mapLocalFrameCut里面的点和地图点的index

	int mapCutPtsCount = 0;//有多少地图点在当前帧能看到的范围内
	DP mapLocalFrameCut(mapLocalFrame.createSimilarEmpty());//mapLocalFrame, 将map地图点换到sensor坐标系
// 	float sensorMaxRange = 100.0;
	for (int i = 0; i < mapPtsCount; i++)	
	{
		if (mapLocalFrame.features.col(i).head(3).norm() < sensorMaxRange)
		{
			mapLocalFrameCut.setColFrom(mapCutPtsCount, mapLocalFrame, i);
			globalId(0,mapCutPtsCount) = i;//cut 里面的第i个点对应local里面的第j个点
			mapCutPtsCount++;
		}
	}
	mapLocalFrameCut.conservativeResize(mapCutPtsCount);//将后面的零元素resize()
	


	
	PM::Matrix radius_map = mapLocalFrameCut.features.topRows(3).colwise().norm();	//1*cols的矩阵

	PM::Matrix angles_map(2, mapCutPtsCount); // 0=inclination, 1=azimuth

	// No atan in Eigen, so we are for to loop through it...
	for(int i=0; i<mapCutPtsCount; i++)
	{
		const float ratio = mapLocalFrameCut.features(2,i)/radius_map(0,i);
		//if(ratio < -1 || ratio > 1)
			//cout << "Error angle!" << endl;
		angles_map(0,i) = acos(ratio);
		angles_map(1,i) = atan2(mapLocalFrameCut.features(1,i), mapLocalFrameCut.features(0,i));
	}

	// Look for NN in spherical coordinates
	Matches::Dists dists(1,mapCutPtsCount);
	Matches::Ids ids(1,mapCutPtsCount);
	
	// FIXME: those are parameters
	// note: 0.08 rad is 5 deg
	//const float maxAngle = 0.04; // in rad (ICRA 14)
	//const float eps_a = 0.2; // ratio of distance (ICRA 14)
	//const float eps_d = 0.1; // in meters (ICRA 14)
// 	const float maxAngle = 0.01; // in rad (ISER 14)
// 	const float eps_a = 0.1; // ratio of distance (ISER 14)
// 	const float eps_d = 0.1; // in meters (ISER 14)
// 	const float alpha = 0.99;
// 	const float beta = 0.9;
// 	const float eps = 0.0001;

	
	
	featureNNS->knn(angles_map, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH, maxAngle);

	// Define views on descriptors
	//DP::View viewOn_normals_overlap = newPointCloud->getDescriptorViewByName("normals");
	//DP::View viewOn_obsDir_overlap = newPointCloud->getDescriptorViewByName("observationDirections");
	DP::View viewOn_Msec_overlap = pFrame->mFrameMapPoints->getDescriptorViewByName("stamps_Msec");
	DP::View viewOn_sec_overlap = pFrame->mFrameMapPoints->getDescriptorViewByName("stamps_sec");
	DP::View viewOn_nsec_overlap = pFrame->mFrameMapPoints->getDescriptorViewByName("stamps_nsec");
	
	{
		boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
		DP::View viewOnProbabilityStatic = mLocalMappoints->getDescriptorViewByName("probabilityStatic");
		DP::View viewOnProbabilityDynamic = mLocalMappoints->getDescriptorViewByName("probabilityDynamic");
		DP::View viewOnDynamicRatio = mLocalMappoints->getDescriptorViewByName("dynamic_ratio");

		DP::View viewOn_normals_map = mLocalMappoints->getDescriptorViewByName("normals");
		DP::View viewOn_Msec_map = mLocalMappoints->getDescriptorViewByName("stamps_Msec");
		DP::View viewOn_sec_map = mLocalMappoints->getDescriptorViewByName("stamps_sec");
		DP::View viewOn_nsec_map = mLocalMappoints->getDescriptorViewByName("stamps_nsec");



	viewOnDynamicRatio = PM::Matrix::Zero(1, mapPtsCount);
	for(int i=0; i < mapCutPtsCount; i++)
	{
		if(dists(i) != numeric_limits<float>::infinity())//scanner坐标系下的点云在世界地图中有匹配的点
		{
			const int readId = ids(0,i);//ids是knn的结果
			const int mapId = globalId(0,i);
			
			// in local coordinates
			const Eigen::Vector3f readPt = pFrame->mFrameMapPoints->features.col(readId).head(3);
			const Eigen::Vector3f mapPt = mapLocalFrameCut.features.col(i).head(3);
			const Eigen::Vector3f mapPt_n = mapPt.normalized();
			const float delta = (readPt - mapPt).norm(); //角度相近的匹配点，其欧式空间的距离误差
			const float d_max = eps_a * readPt.norm();

			//const double timeMap = viewOn_Msec_map(0,mapId)*10e5 + viewOn_sec_map(0,mapId) + viewOn_nsec_map(0,mapId)*10e-10;
			//const double timeRead = viewOn_Msec_overlap(0,readId)*10e5 + viewOn_sec_overlap(0,readId) + viewOn_nsec_overlap(0,readId)*10e-10;
			//const double deltaTime = timeRead - timeMap;

			const Eigen::Vector3f normal_map = viewOn_normals_map.col(mapId); //原始地图下的normal
			//const Eigen::Vector3f normal_read = viewOn_normals_overlap.col(readId);
			//const Eigen::Vector3f obsDir = viewOn_obsDir_overlap.col(readId);
			//const float viewAngle = acos(normal_map.normalized().dot(obsDir.normalized()));
			//const float normalAngle = acos(normal_map.normalized().dot(normal_read.normalized()));
			
			// Weight for dynamic elements
			const float w_v = eps + (1 - eps)*fabs(normal_map.dot(mapPt_n));
			//const float w_d1 = 1 + eps - acos(readPt.normalized().dot(mapPt_n))/maxAngle;
// 			const float w_d1 =  eps + (1 - eps)*(1 - sqrt(dists(i))/maxAngle);
			const float w_d1 =  eps + (1 - eps)*( sqrt(dists(i))/maxAngle);
			
			
			const float offset = delta - eps_d;
			float w_d2 = 1;
			if(delta < eps_d || mapPt.norm() > readPt.norm())
			{
				w_d2 = eps;
			}
			else 
			{
				if (offset < d_max)
				{
					w_d2 = eps + (1 - eps )*offset/d_max;
				}
			}

			float w_p2 = eps;
			if(delta < eps_d)
			{
				w_p2 = 1;
			}
			else
			{
				if(offset < d_max)
				{
					w_p2 = eps + (1 - eps)*(1 - offset/d_max);
				}
			}

			//cerr << "readPt.norm(): "<< readPt.norm()  << "mapPt.norm(): "<< mapPt.norm() << ", w_p2: " << w_p2 << ", w_d2: " << w_d2 << endl;
		

			// We don't update point behind the reading
			if((readPt.norm() + eps_d + d_max) >= mapPt.norm())
			{
				const float lastDyn = viewOnProbabilityDynamic(0,mapId);
				const float lastStatic = viewOnProbabilityStatic(0, mapId);

				const float c1 = (1 - (w_v*(1 - w_d1)));
				const float c2 = w_v*(1 - w_d1);
				

				//viewOnProbabilityDynamic(0,mapId) += (w_v + w_d2) * w_d1/2;
				//viewOnProbabilityDynamic(0,mapId) += (w_v * w_d2);
				
				//viewOnProbabilityStatic(0, mapId) += (w_p2) * w_d1;
				//viewOnProbabilityStatic(0, mapId) += w_p2;

				// FIXME: this is a parameter
				const float maxDyn = 0.9; // ICRA 14
// 				const float maxDyn = 0.98; // ISER 14

				//Lock dynamic point to stay dynamic under a threshold
				if(lastDyn < maxDyn)
				{
					viewOnProbabilityDynamic(0,mapId) = c1*lastDyn + c2*w_d2*((1 - alpha)*lastStatic + beta*lastDyn);
					viewOnProbabilityStatic(0, mapId) = c1*lastStatic + c2*w_p2*(alpha*lastStatic + (1 - beta)*lastDyn);
				}
				else
				{
					viewOnProbabilityStatic(0,mapId) = eps;
					viewOnProbabilityDynamic(0,mapId) = 1-eps;
				}
				
				
				
				// normalization
				const float sumZ = viewOnProbabilityDynamic(0,mapId) + viewOnProbabilityStatic(0, mapId);
				assert(sumZ >= eps);	
				
				viewOnProbabilityDynamic(0,mapId) /= sumZ;
				
				viewOnProbabilityStatic(0,mapId) /= sumZ;
				
				
				//viewOnDynamicRatio(0,mapId) =viewOnProbabilityDynamic(0, mapId);
				viewOnDynamicRatio(0,mapId) = w_d2;
				

				//viewOnDynamicRatio(0,mapId) =	w_d2;

				// Refresh time
				viewOn_Msec_map(0,mapId) = viewOn_Msec_overlap(0,readId);	
				viewOn_sec_map(0,mapId) = viewOn_sec_overlap(0,readId);	
				viewOn_nsec_map(0,mapId) = viewOn_nsec_overlap(0,readId);	
				
				
			}


		}
	}
	
	mLocalMappoints->addDescriptor("probabilityDynamic", viewOnProbabilityDynamic);
	mLocalMappoints->addDescriptor("probabilityStatic",viewOnProbabilityStatic);
	mLocalMappoints->addDescriptor("dynamic_ratio", viewOnDynamicRatio);
	mLocalMappoints->addDescriptor("stamps_Msec", viewOn_Msec_map);
	mLocalMappoints->addDescriptor("stamps_sec",viewOn_sec_map);
	mLocalMappoints->addDescriptor("stamps_nsec",viewOn_nsec_map);

	}
// 	LOG(INFO)<<"3";
	
	DP tmp_map = mapLocalFrameCut; // FIXME: this should be on mapLocalFrameCut
	mapLocalFrameCut.concatenate(*(pFrame->mFrameMapPoints));//******注意！加点不一样了//

	// build and populate NNS
	featureNNS.reset( NNS::create(mapLocalFrameCut.features, mapLocalFrameCut.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));	
	PM::Matches matches_overlap(
		Matches::Dists(1, readPtsCount),
		Matches::Ids(1, readPtsCount)
	);
	featureNNS->knn(pFrame->mFrameMapPoints->features, matches_overlap.ids, matches_overlap.dists, 1, 0);
	
	DP overlap(pFrame->mFrameMapPoints->createSimilarEmpty());
	DP no_overlap(pFrame->mFrameMapPoints->createSimilarEmpty());


	// FIXME: this is a parameter
//   const float maxDist = pow(0.3, 2); // ICRA 2014
	//const float maxDist = pow(0.1, 2); // ISER 2014
	//const float maxDist = pow(0.1, 2);
// 	const float maxDistNewPoint = pow(0.7,2);

	int ptsOut = 0;
	int ptsIn = 0;
// 	float minAddDist = 2.0;
	
	int r_row, g_row, b_row;
	int r_row_frame, b_row_frame, g_row_frame;
	bool use_color = mLocalMappoints->descriptorExists("r");
	
	if(use_color){
	  {
	    boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
	    r_row = mLocalMappoints->getDescriptorStartingRow("r");
	    g_row = mLocalMappoints->getDescriptorStartingRow("g");
	    b_row = mLocalMappoints->getDescriptorStartingRow("b");
	    
	  }
	  
	  r_row_frame = pFrame->mFrameMapPoints->getDescriptorStartingRow("r");
	  g_row_frame = pFrame->mFrameMapPoints->getDescriptorStartingRow("g");
	  b_row_frame = pFrame->mFrameMapPoints->getDescriptorStartingRow("b");
	}
	
	for (int i = 0; i < readPtsCount; ++i)
	{
		// TEST 将激光周围1m范围的点不加入地图中
		if (((matches_overlap.dists(i) > maxDistNewPoint) && (pFrame->mFrameMapPoints->features.col(i).head(2).norm() > minAddDist))
			||(( pFrame->mFrameMapPoints->features(2,i) < -robot_height)&& (matches_overlap.dists(i) > maxDistNewPoint/5.0) )
		)
		{
			no_overlap.setColFrom(ptsOut, *(pFrame->mFrameMapPoints), i);
			ptsOut++;
		}
		else
		{
			overlap.setColFrom(ptsIn, *(pFrame->mFrameMapPoints), i);
			ptsIn++;
			
			//更新颜色
			if(use_color){
			  if(matches_overlap.ids(i) < mapCutPtsCount)
			  {
			    int globelid = globalId(0,matches_overlap.ids(i));
			    boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
			    
			    if((mLocalMappoints->descriptors(r_row, globelid) == 0) && 
			      (mLocalMappoints->descriptors(g_row, globelid) == 0) &&
			      (mLocalMappoints->descriptors(b_row, globelid) == 0) 
			    ){
			      mLocalMappoints->descriptors(r_row, globelid) = pFrame->mFrameMapPoints->descriptors(r_row_frame, i);
			      mLocalMappoints->descriptors(g_row, globelid) = pFrame->mFrameMapPoints->descriptors(g_row_frame, i);
			      mLocalMappoints->descriptors(b_row, globelid) = pFrame->mFrameMapPoints->descriptors(b_row_frame, i);
			    }
			  }
			}
		}
	}

	no_overlap.conservativeResize(ptsOut);
	overlap.conservativeResize(ptsIn);
// 	cout<<"\033[31m"<<ptsOut<<"\033[0m"<<endl;

	
	// Initialize descriptors
	//no_overlap.addDescriptor("probabilityStatic", PM::Matrix::Zero(1, no_overlap.features.cols()));
	no_overlap.addDescriptor("probabilityStatic", PM::Matrix::Constant(1, no_overlap.features.cols(), priorStatic));
	//no_overlap.addDescriptor("probabilityDynamic", PM::Matrix::Zero(1, no_overlap.features.cols()));
	no_overlap.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, no_overlap.features.cols(), priorDyn));
	no_overlap.addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, no_overlap.features.cols()));
	
	overlap.addDescriptor("probabilityStatic", PM::Matrix::Constant(1, overlap.features.cols(), priorStatic));
	overlap.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, overlap.features.cols(), priorDyn));
	overlap.addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, overlap.features.cols()));

// 	cout<<pFrame->mLocalTicp<<endl;
	pFrame->mMapPoints_no_overlap = transformation->compute(no_overlap,pFrame->mLocalTicp);

}

int LocalMapping::getLocalMapPointsNumber()
{
	boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
	return (int)(Initialized);
}

PointMatcher< float >::TransformationParameters LocalMapping::getLocalTLocalICP()
{
	boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
	return TLocalicp;
}

void LocalMapping::getLocalMapPointsNew(LocalMapping::DP& copyLocalMapPoints)
{
	boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
	copyLocalMapPoints = DP(mLocalMappoints->features,mLocalMappoints->featureLabels
			,mLocalMappoints->descriptors, mLocalMappoints->descriptorLabels/*,
			mLocalMappoints->times, mLocalMappoints->timeLabels*/);
}

void LocalMapping::CreateNewKeyFrames()
{
	//是不是还是需要加上frame计数，不然慢慢开。。。
	//可以和上一个keyframe的距离比较。。。存list吧傻逼
	StopInsertNewFrame();
	cerr<<"CREATING NEW KEY FREAM"<<endl;
	KeyFrame* newKeyFrame;
	Frame* temp ;

	{
	boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);


	temp = mLocalFrames.back();
	mCurrentKeyFrame->mLocalMapPoints = mLocalMappoints;
	KFinLoopProcessing = mCurrentKeyFrame;

// 	//TEST 将第一帧中的近距离点删去
// 	int mapCutPtsCount = 0; int mapPtsCount = temp->mFrameMapPoints->features.cols();
// 	DP currentFramePoints = *(temp->mFrameMapPoints);
// 	DP mapLocalFrameCut(currentFramePoints.createSimilarEmpty());
// 	for(int i = 0; i<mapPtsCount; i++)
// 	{
// 		if(currentFramePoints.features.col(i).head(3).norm() > 2.0){
// 			mapLocalFrameCut.setColFrom(mapCutPtsCount, currentFramePoints, i);
// 			mapCutPtsCount++;
// 		}
// 	}
// 	cerr<<"ori mappoints "<<mapPtsCount<<" filtered points "<<mapCutPtsCount<<endl;
// 	mapLocalFrameCut.conservativeResize(mapCutPtsCount);
// // 	mLocalMappoints = new DP(temp->mFrameMapPoints->features,temp->mFrameMapPoints->featureLabels	//new DP for KeyFrame Local Map
// // 		,temp->mFrameMapPoints->descriptors, temp->mFrameMapPoints->descriptorLabels/*,
// // 		temp->mFrameMapPoints->times, temp->mFrameMapPoints->timeLabels*/);
// 	
// 	
	// keep some points from last keyframe
	
	int mapCutPtsCount = 0; 
	DP mapLocalFrameCurrent = transformation->compute(*mLocalMappoints, (temp->mLocalTicp).inverse());//当前帧啦
	int mapPtsCount = mapLocalFrameCurrent.features.cols();
	DP mapLocalFrameCut(mapLocalFrameCurrent.createSimilarEmpty());
	PM::Matrix radius_reading = mapLocalFrameCurrent.features.topRows(3).colwise().norm();
	for(int i = 0; i<mapPtsCount; i++){
		if(radius_reading(0,i)<20){
			mapLocalFrameCut.setColFrom(mapCutPtsCount, mapLocalFrameCurrent, i);
			mapCutPtsCount++;
		}
	}
	cerr<<"new keyframe "<<mapLocalFrameCut.features.cols()<<endl;
	mapLocalFrameCut.conservativeResize(mapCutPtsCount);
// 	mapLocalFrameCut.concatenate(transformation->compute(temp->mMapPoints_no_overlap,  (temp->mLocalTicp).inverse()));//keep all the points
	mapLocalFrameCut.concatenate(*(temp->mFrameMapPoints));//keep all the points
	
	mLocalMappoints = new DP(mapLocalFrameCut.features, mapLocalFrameCut.featureLabels,mapLocalFrameCut.descriptors,mapLocalFrameCut.descriptorLabels);

	TLocalicp =TLocalicp*( temp->mLocalTicp);
	mpTracker->OkForProcessNewFrame();

	newKeyFrame = new KeyFrame(temp->mstamp,TLocalicp,temp->mnId);//////////////
	newKeyFrame->setGPS(temp->gps);

	newKeyFrame->mRelative_Pose = temp->mLocalTicp;

	newKeyFrame->preKFId = mCurrentKeyFrame->mnId;

	std::pair<long unsigned int, PM::TransformationParameters> neighbor( mCurrentKeyFrame->mnId, temp->mLocalTicp.inverse());
	std::pair<long unsigned int, bool> neighbor_reachable(mCurrentKeyFrame->mnId, false);
	std::pair<long unsigned int, PM::TransformationParameters> neighborinv( newKeyFrame->mnId, (temp->mLocalTicp));
	std::pair<long unsigned int, bool> neighborinv_reachable(newKeyFrame->mnId, true);
	newKeyFrame->neighbours.insert(neighbor);
	newKeyFrame->neighbours_isReachable.insert(neighbor_reachable);
	mCurrentKeyFrame->neighbours.insert(neighborinv);
	mCurrentKeyFrame->neighbours_isReachable.insert(neighborinv_reachable);
	
	temp->SetRelatedKFId(newKeyFrame->mnId);

	mpMap->AddKeyFrame(mCurrentKeyFrame);


	KeyFrame* tempKF;
	tempKF = mCurrentKeyFrame;

// 	mpMap->AddKeyFrame(mCurrentKeyFrame);
	mpLoopMapper->InsertKeyFrame(tempKF);//
	mpLoopMapper->run_offline();


	mCurrentKeyFrame = newKeyFrame;
// 	mpTracker->SetStopCreateNewKeyFrame();

	mLocalFrames.clear();
	mlNewFrames.clear();
	mLocalFrames.push_back(temp);
	}
// 	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TLocalicp, "/map", "/Local", ros::Time::now()));
	LocalMapUpdateForVisualisation = true;
	OKInsertNewFrame();
	
}


void LocalMapping::getLocalMapTime(ros::Time& time)
{
	boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
	time = mLocalstamp;
}

void LocalMapping::StopInsertNewFrame()
{
	boost::mutex::scoped_lock lock(mMutexNewFrames);
	OkForInsertNewFrame = false;
}

void LocalMapping::OKInsertNewFrame()
{
	boost::mutex::scoped_lock lock(mMutexNewFrames);
	OkForInsertNewFrame = true;
}

bool LocalMapping::isOKForInsertNewFrame()
{
	boost::mutex::scoped_lock lock(mMutexNewFrames);
	return OkForInsertNewFrame;
}

void LocalMapping::UPdateTLocalICP(int KFId)
{
  {
	  cerr<<"Correct KeyFrames from "<<KFId<<endl;
	    boost::mutex::scoped_lock lock(mMutexNewLocalMapPoints);
	    mpTracker->SetStopCreateNewKeyFrame();
	    
	    std::vector<KeyFrame*> allKeyFrames;
	      cerr<<"Waiting to get  All KeyFrames"<<endl;
	    mpMap->getAllKeyFrames(allKeyFrames);
	      cerr<<"Get All KeyFrames, size is "<<allKeyFrames.size()<<endl;

// 	    int i = KFId;
	    int i = allKeyFrames.size()-1;
	    for(; i>=0; i--){
		    cerr<<i<<endl;
		    if(allKeyFrames[i]->mnId == KFId)
			    break;
	    }
	    
	    PM::TransformationParameters OptimizedT = allKeyFrames[i++]->getPose();
    // 	PM::TransformationParameters tempT;
	    allKeyFrames.push_back(mCurrentKeyFrame);
	   
	    for(; i<allKeyFrames.size(); i++)
	    {
	      cerr<<i<<endl;
		  OptimizedT = OptimizedT * allKeyFrames[i]->mRelative_Pose;
		  allKeyFrames[i]->OptimizedPose(OptimizedT);
	    }
	    mpTracker->ResetStopCreateNewKeyFrame();
	    TLocalicp = mCurrentKeyFrame->getPose();
	}
	
// 	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TLocalicp, "/map", "/Local", ros::Time::now()));
}
	
void LocalMapping::Stopping()
{
	mCurrentKeyFrame->mLocalMapPoints = mLocalMappoints;
	mpMap->AddKeyFrame(mCurrentKeyFrame);
}



bool LocalMapping::initializationforReSLAM()
{
	int currentKFID;
	if (ros::param::get(std::string("~reSLAM_refKFID"), currentKFID))
	{
		ROS_INFO_STREAM("---ReSLAM, current KF ID w.r.t. " << currentKFID);
		KeyFrame* referenceKF = mpMap->getKeyFrame(currentKFID);
		cout<<"1"<<endl;
		mLocalMappoints = referenceKF->mLocalMapPoints;
		TLocalicp = referenceKF->getPose();
		mCurrentKeyFrame = new KeyFrame(ros::Time::now(),TLocalicp,0);//frame_id begin from 0
		PM::TransformationParameters identity = PM::TransformationParameters::Identity(4,4);
		mCurrentKeyFrame->mRelative_Pose = PM::TransformationParameters::Identity(4,4);
		mCurrentKeyFrame->preKFId = referenceKF->mnId;
		
		cout<<"1"<<endl;
		std::pair<long unsigned int, PM::TransformationParameters> neighbor( mCurrentKeyFrame->mnId, identity);
		std::pair<long unsigned int, bool> neighbor_reachable(mCurrentKeyFrame->mnId, true);
		std::pair<long unsigned int, PM::TransformationParameters> neighborinv( referenceKF->mnId, identity);
		std::pair<long unsigned int, bool> neighborinv_reachable(referenceKF->mnId, true);
		
		mCurrentKeyFrame->neighbours.insert(neighborinv);
		mCurrentKeyFrame->neighbours_isReachable.insert(neighborinv_reachable);
		referenceKF->neighbours.insert(neighbor);
		referenceKF->neighbours_isReachable.insert(neighbor_reachable);
		
		Initialized = true;
		
		return true;
	}else{
		ROS_ERROR_STREAM("something wrong with param 'reSLAM_refKFID', please set it...");
		return false;
	}

}

	bool LocalMapping::hasProcessedOneFrame(){
		boost::mutex::scoped_lock lock(mMutexprocessedOncFrame);
		return processedOneFrame;
	}
	void LocalMapping::setProcessedOneFrame(bool flag){
		boost::mutex::scoped_lock lock(mMutexprocessedOncFrame);
		processedOneFrame = flag;
	}


}
