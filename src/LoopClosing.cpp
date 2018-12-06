#include "LoopClosing.h"
#include "fstream"
// #include "g2o/core/base_edge.h"
#include "pointmatcher/Timer.h"

#include "ros/ros.h"

using namespace PointMatcherSupport;
namespace Velodyne_SLAM {

LoopClosing::LoopClosing()
{
	isUpdate = false;
// 	opitmizer = new Optimizer();
}

LoopClosing::LoopClosing(Map* pMap):mpMap(pMap),mbResetRequested(false),mLastLoopKFTime(0),mLastLoopKFid(0)
{
	mpMatchedKF = NULL;
	NumOfClosure = 0;
	isUpdate = false;
	// load configs

	
	string configFileName;
	if (ros::param::get("~icpConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);//ICPChainBase function
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icp.setDefault();
		}
	}
	else
	{
		ROS_INFO_STREAM("No ICP config file given, using $(find velodyne_slam)/cfg/input_filters.yaml");
		configFileName =ros::package::getPath("laser_mapping")+ "/cfg/icp_dynamic_loop_closing.yaml";
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);//ICPChainBase function
		}else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icp.setDefault();
		}
		
	}
// 	int time = (int )ros::Time::now().sec;
// 	stringstream s ;
// 	s<<time;
// 
// 	filename = ros::package::getPath("laser_slam") +"/Log/LoopClosing_Log"+s.str();+".txt";  
// 	fLoopClosing.open(filename);
	beginKFId = mpMap->getKeyFrameNum();

}

void LoopClosing::SetIcpMapper(ICPMapper* pICPMapper)
{
	mpTracker = pICPMapper;
}

void LoopClosing::SetLocalMapper(LocalMapping* pLocalMapper)
{
	mpLocalMapper = pLocalMapper;
}


void LoopClosing::Run()
{
	ros::Rate r(10);
	//TEST
	while(ros::ok())
	{
		//Detect candidate 
		if(CheckNewKeyFrames())
		{
// 		  cerr<<"[Loop Closing]"<<endl;
			if(DetectLoop())
			{
				
			}	
			mLastProcessedKFId = mpCurrentKF->mnFrameId;
// 		  mpMap->AddKeyFrame(mpCurrentKF);
		}

// 			mpTracker->ResetStopCreateNewKeyFrame();
	    clock_t   now   =   clock(); 

		while(   clock()   -   now   <   100   ); 
	}
}

bool LoopClosing::CheckNewKeyFrames()
{
	boost::mutex::scoped_lock lock(mMutexLoopQueue);
	return (!(mlpLoopKeyFrameQueue.empty()));
}

void LoopClosing::InsertKeyFrame(KeyFrame* pKF)
{
	boost::mutex::scoped_lock lock(mMutexLoopQueue);
	if(pKF->mnId != 0)
	{
// 	  cerr<<"[loop closing]<<InsertKeyFrame"<<endl;
		mlpLoopKeyFrameQueue.push_back(pKF);
		if(mlpLoopKeyFrameQueue.size()>5)
			mlpLoopKeyFrameQueue.pop_front();
// 		mpLocalMapper->RequstStop();
	}
}

long unsigned int LoopClosing::GetFirstKFId()
{
	boost::mutex::scoped_lock lock(mMutexLoopQueue);
	return mLastProcessedKFId;
}



bool LoopClosing::DetectLoop()
{
    {
        boost::mutex::scoped_lock lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
	
    }
    
    
//     fLoopClosing<<"the "<<mpCurrentKF->mnId<<" KeyFrame"<<endl;
    if((mpCurrentKF->mstamp.sec < mLastLoopKFTime+10) || (mpCurrentKF->mnId < mLastLoopKFid+2))	//TODO:need to be considered
    {
// 	    mpLocalMapper->GoOn();
	    cerr<<"[Loop Closing]time"<<(mpCurrentKF->mstamp.sec-mLastLoopKFTime)<<endl;
	    return false;//TEST
    }
    
    timer t;
    
    
    PM::TransformationParameters TCurrent = mpCurrentKF->getPose();
    vector<KeyFrame* > vpAllCandidates;    
    mpMap->getAllKeyFrames(vpAllCandidates);
    
    vector<KeyFrame*> InDistCandidates;
    vector<KeyFrame*>::iterator vpAllCandidates_it = vpAllCandidates.begin() + beginKFId;
    
    
    for(; vpAllCandidates_it != vpAllCandidates.end(); vpAllCandidates_it++)
    {
	    if(((*vpAllCandidates_it)->mnId + 5) > mpCurrentKF->mnId)//TEST
		    break;
	    PM::TransformationParameters TCandidate = (*vpAllCandidates_it)->getPose();
	    Eigen::Vector2f delta_t = TCandidate.col(3).head(2) - TCurrent.col(3).head(2);
	    if((delta_t.norm()<20))
	    {
		    InDistCandidates.push_back(*vpAllCandidates_it);
		}
	}
	
	if(InDistCandidates.size()==0)
	{
// 		mpLocalMapper->GoOn();
// 	  cerr<<"[loop closer] No Candidate"<<t.elapsed()<<endl;
		return false;
	}
	
// 	fLoopClosing << "Begin the"<< NumOfClosure<<"th optimization with "<<InDistCandidates.size()<<" Candidates"<<std::endl;
// 	icp.setMap(*(mpCurrentKF->mFrameMapPoints));
	icp.setMap(*(mpCurrentKF->mLocalMapPoints));
	vector<KeyFrame* >::iterator vpInDistCandidates_it = InDistCandidates.begin();
	double maxOverlap = 0;
	KeyFrame* bestCandidate;
	PM::TransformationParameters TCandidateToCurrentICP;
	PM::TransformationParameters TCu_Can;
	PM::TransformationParameters bestT;
	PM::TransformationParameters bestComT;
	for(;vpInDistCandidates_it != InDistCandidates.end(); vpInDistCandidates_it++)
	{
		try{
		  
		  TCu_Can = (TCurrent.inverse())*( (*vpInDistCandidates_it)->getPose());

		TCandidateToCurrentICP = icp(*((*vpInDistCandidates_it)->mLocalMapPoints), TCu_Can);
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		
		

		
		if(estimatedOverlap > maxOverlap)
		{
			maxOverlap = estimatedOverlap;
			bestCandidate = (*vpInDistCandidates_it);
			bestT = TCandidateToCurrentICP;
			bestComT = TCu_Can;
		}
		}catch (PM::ConvergenceError error)
		{
			ROS_ERROR_STREAM("LOOP_CLOSING ICP failed to converge: " << error.what());
		}
	}
	cerr<<"[loop closer] After loop icp"<<t.elapsed()<<" In Candidate "<<InDistCandidates.size()<<endl;
	
	if((maxOverlap<0.5))
	{
// 		fLoopClosing<<"fail to overlap, InDistCandidates.size() = "<<InDistCandidates.size()<<endl;
// 		mpLocalMapper->GoOn();
		return false;
	}
// 	pair<long unsigned int,long unsigned int> tpair(bestCandidate->mnId,mpCurrentKF->mnId);
// 	pair<long unsigned int,long unsigned int> tpair(mpCurrentKF->mnId, bestCandidate->mnId);
// 	mpLoopKF.insert(pair<pair<long unsigned int,long unsigned int>, PM::TransformationParameters>(tpair,bestT));//save all the loop edges

	
	std::pair<long unsigned int, PM::TransformationParameters> neighbor( bestCandidate->mnId,bestT);
	std::pair<long unsigned int, PM::TransformationParameters> neighborinv( mpCurrentKF->mnId,bestT.inverse());
	std::pair<long unsigned int, bool> neighbor_reachable(mpCurrentKF->mnId, false);
	std::pair<long unsigned int, bool> neighborinv_reachable(bestCandidate->mnId, true);
	mpCurrentKF->neighbours.insert(neighbor);
	mpCurrentKF->neighbours_isReachable.insert(neighborinv_reachable);
	bestCandidate->neighbours.insert(neighborinv);
	bestCandidate->neighbours_isReachable.insert(neighbor_reachable);
	
	stringstream ss ;
	ss<<(NumOfClosure+1);

	timer to;
	Optimizer optimizer;
// 	optimizer.OptimizeGraph(mpMap, bestCandidate, mpCurrentKF,NumOfClosure++,icp);
// 	optimizer.OptimizeGraph(mpMap,mpLoopKF,NumOfClosure++,icp);
	//TEST
	optimizer.OptimizeGraph(mpMap,NumOfClosure++, icp);
	
	mpLocalMapper->UPdateTLocalICP(mpCurrentKF->mnId);
	isUpdate = true;

	cerr<<"[loop closer] Optimize takes "<<t.elapsed()<<endl;
	
	


    
	mLastLoopKFid = mpCurrentKF->mnId;
	mLastLoopKFTime = mpCurrentKF->mstamp.sec;

	

}

 

	

}