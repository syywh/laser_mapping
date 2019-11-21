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
	
	
// distanceCheck(getParam<double>("distance_check", 20));
// thread_num(getParam<int >("thread_num", 8));
// icpOverlap(getParam<double>("icp_overlap", 0.6));
	
	distanceCheck = 20;
	thread_num = 8;
	icpOverlap = 0.3;
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

void LoopClosing::run_offline()
{
	DetectLoop();
	mLastProcessedKFId = mpCurrentKF->mnFrameId;

}


bool LoopClosing::CheckNewKeyFrames()
{
	boost::mutex::scoped_lock lock(mMutexLoopQueue);
	return (!(mlpLoopKeyFrameQueue.empty()));
}

void LoopClosing::InsertKeyFrame(KeyFrame* pKF)
{
	boost::mutex::scoped_lock lock(mMutexLoopQueue);
	
// 	if(pKF->mnId != 0)
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
		 
		(*vpInDistCandidates_it)->mLocalMapPoints = 
				new DP(DP::load(saving_dir+"/"+to_string((*vpInDistCandidates_it)->mnId)+"/DataPoints.vtk"));
		TCandidateToCurrentICP = icp(*((*vpInDistCandidates_it)->mLocalMapPoints), TCu_Can);
		delete (*vpInDistCandidates_it)->mLocalMapPoints;
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

 
int LoopClosing::checkAllKeyFrameByDistance()
{
	//---construct a vector from map keyframes
	vector<KeyFrame*> allKeyFrames;

	mpMap->getAllKeyFrames(allKeyFrames);
	
	//---add candidate keyframes to candidateKFByDistance in consideration of distance
	
	candidateKFByDistance.clear();
	
	for(size_t i = 0; i < (allKeyFrames.size()); i++)
	{
		PM::TransformationParameters TCurrent = allKeyFrames[i]->getPose();
		cout << "node " << allKeyFrames[i]->mnId <<" " << allKeyFrames[i]->neighbours.size() <<" ";
// 		for(size_t j = (i+1); j < allKeyFrames.size(); j++)	//跟id没关系啦，要遍历
		for(size_t j = 0; j < allKeyFrames.size(); j++)
		{
			PM::TransformationParameters TCandidate = allKeyFrames[j]->getPose();
// 			Eigen::Vector3f delta_t = TCandidate.col(3).head(3) - TCurrent.col(3).head(3);
			//FIXME perhaps not right in every situation
			//changed to xy distance
// 			if((fabs(allKeyFrames[j]->mnId - allKeyFrames[i]->mnId ) > 1) && 
// 				(allKeyFrames[i]->neighbours.find(allKeyFrames[j]->mnId)!=(allKeyFrames[i]->neighbours.end())))
// 				{
// 					cout << "add " <<   allKeyFrames[i]->mnId <<" - "<< allKeyFrames[j]->mnId<<endl;
// 					pair<int, int> temp_pair(allKeyFrames[i]->mnId, allKeyFrames[j]->mnId);
// 					candidateKFByDistance.push_back(temp_pair);
// 				}
						
#if 1 // trick for yinhuan
			Eigen::Vector2f delta_t = TCandidate.col(3).head(2) - TCurrent.col(3).head(2);

			if((delta_t.norm()<distanceCheck))
			{
				pair<int, int> temp_pair(allKeyFrames[i]->mnId, allKeyFrames[j]->mnId);
				// 该点不是原本邻居
				if( allKeyFrames[i]->neighbours.find(allKeyFrames[j]->mnId) == (allKeyFrames[i]->neighbours.end()) &&
					(fabs(allKeyFrames[i]->mnId - allKeyFrames[j]->mnId) > 1)
				){
					cout << "add " <<   allKeyFrames[i]->mnId <<" - "<< allKeyFrames[j]->mnId<<endl;
					candidateKFByDistance.push_back(temp_pair);
				}
				//DE
// 				candidateKFByDistance.push_back(temp_pair);
				
				
			}
#endif
		}
	}
	

// 	pair<int, int> temp_pair(allKeyFrames.front()->mnId,allKeyFrames.back()->mnId);
// 	candidateKFByDistance.push_back(temp_pair);
	
// 	pair<int, int> temp_pair(306,386);
// 	candidateKFByDistance.push_back(temp_pair);
// 	pair<int, int> temp_pair2(306,384);
// 	candidateKFByDistance.push_back(temp_pair2);
// 	pair<int, int> temp_pair3(269,337);
// 	candidateKFByDistance.push_back(temp_pair3);
// 	
	return candidateKFByDistance.size();
}
	
int LoopClosing::computeICP()
{

// 	string KFbase = ros::package::getPath("laser_mapping")+"/KeyFrame/frames/";
// 	fstream recordMatches;
// 	string frecordMatches = ros::package::getPath("offline_optimization")+"/recordMatch.txt";
// 	recordMatches.open(frecordMatches);
	
// 	int thread_num = 8;
	if(thread_num >8)
		thread_num = 8;
	omp_set_num_threads(thread_num);
	cerr<<"thread_num = "<<thread_num<<endl;
// 	#pragma omp parallel for
	for(size_t i = 0;i<candidateKFByDistance.size(); i++)
	{
		
		KeyFrame* firstKF;
		KeyFrame* secondKF;
		
		firstKF = mpMap->getKeyFrame(candidateKFByDistance[i].first);
		secondKF = mpMap->getKeyFrame(candidateKFByDistance[i].second);
		
		cerr<<"computeICP + candidate by distance  "<<firstKF->mnId<<" "<<secondKF->mnId<<" ";
		
// 		if( !(firstKF->mLocalMapPoints) )
// 		{
// 			stringstream nodestring;
// 			nodestring<<KFbase<<candidateKFByDistance[i].first<<"/DataPoints.vtk";
// 			firstKF->mLocalMapPoints = new DP(DP::load(nodestring.str()));
// 		}
// 		if( !(secondKF->mLocalMapPoints) )
// 		{
// 			stringstream nodestring;
// 			nodestring<<KFbase<<candidateKFByDistance[i].second<<"/DataPoints.vtk";
// 			secondKF->mLocalMapPoints = new DP(DP::load(nodestring.str()));
// // 
// 		}

// 		PM::ICPSequence icp_copy;
// 		ifstream ifs(configFileName.c_str());
// 		icp_copy.loadFromYaml(ifs);

		
		icp.setMap(*(firstKF->mLocalMapPoints));
		PM::TransformationParameters Ticp = PM::TransformationParameters::Identity(4 ,4);
		PM::TransformationParameters Tpre;
		try{
			Tpre = (firstKF->getPose().inverse())*(secondKF->getPose());
			if(abs(Tpre(2,3)) >10)
				Tpre(2,3) /= 3;
			Ticp = icp(*(secondKF->mLocalMapPoints), Tpre);
			
			Eigen::Vector2f delta_t = Tpre.col(3).head(2) - Ticp.col(3).head(2);
			if((delta_t.norm()>20))//相当于可矫正的误差,小了曹楼那边木有边
			{ 
// 				recordMatches<<firstKF->mnId <<"  "<<secondKF->mnId<<" delta too large"<<endl;
				continue;
			}
			const double estimatedOverlap = icp.errorMinimizer->getOverlap();
			if(estimatedOverlap>icpOverlap)
			{
				std::pair<long unsigned int, PM::TransformationParameters> neighbor( firstKF->mnId,Ticp.inverse());
				std::pair<long unsigned int, PM::TransformationParameters> neighborinv( secondKF->mnId,Ticp);
				std::pair<long unsigned int, bool> neighbor_reachable(firstKF->mnId, false);
				std::pair<long unsigned int, bool> neighborinv_reachable(secondKF->mnId, true);

				std::map<long unsigned int, PM::TransformationParameters>::iterator n_it;
				std::map<long unsigned int, bool>::iterator r_it;
				
				n_it = secondKF->neighbours.find(firstKF->mnId);
				if(n_it != secondKF->neighbours.end()){
					secondKF->neighbours.erase(n_it);
					secondKF->neighbours_isReachable.erase(secondKF->neighbours_isReachable.find(firstKF->mnId));
				}
				r_it = firstKF->neighbours_isReachable.find(secondKF->mnId);
				if(r_it != firstKF->neighbours_isReachable.end()){
					firstKF->neighbours_isReachable.erase(r_it);
					firstKF->neighbours.erase(firstKF->neighbours.find(secondKF->mnId));
				}
				secondKF->neighbours.insert(neighbor);	
				secondKF->neighbours_isReachable.insert(neighbor_reachable);
				firstKF->neighbours.insert(neighborinv);
				firstKF->neighbours_isReachable.insert(neighborinv_reachable);
				
				cerr<<"added edge"<<endl;
				
// 				recordMatches<<firstKF->mnId <<"  "<<secondKF->mnId<<" OK"<<endl;
			}else
			{
				cerr<<"failed with overlap " <<estimatedOverlap <<endl;
// 				recordMatches<<firstKF->mnId <<"  "<<secondKF->mnId<<" overlap too small"<<endl;
			}
			

		}catch (runtime_error error)
		{
			cerr<<"KeyFrame "<<firstKF->mnId<<" and KeyFrame "<<secondKF->mnId<<endl;
			ROS_ERROR_STREAM("LOOP_CLOSING ICP failed to converge: " << error.what());
		}
	}
	timer to;
	Optimizer optimizer;
// 	optimizer.OptimizeGraph(mpMap, bestCandidate, mpCurrentKF,NumOfClosure++,icp);
// 	optimizer.OptimizeGraph(mpMap,mpLoopKF,NumOfClosure++,icp);
	//TEST
	optimizer.OptimizeGraph(mpMap,NumOfClosure++, icp);
	
// 	mpLocalMapper->UPdateTLocalICP(mpCurrentKF->mnId);
	isUpdate = true;

// 	cerr<<"[loop closer] Optimize takes "<<t.elapsed()<<endl;
// 	recordMatches.close();
}
}