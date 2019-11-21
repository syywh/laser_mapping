#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

// #include "icp_mapper.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "Frame.h"
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "icp_mapper.h"
#include "fstream"
#include<ros/package.h>
#include "Optimizer.h"
#include "IMU/g2otypes.h"
// #include <g2o/types/slam3d/edge_se3.h>
#include <omp.h>


namespace Velodyne_SLAM
{
class LocalMapping;
class ICPMapper;
class Optimizer;

class LoopClosing{
public:
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	LoopClosing();
	LoopClosing(Map* pMap);
	
	void SetIcpMapper(ICPMapper* pICPMapper);	//used for relocalization
	
	void SetLocalMapper(LocalMapping* pLocalMapper);
	long unsigned int  GetFirstKFId();
	
	void Run();
	void InsertKeyFrame(KeyFrame* pKF);
	
	void run_offline();
	ofstream fLoopClosing;
	ofstream fLoopClosing_Loop_closing;
	
	int checkAllKeyFrameByDistance();
	int computeICP();
	
	void setSavingDir(std::string& dir_name){
		saving_dir = dir_name;
	}
	
	bool isUpdate;
// 	void RequestReset();
	
protected:
	bool CheckNewKeyFrames();
	bool DetectLoop();
	
	bool mbResetRequested;
	
	boost::mutex mMutexReset;
	
	Map* mpMap;
	ICPMapper* mpTracker;
	LocalMapping *mpLocalMapper;
	
	std::list<KeyFrame*> mlpLoopKeyFrameQueue;	//wait for loop closing check
	boost::mutex mMutexLoopQueue;	//mutex for list queue
	
	//Loop detector variables
	KeyFrame* mpCurrentKF;
	KeyFrame* mpMatchedKF;
	
	map<pair<long unsigned int ,long unsigned int>,PM::TransformationParameters > mpLoopKF;
	
	std::vector<g2o::EdgeSE3*> edges;
	
	long unsigned int mLastLoopKFid;
	int mLastLoopKFTime;
	long unsigned int mLastProcessedKFId;
	
	PM::ICPSequence icp;
	
	string filename ;
	string filename_loop_closing ;
// 	Optimizer* opitmizer;
	
	int NumOfClosure;
	int beginKFId;
	
	int thread_num;
	double distanceCheck;
	double icpOverlap;
	
	vector<pair<int, int> > candidateKFByDistance;
	
	std::string saving_dir;
	
};
}


#endif