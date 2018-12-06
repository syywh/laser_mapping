#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "KeyFrame.h"
// #include "LoopClosing.h"
#include "Frame.h"

#include<Eigen/Dense>

#include "IMU/IMUPreintegrator.h"
#include "Thirdparty/sophus/sophus/se3.hpp"
#include "IMU/configparam.h"
#include "pointmatcher/PointMatcher.h"

#include "IMU/g2otypes.h"
#include "preintegration_opt.h"

namespace Velodyne_SLAM
{

// class LoopClosing;

class Optimizer
{
public:
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	Optimizer();
// 	int  PoseOptimization(Frame* pFrame);

	void  OptimizeGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, int Count,PM::ICPSequence& icp);
	void  OptimizeGraph(Map* pMap,map<pair<long unsigned int ,long unsigned int>,PM::TransformationParameters >& pLoopKFMap, int Count,PM::ICPSequence& icp);
	void OptimizeGraph(Map* pMap, int Count, PM::ICPSequence& icp);
	Eigen::Vector3d static OptimizeInitialGyroBias(const vector<PM::TransformationParameters> &vTwc, const vector<vill::IMUPreintegrator> &vImuPreInt);
	
	bool static OptimizePosewithIMU(vector<Frame*> pFs, Vector3d& gw, Vector3d& dbiasa, Vector3d& dbiasg, Vector3d& velo);
	
	static void OptimizeInitialGravityBa(vector<vill::IMUData>& vimudatas, Vector2d& pitch_roll, Vector3d& biasa );
	
	void addGraphVertex(g2o::SparseOptimizer& optimizer, 
			    long unsigned int vertex_id, 
		     PM::TransformationParameters pose_matrix, 
		     bool fixed=false);

	void addGraphEdge( g2o::SparseOptimizer& optimizer,
                  long unsigned int  pre_vertex_idx,
                  long unsigned int  cur_vertex_idx, 
                PM::TransformationParameters relative_pose_matrix, 
                std::vector<float> std_var_vec );
	
	std::vector<g2o::VertexSE3Expmap*> vertices;
	std::vector<g2o::EdgeSE3*> odometryEdges;
	std::vector<g2o::EdgeSE3*> edges;
	
	
	
       const float HuberKer = 12.59;
      	const float thHuber = sqrt(HuberKer);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
