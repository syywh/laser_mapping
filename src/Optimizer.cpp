#include "Optimizer.h"
#include <boost/graph/graph_concepts.hpp>
#include "ros/package.h"
#include "glog/logging.h"
#include "Eigen/Core"


namespace Velodyne_SLAM 
{

Optimizer::Optimizer()
{
	edges.clear();
	vertices.clear();
	edges.resize(0);
	vertices.resize(0);
}

void Optimizer::OptimizeGraph(Map* pMap, int Count, PointMatcher< float >::ICPSequence& icp)
{
	g2o::SparseOptimizer graphOptimizer;
 
	g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();///
	g2o::BlockSolverX* blockSolver = new g2o::BlockSolverX(linearSolver);
	
	g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
// 	g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
	
	graphOptimizer.setAlgorithm(optimizationAlgorithm);
	
	std::vector<float> std_var_vec = {10, 10, 10, 6,6,6};
	std::vector<float> std_var_vec_closer = {6, 6, 6, 4,4,4};
	
	std::vector<KeyFrame*> allKeyFrames;
	pMap->getAllKeyFrames(allKeyFrames);
	
	std::vector<KeyFrame*>::iterator allKeyFrames_it = allKeyFrames.begin();
	set<int> kfIds;
	for(; allKeyFrames_it != allKeyFrames.end(); allKeyFrames_it++)
	{
		addGraphVertex(graphOptimizer,(*allKeyFrames_it)->mnId, (*allKeyFrames_it)->getPose());
		kfIds.insert((*allKeyFrames_it)->mnId);
	}
	allKeyFrames_it = allKeyFrames.begin();
	for(; allKeyFrames_it != allKeyFrames.end(); allKeyFrames_it++)
	{
	
		 map<long unsigned int, PM::TransformationParameters>::iterator neighbor_it = (*allKeyFrames_it)->neighbours.begin();
		  map<long unsigned int, bool> tempReachable = (*allKeyFrames_it)->neighbours_isReachable;
		for(; neighbor_it !=(*allKeyFrames_it)->neighbours.end(); neighbor_it++)
		{
// 			cerr<<neighbor_it->first<<" is "<<tempReachable[neighbor_it->first]<<endl;
			if(kfIds.find(neighbor_it->first) != kfIds.end()){
				if(tempReachable[neighbor_it->first])
				{
	// 				cerr<<"2222"<<endl;
					addGraphEdge(graphOptimizer , (*allKeyFrames_it)->mnId,neighbor_it->first,
						neighbor_it->second,std_var_vec
					);
				}
			}
		}
	}
	
	stringstream s;
	string temp_s = ros::package::getPath("dynamic_SLAM") +"/Log/g2o/";  
	s<<temp_s<<"optimize_before"<<Count<<".g2o";
	
	g2o::VertexSE3Expmap * firstRobotPose = dynamic_cast<g2o::VertexSE3Expmap*>(graphOptimizer.vertex(0));
	firstRobotPose->setFixed(true);
	graphOptimizer.setVerbose(true);
	
	graphOptimizer.save(s.str().c_str());
	
	std::cerr <<"Optimizing"<<std::endl;
	graphOptimizer.initializeOptimization();
	graphOptimizer.optimize(20);
	std::cerr << "done." << std::endl;
	
	stringstream ss;
	ss<<temp_s<<"optimize_after"<<Count<<".g2o";
	graphOptimizer.save(ss.str().c_str());
	
	

	PM::TransformationParameters temp = PM::TransformationParameters::Identity(4, 4);
	for(size_t t =0; t<allKeyFrames.size(); t++)
	{
		KeyFrame* pKF = allKeyFrames[t];
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(graphOptimizer.vertex(pKF->mnId));
		SE3d iso3 = vSE3->estimate();
		Eigen::Vector3d eigt = iso3.translation();
// 		Eigen::Matrix m(3,3) ;
		temp = pKF->getPose();
		temp.col(3).head(3) = eigt.cast<float>();
		temp.block(0,0,3,3) =  iso3.rotationMatrix().cast<float>();
// 		temp.block<0,0>(3,3) = iso3.cast<float>();
		
		pKF->OptimizedPose(temp);
	}
	
	for(size_t t = 0; t < allKeyFrames.size() ; t++){
		KeyFrame* pKF = allKeyFrames[t];
		
		for (auto neighbour_it = pKF->neighbours.begin(); neighbour_it != pKF->neighbours.end(); ++neighbour_it){
			int neighbor_id =  neighbour_it->first;
			KeyFrame* neighbor_KF = pMap->getKeyFrame(neighbor_id);
			PM::TransformationParameters pose_matrix = pKF->getPose().inverse() * neighbor_KF->getPose() ;
			neighbour_it->second = pose_matrix;
			
			
		}
	}
	
	graphOptimizer.clear();
}


void Optimizer::addGraphEdge(g2o::SparseOptimizer& optimizer,   long unsigned int  pre_vertex_idx,  
			     long unsigned int  cur_vertex_idx, PM::TransformationParameters relative_pose_matrix,  vector< float > std_var_vec)
{
	Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
	transNoise(0, 0) = std::pow(std_var_vec[0], 2);
	transNoise(1, 1) = std::pow(std_var_vec[1], 2);
	transNoise(2, 2) = std::pow(std_var_vec[2], 2);
	// std::cout << "translation noise: " << std::pow(transNoise(0, 0),0.5) << std::endl;

	Eigen::Matrix3d rotNoise = Eigen::Matrix3d::Zero();
	rotNoise(0, 0) = std::pow(std_var_vec[3], 2);
	rotNoise(1, 1) = std::pow(std_var_vec[4], 2);
	rotNoise(2, 2) = std::pow(std_var_vec[5], 2);
	// std::cout << "rotation noise: " << std::pow(rotNoise(0, 0),0.5) << std::endl;

	//????
	Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
	information.block<3,3>(0,0) = transNoise.inverse();
	information.block<3,3>(3,3) = rotNoise.inverse();

	g2o::VertexSE3Expmap* prev = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(pre_vertex_idx)->second);
	g2o::VertexSE3Expmap* cur = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertices().find(cur_vertex_idx)->second);
	//  g2o::VertexSE3* prev = vertices[pre_vertex_idx];
	//  g2o::VertexSE3* cur  = vertices[cur_vertex_idx];
	// Eigen::Isometry3d iso3_edge = prev->estimate().inverse() * cur->estimate();

	Eigen::Matrix4d iso3_edge;
// 	relative_pose_matrix = relative_pose_matrix.inverse();
	iso3_edge.block<3,3>(0,0) = relative_pose_matrix.block<3,3>(0,0).cast<double>();
	iso3_edge.block<3,1>(0,3) = relative_pose_matrix.block<3,1>(0,3).cast<double>();
	
	g2o::EdgeSE3* e = new g2o::EdgeSE3;
	e->setVertex(0, prev);
	e->setVertex(1, cur);
	e->setMeasurement(SE3d(iso3_edge));
	e->setInformation(information);
	
	g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
	e->setRobustKernel(rk);
	rk->setDelta(thHuber);
	
	// g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(e->vertex(0));
	// g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(e->vertex(1));
	// g2o::HyperGraph::VertexSet aux; aux.insert(from);
	// e->initialEstimate(aux, to);
// 	odometryEdges.push_back(e);
	edges.push_back(e);

// 	std::cout << "added [edge " << prev->id() << "-" << cur->id() << "] ";
// 	e->write(std::cout);
// 	std::cout << endl;

	optimizer.addEdge(e);
}

void Optimizer::addGraphVertex(g2o::SparseOptimizer& optimizer,  long unsigned int vertex_id, PM::TransformationParameters pose_matrix, bool fixed)
{
	g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
	v->setId(vertex_id);
	v->setFixed(false);
	// Eigen::AngleAxisd rotz(0.5, Eigen::Vector3d::UnitZ());
	// Eigen::AngleAxisd roty(-0.5, Eigen::Vector3d::UnitY());
	// Eigen::Matrix3d rot = (rotz * roty).toRotationMatrix();
	Eigen::Matrix4d iso3;
	iso3.block<3,3>(0,0) = pose_matrix.block<3,3>(0,0).cast<double>();
	iso3.block<3,1>(0,3) = pose_matrix.block<3,1>(0,3).cast<double>();
	// iso3 = rot;
	// iso3.translation() = iso3.linear() * Eigen::Vector3d(3, 0, 0);
	v->setEstimate(SE3d(iso3));
	vertices.push_back(v);
// 	std::cout << "added: [vertex " << vertex_id << "] Fixed? " << v->fixed() << std::endl;;
	v->write(std::cout);
// 	std::cout << std::endl;

	optimizer.addVertex(v);
}

Vector3d Optimizer::OptimizeInitialGyroBias(const vector< PM::TransformationParameters >& vTwc, 
					    const vector< vill::IMUPreintegrator >& vImuPreInt)
{
       int N = vTwc.size();
        if (vTwc.size() != vImuPreInt.size()) cerr << "vTwc.size()!=vImuPreInt.size()" << endl;
        Matrix4d Tbc = vill::ConfigParam::GetEigTbl();
        Matrix3d Rcb = Tbc.topLeftCorner(3, 3).transpose();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver = 
			      new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
// 	LOG(INFO) << (linearSolver == NULL) << endl;
	optimizer.setVerbose(true);

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
// 	LOG(INFO) << (solver_ptr == NULL) << endl;

        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
//         g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
// 	LOG(INFO) << (solver == NULL) << endl;
        optimizer.setAlgorithm(solver);

        // Add vertex of gyro bias, to optimizer graph
//         cout << optimizer.vertices()[0] << endl;
	optimizer.clear();
	g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias;
        vBiasg->setEstimate(Vector3d());
        vBiasg->setId(0);
        vBiasg->setFixed(false);
	optimizer.addVertex(vBiasg);
// 	cout << vBiasg->id() << endl;
//         cout <<  << endl;
// 	
// 	cout << optimizer.vertices().size() << endl;
// 	cout << vBiasg << endl;
// 	cout << (optimizer.vertex(0) == NULL) << endl;

        // Add unary edges for gyro bias vertex
        //for(std::vector<KeyFrame*>::const_iterator lit=vpKFs.begin(), lend=vpKFs.end(); lit!=lend; lit++)
        for (int i = 0; i < N; i++) {
            // Ignore the first KF
            if (i == 0)
                continue;

            const SE3d &Twi = SE3d(vTwc[i - 1].cast<double>());    // pose of previous KF
            Matrix3d Rwci = Twi.rotationMatrix();
            const SE3d &Twj = SE3d(vTwc[i].cast<double>());        // pose of this KF
            Matrix3d Rwcj = Twj.rotationMatrix();

            const IMUPreintegrator &imupreint = vImuPreInt[i];

            g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
            eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            // measurement is not used in EdgeGyrBias
            eBiasg->dRbij = imupreint.getDeltaR();
            eBiasg->J_dR_bg = imupreint.getJRBiasg();
            eBiasg->Rwbi = Rwci * Rcb;
            eBiasg->Rwbj = Rwcj * Rcb;
            eBiasg->setInformation(Eigen::Matrix3d::Identity());
// 	    LOG(INFO)<<endl<<eBiasg->dRbij;
// 	    LOG(INFO)<<endl<<eBiasg->Rwbi<<endl <<eBiasg->Rwbj<<endl<<endl; 
            optimizer.addEdge(eBiasg);
        }

        // It's actualy a linear estimator, so 1 iteration is enough.
        //optimizer.setVerbose(true);
//         optimizer.save("/home/turtle/catkin_ws/src/dynamic_SLAM/log/a.g2o");
        optimizer.initializeOptimization();
        optimizer.optimize(1);
// 	optimizer.save("/home/turtle/catkin_ws/src/dynamic_SLAM/log/b.g2o");

        g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));

        return vBgEst->estimate();
}

bool Optimizer::OptimizePosewithIMU(std::vector< Frame* > pFs, Vector3d& gw, Vector3d& dbiasa, Vector3d& dbiasg, Vector3d& velo)
{
	  // Setup optimizer
// 	cout <<"[OptimizePosewithIMU]"<< endl;
	PointMatcherSupport::timer t;
	
	Vector3d GravityVec = gw;
	SE3d Tbl = vill::ConfigParam::GetSE3Tbl();
	SO3d Rbl = Tbl.so3();
	Vector3d Pbl = Tbl.translation();
    
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolverX::LinearSolverType *linearSolver;

	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

	g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);

	const float thHuberNavStatePVR = sqrt(21.666);
	const float thHuberNavStateBias = sqrt(16.812);
	const float thHuberOne = sqrt(3.84);

	
	vector<g2o::EdgeNavStatePVR *> vpEdgesNavStatePVR;
	vector<g2o::EdgeNavStateBias *> vpEdgesNavStateBias;
	vector<g2o::EdgeNavStateICP*> vpEdgesNavStateOnlyPose;
	
	for(std::vector< Frame* >::iterator vit = pFs.begin(), vend = pFs.end(); vit != vend;vit++ ){
	    Frame* pF = *vit;
// 	    LOG(INFO)<<pF->mnId;
	    int idF = pF->mnId * 2;
	    
	    if(pF->mnId == (*(pFs.begin()))->mnId)
// 	    if(pF->mnId == 0)
	    {
		// Vertex of PVR
			{
				g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
				vNSPVR->setEstimate(pF->GetNavState());
				
	// 		    cerr<<pF->GetNavState().Get_P()<<endl
	// 		    <<pF->GetNavState().Get_R().matrix()<<endl
	// 		    <<pF->GetNavState().Get_V()
	// 		    <<pF->GetNavState().Get_BiasAcc()
	// 		    <<pF->GetNavState().Get_BiasGyr()<<endl;
				
				vNSPVR->setId(idF);
				vNSPVR->setFixed(true);
				optimizer.addVertex(vNSPVR);
			}
			// Vertex of Bias
			{
				g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
				vNSBias->setEstimate(pF->GetNavState());
				vNSBias->setId(idF + 1);
				vNSBias->setFixed(true);
				optimizer.addVertex(vNSBias);
			}
			continue;
	    }
	
		// Vertex of PVR
		{
			g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
			vNSPVR->setEstimate(pF->GetNavState());
			vNSPVR->setId(idF);
			vNSPVR->setFixed(false);
// 		    cerr<<pF->mnId<<endl<<"pose: "<<pF->GetNavState().Get_P().transpose()<<endl
// 		    <<pF->GetNavState().Get_R().matrix()<<endl
// 		    <<"velo: "<<pF->GetNavState().Get_V().transpose()<<endl
// 		    <<"ba: "<<pF->GetNavState().Get_BiasAcc().transpose()<<endl
// 		    <<"bg: "<<pF->GetNavState().Get_BiasGyr().transpose()<<endl;
			optimizer.addVertex(vNSPVR);
		}
		// Vertex of Bias
		{
			g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
			vNSBias->setEstimate(pF->GetNavState());
			vNSBias->setId(idF + 1);
			vNSBias->setFixed(false);
			optimizer.addVertex(vNSBias);
		}
	}
	
	for(std::vector< Frame* >::iterator vit = pFs.begin(), vend = pFs.end(); vit != vend;vit++ ){
		Frame* pF = *vit;
		g2o::EdgeVelocityConstraint* e = new g2o::EdgeVelocityConstraint();
		e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF->mnId)));
		e->setMeasurement(0);
		Eigen::Matrix<double,1,1> info = Eigen::Matrix<double,1,1>::Identity() * 1000;
		e->setInformation(info);
		
		g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		e->setRobustKernel(rk);
		rk->setDelta(thHuberOne);
		
		optimizer.addEdge(e);
	}
	
	
	Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
	InvCovBgaRW.topLeftCorner(3, 3) =
			Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
	InvCovBgaRW.bottomRightCorner(3, 3) =
			Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
		
	float HuberKernel = 12.59;
	float thHuber_ = sqrt(HuberKernel);
	
	// add edges
	//1. add priori
	Frame* pF_first = pFs[0];
// 	Frame* frame_before_first = pF_first->previousF;
	if(pF_first->getMarginal()){
		g2o::EdgePrioriNavStatePVRBias* epriori = new g2o::EdgePrioriNavStatePVRBias();
		epriori->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF_first->mnId)));
		epriori->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF_first->mnId + 1)));
		
		epriori->setMeasurement(pF_first->GetNavState());
		epriori->setInformation(pF_first->getMarginalizedInfo());
		
		g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
		epriori->setRobustKernel(rk);
		rk->setDelta(thHuberNavStatePVR);
		
		optimizer.addEdge(epriori);
	}
	
	
	for (int ei = 1; ei < pFs.size(); ei++) {
	    
	  
		Frame *pF1 = pFs[ei];                      // Current KF, store the IMU pre-integration between previous-current
		Frame *pF0 = pFs[ei-1];   // Previous KF

// 		cout<<"previous "<<pF0->mnId<<" current "<<pF1->mnId<<endl;
		// PVR edge
		{
			g2o::EdgeNavStatePVR *epvr = new g2o::EdgeNavStatePVR();
			epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF0->mnId)));
			epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF1->mnId)));
			epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF0->mnId + 1)));
			epvr->setMeasurement(pF1->GetIMUPreint());

// 			cout<<"pre:\n"<<pF1->GetIMUPreint().getDeltaR()<<endl<<pF1->GetIMUPreint().getDeltaP().transpose()<<endl;
			NavState n0 = pF0->GetNavState();
			NavState n1 = pF1->GetNavState();
// 			cout<<"Nav\n"<<(n0.Get_R().inverse() * n1.Get_R()).matrix()<<endl
// 			<<(n0.Get_R().inverse()*(n1.Get_P() - n0.Get_P())).transpose()<<endl;

			Matrix9d InvCovPVR = pF1->GetIMUPreint().getCovPVPhi().inverse();

			epvr->setInformation(InvCovPVR);
			epvr->SetParams(GravityVec);

			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			epvr->setRobustKernel(rk);
			rk->setDelta(thHuberNavStatePVR);

			optimizer.addEdge(epvr);
			vpEdgesNavStatePVR.push_back(epvr);
		
		}
             
		//icp edge
		{
			g2o::EdgeNavStateICP* epp = new g2o::EdgeNavStateICP();
			epp->setTbl(vill::ConfigParam::GetSE3Tbl());
			epp->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pF0->mnId)));
			epp->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(2*pF1->mnId)));
			PM::TransformationParameters deltaT = pF0->GetGlobalPose().inverse() * pF1->GetGlobalPose();
			epp->setMeasurement(SE3d(deltaT.cast<double>()));
// 			cerr<<"icpT\n"<<vill::ConfigParam::GetEigTbl().cast<float>() * deltaT * vill::ConfigParam::GetEigTbl().inverse().cast<float>()<<endl;
			Matrix6d info = (Matrix6d::Identity()*1000);
			
// 			info(3,3) = 1000;
// 			info(4,4) = 1000;
// 			info(5,5) = 1000;
			
			epp->setInformation(info);
			
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			epp->setRobustKernel(rk);
			rk->setDelta(thHuber_);
			
			optimizer.addEdge(epp);
			vpEdgesNavStateOnlyPose.push_back(epp);
	    }
		// Bias edge
		{
			g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
			ebias->setVertex(0,
								dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF0->mnId + 1)));
			ebias->setVertex(1,
								dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pF1->mnId + 1)));
			ebias->setMeasurement(pF1->GetIMUPreint());

			ebias->setInformation(InvCovBgaRW / pF1->GetIMUPreint().getDeltaTime());

			g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
			ebias->setRobustKernel(rk);
			rk->setDelta(thHuberNavStateBias);

			optimizer.addEdge(ebias);
			vpEdgesNavStateBias.push_back(ebias);
		}

	}
        
        
	optimizer.initializeOptimization();
	optimizer.optimize(10);
        
	//recovery
	int update = 0;
	for(int ei = 0; ei < pFs.size(); ei++){
		Frame *pFi = pFs[ei];
		g2o::VertexNavStatePVR *vNSPVR = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(2 * pFi->mnId));
		g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(
				2 * pFi->mnId + 1));
		// In optimized navstate, bias not changed, delta_bias not zero, should be added to bias
		const NavState &optPVRns = vNSPVR->estimate();
		const NavState &optBiasns = vNSBias->estimate();
		
		Vector3d dt = pFi->GetNavState().Get_P() - optPVRns.Get_P();
		Matrix3d dr = pFi->GetNavState().Get_RotMatrix().transpose() * optPVRns.Get_RotMatrix();
	
		// Update NavState

// 	    cout<<"v: "<<optPVRns.Get_V().transpose()<<endl;
		if(optBiasns.Get_dBias_Acc().norm()<1/*e-3*/ && optBiasns.Get_dBias_Gyr().norm()<1/*e-4*/)
		{ 
			pFi->SetNavStatePos(optPVRns.Get_P());
			pFi->SetNavStateVel(optPVRns.Get_V());
			pFi->SetNavStateRot(optPVRns.Get_R());
			pFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
			pFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
			cout<<pFi->mnId <<" "<<optBiasns.Get_dBias_Gyr().transpose()<<" "<<optBiasns.Get_dBias_Acc().transpose()
			<<" " << optPVRns.Get_V().transpose()<<endl;
			update = ei;
		}else{
			cout<<"failed to update dbias "<<pFi->mnId <<" "<<optBiasns.Get_dBias_Gyr().transpose()<<" "<<optBiasns.Get_dBias_Acc().transpose()<<endl;
		}
	    // Update pose Tcw
//             pFi->UpdatePoseFromNS(Tbl);
	   
	}
	Frame* lastF = pFs.back();
	dbiasa = pFs[update]->GetNavState().Get_dBias_Acc();
	dbiasg = pFs[update]->GetNavState().Get_dBias_Gyr();
	int num = pFs.size();
	velo = pFs[update]->GetNavState().Get_V();
// 	lastF->UpdateNavStatePVRFromTwc(SE3d(lastF->GetGlobalPose().cast<double>()), vill::ConfigParam::GetSE3Tbl());
	
// 	{
// 	      Vector3d gw_refine;
// 	      VectorXd x;
// 	      list<KeyFrame *> lInitKFs = lLocalKeyFrames;
// 	      LIV::preintegration_opt::LinearAlignment(pFs,gw_refine,x);
// 	      LIV::preintegration_opt::setGravity(gw_refine);
// 	      LOG(INFO)<<"gw\n"<<gw_refine.transpose();
// 	      LinearAlignment(lInitKFs, gw_refine, x);
// 	      pLM->setGravityVec(gw_refine);
// 	}
	
// 	std::vector<g2o::OptimizableGraph::Vertex *> margVerteces;
// 	margVerteces.push_back(optimizer.vertex(2 * (pF_first->mnId)));
// 	margVerteces.push_back(optimizer.vertex(2 * (pF_first->mnId) + 1 ));

	//TODO: how to get the joint marginalized covariance of PVR&Bias
// 	g2o::SparseBlockMatrixXd spinv;
// 	optimizer.computeMarginals(spinv, margVerteces);
// // 	// spinv include 2 blocks, 9x9-(0,0) for PVR, 6x6-(1,1) for Bias
// 	Matrix<double, 15, 15> margCovInv = Matrix<double, 15, 15>::Zero();
// 	margCovInv.topLeftCorner(9, 9) = spinv.block(0, 0)->inverse();
// 	margCovInv.bottomRightCorner(6, 6) = spinv.block(1, 1)->inverse();
// 	pFrame->mMargCovInv = margCovInv;
// 	pFrame->mNavStatePrior = ns_recov;
	
// 	pFs[1]->setMarginal(true);
        
	LOG(INFO)<<"Opt with IMU : "<<t.elapsed();
}

void Optimizer::OptimizeInitialGravityBa(vector< IMUData >& vimudatas, Vector2d& pitch_roll, Vector3d& biasa)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

//         g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    
    
    //add vertex for pitch and roll;
    g2o::VertexSO3PitchRoll *vpitchroll = new g2o::VertexSO3PitchRoll();
    vpitchroll->setId(0);
    vpitchroll->setEstimate(Vector2d(0,0));
    optimizer.addVertex(vpitchroll);
    
    //add vertex for initializing acc bias;
    g2o::VertexBa *vBa = new g2o::VertexBa();
    vBa->setId(1);
    vBa->setEstimate(Vector3d(0,0,0));
    optimizer.addVertex(vBa);
    
    g2o::EdgeBiasa *eBa = new g2o::EdgeBiasa();
    eBa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
    Matrix3d infoba = 300 * Eigen::Matrix3d::Identity();
    eBa->setInformation(infoba);
    optimizer.addEdge(eBa);
    
    double huber_th3 = sqrt(7.18);
    
    
    for(int i = 0; i < vimudatas.size(); i++){
      vill::IMUData imudata = vimudatas[i];
      Vector3d measure_acc = imudata._a;
      
      g2o::EdgeCoarsePitchRollBa * epitchrollba = new g2o::EdgeCoarsePitchRollBa();
      epitchrollba->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
      epitchrollba->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
      epitchrollba->setMeasurement(measure_acc);
      
      Matrix3d info = 100 * Eigen::Matrix3d::Identity();
//       info(0,0) = 10;
//       info(1,1) = 10;
      epitchrollba->setInformation(info);
      
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      epitchrollba->setRobustKernel(rk);
      rk->setDelta(huber_th3);
      
      optimizer.addEdge(epitchrollba);
    }
    
    cout<<"begin optimization "<<endl;
      optimizer.initializeOptimization();
      optimizer.optimize(10);
      g2o::VertexSO3PitchRoll* estimatedpr = static_cast<g2o::VertexSO3PitchRoll*>(optimizer.vertex(0));
      LOG(INFO)<<"optimized pitch and roll\n"<<estimatedpr->estimate()<<endl<<endl;
      g2o::VertexBa* estimatedBa = static_cast<g2o::VertexBa*>(optimizer.vertex(1));
      LOG(INFO)<<"optimized bias acc\n"<<estimatedBa->estimate()<<endl;
      cout<<"======="<<endl;
      
//       optimizer.optimize(1);
//       estimatedpr = static_cast<g2o::VertexSO3PitchRoll*>(optimizer.vertex(0));
//       LOG(INFO)<<"optimized pitch and roll\n"<<estimatedpr->estimate()<<endl<<endl;
//       estimatedBa = static_cast<g2o::VertexBa*>(optimizer.vertex(1));
//       LOG(INFO)<<"optimized bias acc\n"<<estimatedBa->estimate()<<endl;
//       
//       cout<<"======="<<endl;
//       optimizer.optimize(1);
//       estimatedpr = static_cast<g2o::VertexSO3PitchRoll*>(optimizer.vertex(0));
//       LOG(INFO)<<"optimized pitch and roll\n"<<estimatedpr->estimate()<<endl<<endl;
//       estimatedBa = static_cast<g2o::VertexBa*>(optimizer.vertex(1));
//       LOG(INFO)<<"optimized bias acc\n"<<estimatedBa->estimate()<<endl;

      
      pitch_roll = estimatedpr->estimate();
      biasa = estimatedBa->estimate();
}


}
