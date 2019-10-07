#include "preintegration_opt.h"
#include "pointmatcher/Timer.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

namespace LIV{

// Vector3d preintegration_opt::mGravityVec = Vector3d();
// boost::mutex preintegration_opt::mMutexGravity;


  
preintegration_opt::preintegration_opt():
mbIMUIniting(true),mbIMUInited(false)
{
//   mptIMUInitial = new std::thread(&preintegration_opt::imuIniting, this);
//   preintegration_opt::mGravityVec(2) = -9.8;

  if(! ros::param::get("~windows_width", window_width)){
    ROS_ERROR_STREAM("cannot load windows_width");
    window_width = 50;
  }else{
    ROS_INFO_STREAM("Load windows_width "<<window_width);
  }
  
  if(! ros::param::get("~delta_width", delta)){
    ROS_ERROR_STREAM("cannot load delta_width");
    delta = 10;
  }else{
    ROS_INFO_STREAM("Load delta_width "<<delta);
  }
  
  bBiasUpdated = false;
  
  vel_pub = n.advertise<geometry_msgs::Vector3>("opt_vel",1,true);
  dbias_pub = n.advertise<geometry_msgs::Twist>("opt_dbias",1,true);
}

void preintegration_opt::Run()
{
  ros::Rate r(10);
  //记得留下最后一个
  
  while(ros::ok()){
	if(checkNewFrames()){
		PointMatcherSupport::timer t;
		vector<Velodyne_SLAM::Frame*> copyFrames;
		{
			boost::mutex::scoped_lock lock(mMutexProcessFrames);
// 		copyFrames = vFrames;
			copyFrames.assign(vFrames.begin(), vFrames.end());
	
		for(int i = 0; i < copyFrames.size(); i++)
			cerr<<copyFrames[i]->mnId<<" ";
		cerr<<endl;
		LOG(INFO)<<"copy size "<<copyFrames.size();
		Velodyne_SLAM::Frame* endFrame = vFrames.back();
		vFrames.clear();
		vFrames.push_back(endFrame);
		}
		
		if(processFrames(copyFrames)){
		setBiasUpdated(true);
		}
		cerr<<"optimization took "<<t.elapsed()<<endl;
	  
	}
	r.sleep();
  }

}

//别人用的
void preintegration_opt::insertFrameswithIMU(Velodyne_SLAM::Frame* pF)
{
  boost::mutex::scoped_lock lock(mMutexProcessFrames);
  vFrames.push_back(pF);
  pF->previousF = mCurrentFrame;
  mCurrentFrame = pF;
}

void preintegration_opt::insertFrame(Velodyne_SLAM::Frame* pF)
{
	if(pF->mnId == 0) return;
	lFrames.push_back(pF);
	mCurrentFrame = pF;
	if(lFrames.size() > window_width)
		lFrames.pop_front();

}


bool preintegration_opt::checkNewFrames()
{
  boost::mutex::scoped_lock lock(mMutexProcessFrames);
  return (vFrames.size() > window_width);
}

// bool preintegration_opt::isInitialized()
// {
//   return initialized;
// 
// }

void preintegration_opt::SetIMUIniting(bool flag)
{
    boost::mutex::scoped_lock lock(mMutexIMUIniting);
    mbIMUIniting = flag;
}

bool preintegration_opt::GetIMUIniting()
{
    boost::mutex::scoped_lock lock(mMutexIMUIniting);
    return mbIMUIniting;
}

bool preintegration_opt::GetIMUInited()
{
  boost::mutex::scoped_lock lock(mMutexIMUInited);
  return mbIMUInited;
}

void preintegration_opt::SetIMUInited(bool flag)
{
  boost::mutex::scoped_lock lock(mMutexIMUInited);
  mbIMUInited = flag;
}

std::vector< Velodyne_SLAM::Frame* > preintegration_opt::GetFramesForInit()
{
  boost::mutex::scoped_lock lock(mMutexFramesForInit);
  return vFramesForInit;

}

void preintegration_opt::insertFramesForInitialization(Velodyne_SLAM::Frame* pF)
{
      boost::mutex::scoped_lock lock(mMutexFramesForInit);
      if(pF->mnId < 2) return;
      if(vFramesForInit.size())
	pF->previousF = mCurrentFrame;
      vFramesForInit.push_back(pF);
      mCurrentFrame = pF;

}


bool preintegration_opt::TryInitIMU()
{
    PointMatcherSupport::timer t;
    bool bIMUInited = false;
    
    {
        boost::mutex::scoped_lock lock(mMutexFramesForInit);
	if(vFramesForInit.size() < 51)
	  return false;
    }
    
    SE3d Tbl = vill::ConfigParam::GetSE3Tbl();
    SE3d Tlb = Tbl.inverse();
    Matrix3d Rlb = Tlb.rotationMatrix();
    Vector3d plb = Tlb.translation();
    
    vector<Velodyne_SLAM::Frame* > vFramesforinit = GetFramesForInit();
    int N = vFramesforinit.size();
    Velodyne_SLAM::Frame*  pNewestFrame = vFramesforinit[N-1];
    vector<PM::TransformationParameters> vTwl;
    vector<vill::IMUPreintegrator> vIMUPreInt;
    vector<Velodyne_SLAM::Frame*> vFInit;
    
    cerr<<"imu size in each frames"<<endl;
    int deltacnt = 10;
    vector<vill::IMUData> tempimudata;
//     vector<vill::IMUData> savefornearestFrame = vFramesforinit.back().;
    for(int i = 0 + deltacnt ; i < N; i = i+deltacnt){
      Velodyne_SLAM::Frame* pF = vFramesforinit[i];
      for(int j = deltacnt-1; j >= 0; j--)
	tempimudata.insert(tempimudata.end(), vFramesforinit[i-j]->mvIMUdatas.begin(),vFramesforinit[i-j]->mvIMUdatas.begin());
      pF->mvIMUdatas.insert(pF->mvIMUdatas.begin(), tempimudata.begin(), tempimudata.end());
      pF->ComputePreInt();
      cerr<<pF->mnId<<"|"<< pF->mvIMUdatas.size()<<" ";
      vTwl.push_back(pF->GetGlobalPose());
      vIMUPreInt.push_back(pF->GetIMUPreint());
    }
    cerr<<endl;
    // Step 1. // Try to compute initial gyro bias, using optimization with Gauss-Newton
    Vector3d bgest = Velodyne_SLAM::Optimizer::OptimizeInitialGyroBias(vTwl, vIMUPreInt);
    LOG(INFO)<<"bgest"<<endl<<bgest;

    
    // recompute
    for(int i = 0; i < N; i++){
	vFramesforinit[i]->ComputePreInt(bgest);
    }
    
    bool bimuInitialized = false;
    Vector3d gw;
    VectorXd x;
    if(LinearAlignment(vFramesforinit,gw, x )){
      bIMUInited = true;
      {
	boost::mutex::scoped_lock lock(mMutexBiasUpdated);
	dbiasg = bgest;
	dbiasa = Vector3d();
	velo = Vector3d();
      }
      setBiasUpdated(true);
    }

    LOG(INFO)<<bIMUInited<<endl<<gw;
    mGravityVec = gw;
//     mGravityVec 
    
    return bIMUInited;

}


//initialize the bias of gyro and gravity
void preintegration_opt::imuIniting()
{
      unsigned long initedid = 0;
      LOG(INFO) << "IMU: Start VINSInit Thread";
//       SetIMUIniting(true);
      
      while(true){
	    if(Velodyne_SLAM::Frame::nNextId > 2){
	      bool initialized = GetIMUInited();
	      if( !initialized ){
		{
		  boost::mutex::scoped_lock lock(mMutexFramesForInit);
		  if(mCurrentFrame == nullptr) continue;
		  if( mCurrentFrame->mnId > initedid)
		      initedid = mCurrentFrame->mnId;
		}
		bool tmpInitialized = TryInitIMU();
		if(tmpInitialized){
		  LOG(INFO)<<"IMU: inited, quit VINSinitThread" << endl;
		  break;
		}
	      }
	    }
	    std::this_thread::sleep_for(std::chrono::milliseconds(int(3)));
	    
      }
//     SetIMUIniting(false);
    SetIMUInited(true);
    LOG(INFO) << "IMU: finished, quit VINSInitThread" << endl;
      
}

MatrixXd preintegration_opt::TangentBasis(Vector3d &g0){
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
	tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}
	

void preintegration_opt::RefineGravity(vector<Velodyne_SLAM::Frame*> &all_image_frame, Vector3d &g, VectorXd &x)
{
    Vector3d gI(0, 0, 1.0);
    Vector3d GI = gI * ConfigParam::GetG();
    
    // Extrinsics parameters
    SE3d Tbc = ConfigParam::GetSE3Tbl();
    SE3d Tcb = Tbc.inverse();
    Matrix3d Rbc = Tbc.rotationMatrix();
    Vector3d pbc = Tbc.translation();
    Matrix3d Rcb = Tcb.rotationMatrix();
    Vector3d pcb = Tcb.translation();
  
    Vector3d g0 = g.normalized() * GI.norm();
    Vector3d lx, ly;
    
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    vector<Velodyne_SLAM::Frame*>::iterator frame_i;
    vector<Velodyne_SLAM::Frame*>::iterator frame_j;
    for(int k = 0; k < 4; k++)
    {
	MatrixXd lxly(3, 2);
	lxly = TangentBasis(g0);
	int i = 0;
	for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
	{
	    frame_j = next(frame_i);

	    MatrixXd tmp_A(6, 9);
	    tmp_A.setZero();
	    VectorXd tmp_b(6);
	    tmp_b.setZero();

	    double dt = (*frame_j)->mIMUPreint.getDeltaTime();

	    Matrix3d Rwc_i = (*frame_i)->mTicp.block<3,3>(0,0).cast<double>();
	    Matrix3d Rwc_j = (*frame_j)->mTicp.block<3,3>(0,0).cast<double>();
	    Matrix3d Rcw_i = Rwc_i.transpose();
	    Matrix3d Rcw_j = Rwc_j.transpose();
	    Matrix3d Rbw_i = Rbc * Rcw_i;
	    Matrix3d Rwb_j = Rwc_j * Rcb;
	    Vector3d pwc_i = (*frame_i)->mTicp.block<3,1>(0,3).cast<double>();
	    Vector3d pwc_j = (*frame_j)->mTicp.block<3,1>(0,3).cast<double>();
	    
	    tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
	    tmp_A.block<3, 2>(0, 6) = -Rbw_i * dt * dt / 2 * Matrix3d::Identity() * lxly;
	    tmp_A.block<3, 1>(0, 8) = Rbw_i * (pwc_j - pwc_i) / 100.0;     
	    tmp_b.block<3, 1>(0, 0) = (*frame_j)->mIMUPreint.getDeltaP() + Rbw_i * (Rwc_i - Rwc_j) * pcb + Rbw_i * dt * dt / 2 * g0;

	    tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
	    tmp_A.block<3, 3>(3, 3) = Rbw_i * Rwb_j;
	    tmp_A.block<3, 2>(3, 6) = -Rbw_i * dt * Matrix3d::Identity() * lxly;
	    tmp_b.block<3, 1>(3, 0) = (*frame_j)->mIMUPreint.getDeltaV() + Rbw_i * dt * Matrix3d::Identity() * g0;

	    Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
	    cov_inv.setIdentity();

	    MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
	    VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

	    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
	    b.segment<6>(i * 3) += r_b.head<6>();

	    A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
	    b.tail<3>() += r_b.tail<3>();

	    A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
	    A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
	}
	A = A * 1000.0;
	b = b * 1000.0;
	x = A.ldlt().solve(b);
	VectorXd dg = x.segment<2>(n_state - 3);
	g0 = (g0 + lxly * dg).normalized() * GI.norm();
	cout << g0.norm() << endl;
	cout<<"g0: "<<g0.transpose()<<endl;
    }   
    g = g0;
}

bool preintegration_opt::LinearAlignment(vector< Velodyne_SLAM::Frame* > vFrames, Vector3d& g, VectorXd &x)
{
    // Extrinsics parameters
    SE3d Tbc = vill::ConfigParam::GetSE3Tbl();
    SE3d Tcb = Tbc.inverse();
    Matrix3d Rbc = Tbc.rotationMatrix();
    Vector3d pbc = Tbc.translation();
    Matrix3d Rcb = Tcb.rotationMatrix();
    Vector3d pcb = Tcb.translation();
    
    int all_frame_count = vFrames.size();
    int n_state = all_frame_count * 3 + 3 + 1;
    
    Eigen::MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();
    
    vector<Velodyne_SLAM::Frame*>::iterator frame_i;
    vector<Velodyne_SLAM::Frame*>::iterator frame_j;
    int i = 0;
    for (frame_i = vFrames.begin(); next(frame_i) != vFrames.end(); frame_i++, i++)
    {
	frame_j = next(frame_i);

	MatrixXd tmp_A(6, 10);
	tmp_A.setZero();
	VectorXd tmp_b(6);
	tmp_b.setZero();

	double dt = (*frame_j)->mIMUPreint.getDeltaTime();
	
	Matrix3d Rbl = vill::ConfigParam::GetEigTbl().block<3,3>(0,0);
	Matrix3d Rwc_i = (*frame_i)->mTicp.block<3,3>(0,0).cast<double>();
	Matrix3d Rwc_j = (*frame_j)->mTicp.block<3,3>(0,0).cast<double>();
	Matrix3d Rcw_i = Rwc_i.transpose();
	Matrix3d Rcw_j = Rwc_j.transpose();
	Matrix3d Rbw_i = Rbl * Rcw_i;
	Matrix3d Rwb_j = Rwc_j * Rcb;
	Vector3d pwc_i = (*frame_i)->mTicp.block<3,1>(0,3).cast<double>();
	Vector3d pwc_j = (*frame_j)->mTicp.block<3,1>(0,3).cast<double>();
	
	tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
	tmp_A.block<3, 3>(0, 6) = -Rbw_i * dt * dt / 2 * Matrix3d::Identity();
	tmp_A.block<3, 1>(0, 9) = Rbw_i * (pwc_j - pwc_i) / 100.0;     
	tmp_b.block<3, 1>(0, 0) = (*frame_j)->mIMUPreint.getDeltaP() + Rbw_i * (Rwc_i - Rwc_j) * pcb;

	tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
	tmp_A.block<3, 3>(3, 3) = Rbw_i * Rwb_j;
	tmp_A.block<3, 3>(3, 6) = -Rbw_i * dt * Matrix3d::Identity();
	tmp_b.block<3, 1>(3, 0) = (*frame_j)->mIMUPreint.getDeltaV();

	Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
	cov_inv.setIdentity();

	MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
	VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

	A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
	b.segment<6>(i * 3) += r_b.head<6>();

	A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
	b.tail<4>() += r_b.tail<4>();

	A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
	A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    double s = x(n_state - 1) / 100.0;

    g = x.segment<3>(n_state - 4);
    
    Vector3d gI(0, 0, 1.0);
    Vector3d GI = gI * ConfigParam::GetG();
    
    if(fabs(g.norm() - GI.norm()) > 1.0 || s < 0)
    {
	return false;
    }

    cout<<"g: "<<g.transpose()<<endl;
    RefineGravity(vFrames, g, x);
    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    
    if(s < 0.0 )
	return false;   
    else
	return true;
    

}

// void preintegration_opt::setGravity(Vector3d& gravity)
// {
//   boost::mutex::scoped_lock lock(mMutexGravity);
//   LIV::preintegration_opt::mGravityVec = gravity;
// 
// }

// Vector3d preintegration_opt::getGravity()
// {
//   boost::mutex::scoped_lock lock(mMutexGravity);
//   return LIV::preintegration_opt::mGravityVec;
// }

void preintegration_opt::setBiasUpdated(bool flag)
{
  boost::mutex::scoped_lock lock(mMutexBiasUpdated);
  bBiasUpdated = flag;
}

bool preintegration_opt::checkBiasUpdated()
{
  boost::mutex::scoped_lock lock(mMutexBiasUpdated);
  return bBiasUpdated;
}

Vector3d preintegration_opt::getdbiasa()
{
  boost::mutex::scoped_lock lock(mMutexBiasUpdated);
  return dbiasa;
}


Vector3d preintegration_opt::getdbiasg()
{
  boost::mutex::scoped_lock lock(mMutexBiasUpdated);
  return dbiasg;
}

Vector3d preintegration_opt::getVelo()
{
  boost::mutex::scoped_lock lock(mMutexBiasUpdated);
  return velo;
}



bool preintegration_opt::processFrames(std::vector< Velodyne_SLAM::Frame* >& pFs)
{
  int N = pFs.size();
  
  if(pFs.size() == 1) return false;
  Vector3d tempGravity = imu_preintegration::getGravity();
  
//   int delta = 5;
  
  vector<Velodyne_SLAM::Frame*> simplified_Frames;
  simplified_Frames.push_back(pFs[0]);
  for(int i = 0 + delta; i < N; i = i + delta){
    vector<vill::IMUData> tempimudata;
    cout<<">"<<tempimudata.size()<<endl;
    for(int j = (delta-1); j > 0; j--){
      cout<<"? "<<pFs[i-j]->mnId<<endl;
      tempimudata.insert(tempimudata.end(), pFs[i-j]->mvIMUdatas.begin(), pFs[i-j]->mvIMUdatas.end());
      cout<<">"<<tempimudata.size()<<endl;
    }
    cout<<"<"<<pFs[i]->mvIMUdatas.size()<<endl;
    pFs[i]->mvIMUdatas.insert(pFs[i]->mvIMUdatas.begin(), tempimudata.begin(), tempimudata.end());
    cout<<"<"<<pFs[i]->mvIMUdatas.size()<<endl;
    pFs[i]->previousF_KF = pFs[i-delta];
    pFs[i]->ComputePreInt();
    simplified_Frames.push_back(pFs[i]);
    tempimudata.clear();
  }
//   
  
  
  Vector3d pdbiasa, pdbiasg, pvelo;
  bool res = Velodyne_SLAM::Optimizer::OptimizePosewithIMU(simplified_Frames, tempGravity, pdbiasa, pdbiasg, pvelo);
  
  {
    boost::mutex::scoped_lock lock(mMutexBiasUpdated);
    dbiasa = pdbiasa;
    dbiasg = pdbiasg;
    velo = pvelo;
    cout<<"dba: "<<dbiasa.transpose()<<endl;
    cout<<"dbg: "<<dbiasg.transpose()<<endl;
    cout<<"vel: "<<velo.transpose()<<endl;
  }
  setBiasUpdated(true);
  geometry_msgs::Vector3 velo_msg;
  
  velo_msg.x = pvelo[0];
  velo_msg.y = pvelo[1];
  velo_msg.z = pvelo[2];
  
  geometry_msgs::Twist dbias_msg;
  dbias_msg.linear.x = pdbiasa[0];
  dbias_msg.linear.y = pdbiasa[1];
  dbias_msg.linear.z = pdbiasa[2];
  dbias_msg.angular.x = pdbiasg[0];
  dbias_msg.angular.y = pdbiasg[1];
  dbias_msg.angular.z = pdbiasg[2];
  
  vel_pub.publish(velo_msg);
  dbias_pub.publish(dbias_msg);
  
//   mimu_preintegrator->setBiasa(pdbiasa);
//   mimu_preintegrator->setBiasg(pdbiasg);
//   mimu_preintegrator->setBiasa_g(pdbiasa, pdbiasg);
//   mimu_preintegrator->setBiasUpdated(true);

  
//   for(int i = 0; i < (pFs.size()-1); i++){
//     if(pFs[i]->isProcessed()){
//       delete pFs[i];
//     }
//   }
  return true;
  

}

void preintegration_opt::processLaserFrames()
{	
	if(lFrames.size() == 1) return;
	
	Vector3d pdbiasa, pdbiasg, pvelo;
	
	Vector3d tempGravity = imu_preintegration::getGravity();
	
	vector<Velodyne_SLAM::Frame* > frames_in_window;
	
	list<Velodyne_SLAM::Frame*>::iterator it_lFrames = lFrames.begin();
// 	for(; it_lFrames != lFrames.end(); it_lFrames++){
// 		Velodyne_SLAM::Frame* pF = *it_lFrames;
// 		pF->ComputePreInt();
// 		frames_in_window.push_back(pF);
// 	}
	
	vector<Velodyne_SLAM::Frame*> vFrames, simplified_frames;
	vFrames.assign(lFrames.begin(), lFrames.end());
	
	simplified_frames.push_back(vFrames[0]);
	
  for(int i = 0 + delta; i < vFrames.size(); i = i + delta){
    vector<vill::IMUData> tempimudata;

    for(int j = 1; j < delta; j++){

      tempimudata.insert(tempimudata.end(), vFrames[i-delta+j]->mvIMUdatas.begin(), vFrames[i-delta+j]->mvIMUdatas.end());

    }

    vFrames[i]->mvIMUdatas.insert(vFrames[i]->mvIMUdatas.begin(), tempimudata.begin(), tempimudata.end());


    vFrames[i]->previousF_KF = vFrames[i-delta];
    vFrames[i]->ComputePreInt();
    simplified_frames.push_back(vFrames[i]);
    tempimudata.clear();
  }
	
	if(simplified_frames.size() < 2)
		return;
	
	bool res = Velodyne_SLAM::Optimizer::OptimizePosewithIMU(simplified_frames, tempGravity, pdbiasa, pdbiasg, pvelo);
	
	mimu_preintegrator->setBiasa(pdbiasa);
	mimu_preintegrator->setBiasg(pdbiasg);
	
	it_lFrames = lFrames.begin();
	for(; it_lFrames != lFrames.end(); it_lFrames++){
		Velodyne_SLAM::Frame* pF = *it_lFrames;
		int imuId = pF->getIMUId();
		double oriTime = mimu_preintegrator->v_NavStates[imuId].Get_Time();
		mimu_preintegrator->v_NavStates[imuId] = pF->GetNavState();
		mimu_preintegrator->v_NavStates[imuId].Set_Time(oriTime);
	}
	
	mimu_preintegrator->setUpdateNavState(lFrames.back()->getIMUId());
// 	lFrames.clear();
	
	

}


void preintegration_opt::setimu_preintegration(imu_preintegration* pimu_preintegrator)
{
  mimu_preintegrator = pimu_preintegrator;

}


}