#include <imu_preintegration.h>
#include <pointmatcher/Timer.h>
#include "glog/logging.h"
#include "ros/package.h"
#include "Optimizer.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <math.h> 
// #include "pointmatcher_ros/get_params_from_server.h"

using namespace std;


namespace LIV{
Vector3d imu_preintegration::gw = Vector3d();
boost::mutex imu_preintegration::mMutexGravity;
  
imu_preintegration::imu_preintegration(ros::NodeHandle& nh):
  n(nh),init(false)
//   laserFrame(getParam<string>("odom_frame", "/velodyne")),
//   imuFrame(getParam<string>("imu_frame","/imu"))
{
  if(! ros::param::get("~imu_msg_name", imu_msg_str)){
    imu_msg_str = "/imu";
  }
  imupreintegrator.reset();
  cerr << "======== imu msg name: " << imu_msg_str << endl;
  
  if(! ros::param::get("~imu_frame", imuFrame))
    imuFrame = "/imu";
  
  if(! ros::param::get("~laser_frame", laserFrame))
    laserFrame = "/velodyne";
  
  if(! ros::param::get("~map_frame", mapFrame))
    laserFrame = "/map";
  
  if(! ros::param::get("~initial_time", initial_time)){
    ROS_ERROR_STREAM("Can not load initial time");
    initial_time = 5.0;
  }else{
    ROS_INFO_STREAM("Load initial_time "<<initial_time);
  }
  
    
  T = PM::TransformationParameters::Identity(4,4);
  Tinit = PM::TransformationParameters::Identity(4,4);
  mGravity  = Vector3d();
  mGravity(0) = 0; mGravity(1) = 0;
  mGravity(2) = -9.81;
  
  Tlb = vill::ConfigParam::GetEigT_lb().cast<float>();
  
  setBiasa_g(Vector3d(0,0,0),Vector3d(0,0,0));
  
  f.open(ros::package::getPath("laser_mapping")+"/Log/imu.txt");
  initialization_cnt = 0;
//   = new std::thread(&preintegration_opt::imuIniting, this);
  LOG(INFO)<<"Please keep the robot still until the gravity has been estimated...";
  mCurrentState = NOT_INITIALIZED;
  CoarseInitialized = false;
  mvelo = Vector3f(0, 0,0);
  Tcur = PM::TransformationParameters::Identity(4,4);
  
  imu_pub = n.advertise<nav_msgs::Odometry>("imu_odom", 50, true);
//   CoarseInitializationThread = new std::thread(&imu_preintegration::CoarseInitialization, this);
}

imu_preintegration::~imu_preintegration()
{

}



void imu_preintegration::Run()
{
  //imu_sub = n.subscribe(imu_msg_str.c_str(),1,&imu_preintegration::gotIMU,this);
  dbias_sub = n.subscribe("opt_dbias", 1, &imu_preintegration::gotOptDbias, this);


}


void imu_preintegration::gotIMU(const sensor_msgs::Imu& imuMsgIn)
{
  addIMUMsg(imuMsgIn);

}

void imu_preintegration::gotOptDbias(const geometry_msgs::Twist& dbiasMsgIn)
{
//   cout<<"got opt dbias dbias~~~~~~~~~~~~~"<<endl;
  boost::mutex::scoped_lock lock(mMutexBias);
  biasa += Vector3d(dbiasMsgIn.linear.x, dbiasMsgIn.linear.y, dbiasMsgIn.linear.z);
  biasg += Vector3d(dbiasMsgIn.angular.x, dbiasMsgIn.angular.y, dbiasMsgIn.angular.z);
  return;

}

void imu_preintegration::gotOptmveloFrame(const geometry_msgs::Vector3& optVelMsgIn)
{
  boost::mutex::scoped_lock lock(mMutexVel);
  mvelo = Vector3f(float(optVelMsgIn.x), float(optVelMsgIn.y), float(optVelMsgIn.z));
  return;
}

Vector3f imu_preintegration::getVel()
{
  boost::mutex::scoped_lock lock(mMutexVel);
  return mvelo;
}

void imu_preintegration::setCurLaserPose(PointMatcher< float >::TransformationParameters& T)
{
  boost::mutex::scoped_lock lock(mMutexCurPose);
  Tcur = T;
}

PointMatcher< float >::TransformationParameters imu_preintegration::getCurLaserPose()
{
  boost::mutex::scoped_lock lock(mMutexCurPose);
  return Tcur;

}




void imu_preintegration::CoarseInitialization()
{
  
  LOG(INFO)<<"accept "<<lIMUdata.size()<<" imu data for initialization"<<endl;
  //estimate the bias of gyro
  Vector3d gyro_w = Vector3d(0,0,0);
  int not_nan = 0;
  for(int i = 0; i < lIMUdata.size(); i++){
    Vector3f pData = lIMUdata[i]._g.cast<float>();
    if(isnan(pData(0)) || isnan(pData(1)) || isnan(pData(2)) ){
      continue;
      
    }
    not_nan++;
    gyro_w(0) = gyro_w(0) + lIMUdata[i]._g(0);
    gyro_w(1) = gyro_w(1) + lIMUdata[i]._g(1);
    gyro_w(2) = gyro_w(2) + lIMUdata[i]._g(2);
  }
  
  gyro_w = gyro_w / double(lIMUdata.size());
  biasg = gyro_w;
  
  LOG(INFO)<<"estimated bias of gyro from "<<not_nan<<" data\n"<<biasg<<endl<<endl;
  

  //should do not have to add mutex, single thread until now
  Velodyne_SLAM::Optimizer::OptimizeInitialGravityBa(lIMUdata, pitch_roll, biasa);
  
}




void imu_preintegration::addIMUMsg(const sensor_msgs::Imu& imuMsgIn)
{

  //At the beginning, the robot should be still until the gravity has been estimated.
  if(mCurrentState == NOT_INITIALIZED){
      vill::IMUData imudata(imuMsgIn.angular_velocity.x,imuMsgIn.angular_velocity.y,imuMsgIn.angular_velocity.z,
			  imuMsgIn.linear_acceleration.x,imuMsgIn.linear_acceleration.y,imuMsgIn.linear_acceleration.z,
			  imuMsgIn.header.stamp.toSec());
    if(lIMUdata.size() > 3){
      if((fabs(imudata._g(0)-lIMUdata.back()._g(0)) > 0.015 )|| 
  	    (fabs(imudata._g(1)-lIMUdata.back()._g(1)) > 0.015 ) ||
      	(fabs(imudata._g(2)-lIMUdata.back()._g(2)) > 0.015) || 
      	(imudata._t - lIMUdata.front()._t) > 5){
    	
      	CoarseInitialization();
            
        //T^b0_n
      	initialrotation = (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) 
      		    * Eigen::AngleAxisd(pitch_roll[0], Eigen::Vector3d::UnitY())
      		    * Eigen::AngleAxisd(pitch_roll[1], Eigen::Vector3d::UnitX())).toRotationMatrix();
      		    
      	
  	
  	//compute the initial gravity//TEST
  // 	mGravity = initialrotation * mGravity;
      	gw = mGravity;
      	
      	LOG(INFO)<<"Gravity:\n"<<mGravity;
      	LOG(INFO)<<"curGravity\n"<<initialrotation * mGravity;
      	setCoarseInitialized(true);
      	
      	//T^n_b0
      	Tinit.block<3,3>(0,0) = initialrotation.inverse().cast<float>();
      // 	Tinit = Tinit ;
      	
      	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Tinit,mapFrame, imuFrame, ros::Time::now()));
      	tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(
      	 Tinit * vill::ConfigParam::GetEigTbl().cast<float>(),mapFrame, laserFrame,ros::Time::now()));
        tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>( 
          vill::ConfigParam::GetEigTbl().inverse().cast<float>(), laserFrame, imuFrame, ros::Time::now()));
      	mCurrentState = WORKING;
	
      }
    }
    lIMUdata.push_back(imudata);
  }else{
     
	  double ax = imuMsgIn.linear_acceleration.x;
	  double ay = imuMsgIn.linear_acceleration.y;
	  double az = imuMsgIn.linear_acceleration.z;
	  
	  vill::IMUData imudata(imuMsgIn.angular_velocity.x,imuMsgIn.angular_velocity.y,imuMsgIn.angular_velocity.z,
			  ax,ay,az,imuMsgIn.header.stamp.toSec());
	  PM::TransformationParameters curTT = getCurLaserPose().inverse();
// 	  cout<<imuMsgIn.header.stamp<<endl;
	  if(!init){
	      {
    		  boost::mutex::scoped_lock lock(mMutexIMUPreint);
    		  lIMUdata.clear();////TEST whether add the time of last frame 按照统计来说，是不是补一个平均值
    		  lIMUdata.push_back(imudata);
    		  imupreintegrator.reset();
		  
	      }
	      init = true;
	  }else{
	    Vector3d _biasg = getBiasa();
	    Vector3d _biasa = getBiasg();
	    Vector3f velInLaserFrame = getVel();

	    {
	      boost::mutex::scoped_lock lock(mMutexIMUPreint);
	      double dt = imudata._t - lIMUdata.back()._t;
	      double dt_all = imudata._t- lIMUdata.front()._t;
	      lIMUdata.push_back(imudata);
	      imupreintegrator.update(imudata._g-_biasg, imudata._a-_biasa, dt);
// 	      Vector3d gw = mppreintegration_opter->getGravity();
	      T.block<3,1>(0,3) = imupreintegrator.getDeltaP().cast<float>() ;
	      T.block<3,3>(0,0) = imupreintegrator.getDeltaR().cast<float>();
	      T = Tlb * T; 
	      
	      T.block<3,1>(0,3) += /*(curT.block<3,3>(0,0) * (velInLaserFrame * (float)(dt_all)) )
	      +*/ curTT.block<3,3>(0,0) * (0.5 * mGravity * dt_all *dt_all).cast<float>() ;
	    }
	      
	      //tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>( T, laserFrame, imuFrame, imuMsgIn.header.stamp));
	      //imu_pub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(curTT.inverse()*T, mapFrame, imuMsgIn.header.stamp));
	      tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>( T, laserFrame, imuFrame, ros::Time::now()));
        vT_laser_imu.push_back(T);
        vT_times.push_back(imuMsgIn.header.stamp.toSec());
        imu_pub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(curTT.inverse()*T, mapFrame, ros::Time::now()));
        
       
	  }
  }
// 	  LOG(INFO)<<t.elapsed();
}

void imu_preintegration::getGyroMsg(Eigen::Vector3d & gw, double time)
{
   boost::mutex::scoped_lock lock(mMutexIMUPreint);
   
   double dt0 = fabs(time - lIMUdata.front()._t);
   double dt1 = fabs(time - lIMUdata.back()._t);
   
   if(dt0 < dt1){
     gw = lIMUdata.front()._g;
  }else{
    gw = lIMUdata.back()._g;
  }
  

}

void imu_preintegration::resetPreintegrator()
{
//   boost::mutex::scoped_lock lock(mMutexIMUPreint);
//   lIMUdata.clear();
  init = false;

}

vill::IMUPreintegrator imu_preintegration::getEvaluatedOdomPreintegration()
{
  boost::mutex::scoped_lock lock(mMutexIMUPreint);
  return imupreintegrator;

}

vector< vill::IMUData >& imu_preintegration::getIMUdataVector()
{
  boost::mutex::scoped_lock lock(mMutexIMUPreint);
  return lIMUdata;

}


vector<vill::IMUData>& imu_preintegration::getIMUdataVector(double fromtime, double totime){
  vector<vill::IMUData> vimu;
  {
    boost::mutex::scoped_lock lock(mMutexIMUPreint);
    vimu = lIMUdata;
  }
  
  
}

void imu_preintegration::setCoarseInitialized(bool flag)
{
  boost::mutex::scoped_lock lock(mMutexIMUCoarseInitialized);
  CoarseInitialized = flag;

}

bool imu_preintegration::checkCoarseInitialized()
{
  boost::mutex::scoped_lock lock(mMutexIMUCoarseInitialized);
  return CoarseInitialized;

}

Vector3d imu_preintegration::getGravity()
{
  boost::mutex::scoped_lock lock(mMutexGravity);
  return gw;
}

PointMatcher< float >::TransformationParameters imu_preintegration::getInitialT()
{
  return Tinit;
}

Vector3d imu_preintegration::getBiasa()
{
  boost::mutex::scoped_lock lock(mMutexBias);
//   mMutexBias.lock();
  return biasa;

}

Vector3d imu_preintegration::getBiasg()
{
  boost::mutex::scoped_lock lock(mMutexBias);
  return biasg;

}

void imu_preintegration::setBiasa(Vector3d& mbiasa)
{
//   boost::mutex::scoped_lock lock(mMutexBias);
  mMutexBias.lock();
  biasa = mbiasa + biasa;
  mMutexBias.unlock();
}

void imu_preintegration::setBiasg(Vector3d& mbiasg)
{
  boost::mutex::scoped_lock lock(mMutexBias);
  biasg = mbiasg + biasg;

}

void imu_preintegration::setBiasa_g(const Vector3d mdbiasa, const Vector3d mdbiasg)
{
//   boost::mutex::scoped_lock lock(mMutexBias);
  mMutexBias.lock();
  biasa += mdbiasa;
  biasg += mdbiasg;
  biasUpdated = true;
  mMutexBias.unlock();
  return ;
}



bool imu_preintegration::checkBiasUpdated()
{
  boost::mutex::scoped_lock lock(mMutexBias);
  return biasUpdated;
}

void imu_preintegration::setBiasUpdated(bool flag)
{
  boost::mutex::scoped_lock lock(mMutexBias);
  biasUpdated = flag;
}


  
}

