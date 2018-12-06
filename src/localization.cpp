#include "ros/ros.h"
#include "ros/console.h"
#include<iostream>
#include<fstream>
#include<ros/package.h>
#include<boost/thread.hpp>

#include "icp_mapper.h"
#include "Map.h"
#include "Frame.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

#include "imu_preintegration.h"
#include "odom_preintegration.h"
#include "IMU/configparam.h"
#include "preintegration_opt.h"
using namespace std;
using namespace PointMatcherSupport;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "localization");
	ros::start();
	
	cout<<"Localization with laser and IMU..."<<endl;
	
	ros::NodeHandle n;
	ros::MultiThreadedSpinner spinner(4);
	
	bool use_imu = false, use_odom = false; 
	
	ros::param::get("~use_imu", use_imu);
	cerr<<"use imu data => "<<use_imu;
	
	ros::param::get("~use_odom", use_odom);
	cerr<<"use odom data => " << use_odom;
	
	
	string extrinsicTLC;
	ros::param::get("~extrinsicTLC", extrinsicTLC);
	string refinefile;
	ros::param::get("~refinefile", refinefile);
	string extrinsicTBC;
	ros::param::get("~extrinsicTBC", extrinsicTBC);
	
	Velodyne_SLAM::Map World;
	vill::ConfigParam config(extrinsicTLC, refinefile, extrinsicTBC);
	
	Velodyne_SLAM::LocalMapping localMapper(&World, n);
	boost::thread LocalMappingThread(&Velodyne_SLAM::LocalMapping::RunSlidingMap, &localMapper);
	
// laser 
	Velodyne_SLAM::ICPMapper mapper(n,&World,  use_odom, use_imu);
	boost::thread ICPMapperThread(&Velodyne_SLAM::ICPMapper::RunSlidingMap,&mapper);
	
	LIV::imu_preintegration imu_preintegrator(n);
	LIV::preintegration_opt preintegration_optimizer;
	if(use_imu){
	    //imu
	    boost::thread ImuPreintegrationThread(&LIV::imu_preintegration::Run, &imu_preintegrator);
	    boost::thread PreintegrationThread(&LIV::preintegration_opt::Run, &preintegration_optimizer);
	    
	    mapper.setImuPre(&imu_preintegrator);
	    mapper.setPreintegrationOpt(&preintegration_optimizer); 
	}
	
	
	mapper.SetLocalMapper(&localMapper);

// 	//odom
	LIV::odom_preintegration odom_preintegrator(n);
	if(use_odom)
	  boost::thread OdomPreintegrationThread(&LIV::odom_preintegration::Run, &odom_preintegrator);
	
	localMapper.SetIcpMapper(&mapper);
	
	spinner.spin();
	localMapper.Stopping();
	
	return 0;
	
	
}