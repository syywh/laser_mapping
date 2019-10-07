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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "sensor_msgs/NavSatFix.h"

#include <poslvx/INS.h>
#include <poslvx/INSRMS.h>
#include <poslvx/Status.h>


using namespace std;
using namespace PointMatcherSupport;

void   Delay(int   time)//time*1000为秒数 
{ 
	clock_t   now   =   clock(); 

	while(   clock()   -   now   <   time   ); 
} 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_mapping_with_bag");
	ros::start();
	
	ros::NodeHandle n;
	ros::MultiThreadedSpinner spinner(4);
	
	bool use_imu = false, use_odom = false, use_gps = false; 
	
	ros::param::get("~use_imu", use_imu);
	cerr << "use imu data => " << use_imu;
	
	ros::param::get("~use_odom", use_odom);
	cerr << "  use odom data => " << use_odom;
	
	ros::param::get("~use_gps", use_gps);
	cerr << "  use gps data => " << use_gps;
	
	string task;
	ros::param::get("~TASK", task);
	
	string extrinsicTLC;
	ros::param::get("~extrinsicTLC", extrinsicTLC);
	string refinefile;
	ros::param::get("~refinefile", refinefile);
	string extrinsicTBC;
	ros::param::get("~extrinsicTBC", extrinsicTBC);

// read bag
	string bagname;
	if(!ros::param::get("~bagname",bagname))
	{
		ROS_ERROR("Can not find bag name");
		exit(0);
	}
    rosbag::Bag databag;
    databag.open(bagname, rosbag::bagmode::Read);


	Velodyne_SLAM::Map World;
	vill::ConfigParam config(extrinsicTLC, refinefile, extrinsicTBC);


	Velodyne_SLAM::LocalMapping localMapper(&World, n);
	Velodyne_SLAM::LoopClosing LoopCloser(&World);
	Velodyne_SLAM::ICPMapper mapper(n,&World, use_odom, use_imu);
	Velodyne_SLAM::MapPublisher MapPub(&World, &localMapper,&LoopCloser);
	LIV::imu_preintegration imu_preintegrator(n);
	LIV::preintegration_opt preintegration_optimizer;
	LIV::odom_preintegration odom_preintegrator(n);
	
	mapper.SetLocalMapper(&localMapper);

	localMapper.SetIcpMapper(&mapper);
	localMapper.SetLoopClosing(&LoopCloser);
	
	LoopCloser.SetIcpMapper(&mapper);
	LoopCloser.SetLocalMapper(&localMapper);
	
// 	boost::thread LocalMappingThread(&Velodyne_SLAM::LocalMapping::Run, &localMapper);

// 	boost::thread LoopClosingThread(&Velodyne_SLAM::LoopClosing::Run, &LoopCloser);
	
	if(task == "with_map"){
		string map_dir;
		ros::param::get("~MapReadingFile", map_dir);
		World.load(map_dir);
		MapPub.Refresh();
		
		while(!(mapper.initializationforReSLAM()) && ros::ok()){}
		cout<<"ReSLAM initialization done"<<endl;
	}


    vector<string> topics;
    topics.push_back(mapper.getCloudName());
	
//imu	
	if(use_imu){
// 		boost::thread ImuPreintegrationThread(&LIV::imu_preintegration::Run, &imu_preintegrator);
// 	    boost::thread PreintegrationThread(&LIV::preintegration_opt::Run, &preintegration_optimizer);
    
	    mapper.setImuPre(&imu_preintegrator);
	    mapper.setPreintegrationOpt(&preintegration_optimizer); 
	    topics.push_back(imu_preintegrator.getIMUName());
		preintegration_optimizer.setimu_preintegration(&imu_preintegrator);
	}
	
// 	//odom
	if(use_odom){
	  mapper.setOdoPre(&odom_preintegrator);
	  topics.push_back(odom_preintegrator.getOdomName());
	}

	if(use_gps){
		string gps_msg_name;
		ros::param::get("~gps_msg_name", gps_msg_name);
		cerr << "  gps_msg_name => " << gps_msg_name;
		topics.push_back(gps_msg_name);
	}
	
	boost::thread mapPublisherThread(&Velodyne_SLAM::MapPublisher::Run, &MapPub);

	rosbag::View view(databag, rosbag::TopicQuery(topics));

	ros::Rate r(10);
	

	LOG(INFO) << "process begins...";
	foreach(rosbag::MessageInstance const m, view){


		sensor_msgs::Imu::Ptr imuData = m.instantiate<sensor_msgs::Imu>();
		if(use_imu && imuData){
// 			ROS_INFO_STREAM("read imu data " << imuData->header.stamp);
			imu_preintegrator.addIMUMsg(*imuData);

		}

		nav_msgs::Odometry::Ptr odomData = m.instantiate<nav_msgs::Odometry>();
		if(use_odom && odomData){
			ROS_INFO("read odom data");

		}
		
		sensor_msgs::NavSatFix::Ptr gpsData = m.instantiate<sensor_msgs::NavSatFix>();
		if(use_gps && gpsData){
			ROS_INFO("read gps data");
			mapper.gotGPSNavSatFix(*gpsData);
		}
		
		poslvx::INS::Ptr gpsInsData = m.instantiate<poslvx::INS>();
		if(use_gps && gpsInsData){
// 			ROS_INFO("read ins data");

			gps_common::GPSFix msg;
			msg.header = gpsInsData->header;
			msg.altitude = gpsInsData->altitude;
			msg.latitude = gpsInsData->latitude;
			msg.longitude = gpsInsData->longitude;
			msg.status.satellites_used = gpsInsData->status.gnss_status;
			msg.pitch = gpsInsData->pitch;
			msg.roll = gpsInsData->roll;
// 			cout <<"gps status " << msg.status.status <<" " <<gpsInsData->status.gnss_status<< endl;
			
			mapper.gotGPSFix(msg, gpsInsData->heading);
		}
		

		sensor_msgs::PointCloud2::Ptr lidarData = m.instantiate<sensor_msgs::PointCloud2>();
		if (lidarData != NULL){
// 			ROS_INFO_STREAM("read laser data " << lidarData->header.stamp);
		
			mapper.gotCloud(*lidarData);
			if( localMapper.CheckNewFrames() &&(localMapper.ProcessLocalMap())){
				localMapper.CreateNewKeyFrames();
			}
// 			while(ros::ok() && !localMapper.hasProcessedOneFrame()){
// 				
// 			}
			localMapper.setProcessedOneFrame(false);
// 			r.sleep();
// 			Delay(100);

		}

		
		//GPS TODO
		
		if(!(ros::ok()))
			break;

		

	}


// 	
	//Now Only One File open
	localMapper.Stopping();
	mapper.stop();
	
	
	string fileName; 
	
	MapPub.Refresh();
	
	cerr<<"check distance "<<LoopCloser.checkAllKeyFrameByDistance()<<endl;
	LoopCloser.computeICP();
	

	if( !(ros::param::get("~MapSavingFile", fileName)))
	  fileName= ros::package::getPath("laser_mapping")+"/Map/Map.xml";

// 	World.save(fileName);
	World.savewithGlobalPose(fileName);
	cerr<<"Map xml  saved! "<<endl;
	World.saveMap(true);
	cerr<<"Map txt saved!"<<endl;
	mapper.recordKF.close();
	cerr<<"record txt saved!"<<endl;
// 	World.fMap.close();
	return 0;
	
	
}
