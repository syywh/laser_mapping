#include "icp_mapper.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"
#include "fstream"
#include "Frame.h"
#include "vtr/congfig.h"
#include "glog/logging.h"

#define DEBUG_ICP

const double g3dm = 9.80665;

using namespace std;
using namespace PointMatcherSupport;
namespace Velodyne_SLAM {


ICPMapper::ICPMapper(ros::NodeHandle& nh,Map* mMap, bool use_odom, bool use_imu):
	buse_odom(use_odom),
	buse_imu(use_imu),
	n(nh),
	mapPointCloud(0),
	transformation(PM::get().REG(Transformation).create("RigidTransformation")),
	processingNewCloud(false),
	publishMapTf(getParam<bool>("publishMapTf", true)),
// 	useConstMotionModel(getParam<bool>("useConstMotionModel", false)),
	localizing(getParam<bool>("localizing", true)),
	mapping(getParam<bool>("mapping", true)),
	minReadingPointCount(getParam<int>("minReadingPointCount", 1000)),
	minMapPointCount(getParam<int>("minMapPointCount", 10000)),
	inputQueueSize(getParam<int>("inputQueueSize", 1)),
	minOverlap(getParam<double>("minOverlap", 0.2)),
	maxOverlapToMerge(getParam<double>("maxOverlapToMerge", 0.99)),
	tfRefreshPeriod(getParam<double>("tfRefreshPeriod", 0.005)),
	laserFrame(getParam<string>("laser_frame", "/velodyne")),
	mapFrame(getParam<string>("map_frame", "/map")),
	imuFrame(getParam<string>("imu_frame", "/imu")),
	vtkFinalMapName(getParam<string>("vtkFinalMapName", "finalMap.vtk")),
	mapElevation(getParam<double>("mapElevation", 0)),
	priorStatic(getParam<double>("priorStatic", 0.45)),
	priorDyn(getParam<double>("priorDyn", 0.55)),
	TimuToMap(PM::TransformationParameters::Identity(4, 4)),
	publishStamp(ros::Time::now()),
  tfListener(ros::Duration(15000)),
	eps(0.0001),
	Index(0),
	mpMap(mMap),
	Debug_ICP(getParam<bool>("Debug_ICP", false)),
	OKForCreateNewKeyFrame(true),
	OKProcessNewFrame(true),
	lastProcessedKeyFrame(0),
	checkLocalizing(0),
	halfRoadWidth(getParam<float>("halfRoadWidth", 3)),
	MaxLocalMappoints(getParam<float>("MaxLocalMappoints", 25000)),
	KFthreOverlap(getParam<float>("KFthreOverlap", 0.92)),
	maxAngle(getParam<float>("maxAngleforKF", 0.6)),
	maxLocalMapLength(getParam<float>("maxLocalMapLength", 20)),
	message_filter_cloud(n, "/velodyne_points", 3),
	message_filter_gps_position(n, "/rtk_gps/position",3),
	message_filter_gps_orientation(n, "rtk_gps/orientation",3),
	 synchronizer_(PointCloudwithGPS(10), message_filter_cloud, message_filter_gps_position, message_filter_gps_orientation),
	 cloud_in(getParam<string>("cloud_in", "/velodyne_points")),
	 imu_in(getParam<string>("imu_in","/mti/sensor/imu")),
	 bAccMultiply98(false)
{
	cnt_for_lost_imu = 0;
	reInitialized = true;
	if(mpMap->getKeyFrameNum())
	{
		mCurrentState = NOT_LOCALIZEING;
		firstICP = PM::TransformationParameters::Identity(4,4);
		string posefile;
		if(ros::param::get("~firstPose",posefile))
		{
			loadPosetxt(posefile, firstICP);
			cerr<<"Load first Pose \n"<<firstICP<<endl;
		}
		else
		{
			cerr<<"No first Pose\n"<<endl;
		}
		
		
	}else
		mCurrentState = NOT_INITIALIZED;
	// Ensure proper states
	if(localizing == false)
		mapping = false;
	if(mapping == true)
		localizing = true;

	// set logger
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);

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
			cerr<<"[ICP] set default"<<endl;
		}
	}
	else
	{
		ROS_INFO_STREAM("No ICP config file given, using $(find laser_mapping)/cfg/icp_dynamic.yaml");
		configFileName = ros::package::getPath("laser_mapping")+"/cfg/icp_dynamic.yaml";
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icp.loadFromYaml(ifs);//ICPChainBase function
			cerr<<"[ICP] icp chain loaded from "<<configFileName<<endl;
		}else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icp.setDefault();
		}
		
	}
	if (ros::param::get("~icpConfigforInit", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icpInit.loadFromYaml(ifs);//ICPChainBase function
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icpInit.setDefault();
			cerr<<"[ICP] set default"<<endl;
		}
	}
	else
	{
		ROS_INFO_STREAM("No ICP config file given, using $(find laser_mapping)/cfg/icp_dynamic.yaml");
		configFileName = ros::package::getPath("laser_mapping")+"/cfg/icp_dynamic_forInit.yaml";
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			icpInit.loadFromYaml(ifs);//ICPChainBase function
			cerr<<"[ICP] icpInit chain loaded from "<<configFileName<<endl;
		}else
		{
			ROS_ERROR_STREAM("Cannot load ICP config from YAML file " << configFileName);
			icpInit.setDefault();
		}
		
	}
	if (getParam<bool>("useROSLogger", false))
		PointMatcherSupport::setLogger(new PointMatcherSupport::ROSLogger);
	
	if (ros::param::get("~inputFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			inputFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No input filters config file given, $(find velodyne_slam)/cfg/input_filters.yaml");
		configFileName = ros::package::getPath("laser_mapping")+"/cfg/input_filters.yaml";
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			inputFilters = PM::DataPointsFilters(ifs);
			cerr<<"[ICP] input filter loaded from "<<configFileName<<endl;
		}else
		{
			ROS_ERROR_STREAM("Cannot load Input filter config from YAML file " << configFileName);

		}
	}
	
	if (ros::param::get("~mapPreFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPreFilters = PM::DataPointsFilters(ifs);
// 			cerr<<"[ICP] icp chain loaded from "<<configFileName<<endl;
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map pre-filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No map pre-filters config file given, $(find velodyne_slam)/cfg/input_filters.yaml");
	}
	
	if (ros::param::get("~mapPostFiltersConfig", configFileName))
	{
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPostFilters = PM::DataPointsFilters(ifs);
		}
		else
		{
			ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file " << configFileName);
		}
	}
	else
	{
		ROS_INFO_STREAM("No map post-filters config file given, $(find velodyne_slam)/cfg/input_filters.yaml");
		configFileName = ros::package::getPath("laser_mapping")+"/cfg/map_post_filters.yaml";
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPostFilters = PM::DataPointsFilters(ifs);
			cerr<<"[ICP] map post-filters loaded from "<<configFileName<<endl;
		}else
		{
			ROS_ERROR_STREAM("Cannot load Input filter config from YAML file " << configFileName);

		}
	}
	
	//T^l_b
	TimuToLaser = PM::TransformationParameters::Identity(4,4);
	Eigen::Matrix4d Tlb = vill::ConfigParam::GetEigT_lb();
	TimuToLaser = Tlb.cast<float>();
	cout<<"TimuToLaser\n"<<TimuToLaser<<endl;
	
	TodomToLaser = PM::TransformationParameters::Identity(4,4);
	TodomToMap = PM::TransformationParameters::Identity(4,4);
	
	
// 	TimuToLaser(0,3) = 0.2;
// 	TimuToLaser(2,3) = -1.255;

	odomPub = n.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
	recordKF.open( ros::package::getPath("laser_mapping")+"/recordKF.txt");
	if(!recordKF.is_open())
	{
		cerr<<"cannot open recordKF"<<endl;
		recordKF.open("recordKF.txt");
	}
	
// 	publishThread = boost::thread(boost::bind(&ICPMapper::publishLoop, this, tfRefreshPeriod));

}

ICPMapper::~ICPMapper()
{
	#if BOOST_VERSION >= 104100
	
	if (mapBuildingInProgress)
	{
		mapBuildingFuture.wait();
		if (mapBuildingFuture.has_value())
			delete mapBuildingFuture.get();
	}
	#endif // BOOST_VERSION >= 104100
	// wait for publish thread
// 	publishThread.join();
	// save point cloud
	if (mapPointCloud)
	{
		mapPointCloud->save(vtkFinalMapName);
		delete mapPointCloud;
	}
}



void ICPMapper::Run()
{
	if (getParam<bool>("subscribe_cloud", true)){
	  
		cloudSub = n.subscribe(cloud_in, inputQueueSize, &ICPMapper::gotCloud, this);
// 		gps_position = n.subscribe("/rtk_gps/position",1, &ICPMapper::gotGPSPosition, this );
// 		gps_orientation = n.subscribe("/rtk_gps/orientation", 1, &ICPMapper::gotGPSOrientation, this);
// 		gps_fix = n.subscribe("/rtk_gps/gps",1,&ICPMapper::gotGPSFix, this);
		stateSub = n.subscribe("/Localizing_Mode",1, &ICPMapper::gotState, this);
// 		imuSub = n.subscribe(imu_in, 1, &ICPMapper::gotIMU, this);
	}

}

void ICPMapper::RunSlidingMap()
{
	if (getParam<bool>("subscribe_cloud", true)){
		cloudSub = n.subscribe(cloud_in, inputQueueSize, &ICPMapper::gotCloudSlidingMap, this);

	}

}


void ICPMapper::gotGPSPosition(const rtk_gps::Position& gpsPositionMsgIn)
{
	mappergps.altitude = gpsPositionMsgIn.height;
	mappergps.easting = gpsPositionMsgIn.easting;
	mappergps.northing = gpsPositionMsgIn.northing;
// 	mappergps.status = gpsPositionMsgIn.quality;
	mappergps.satellites_used = gpsPositionMsgIn.fixStatelliteNum;
	gps_position_time = gpsPositionMsgIn.header.stamp;
	
}

void ICPMapper::gotGPSOrientation(const rtk_gps::Orientation& gpsOrientationMsgIn)
{
	mappergps.pitch = gpsOrientationMsgIn.pitch;
	mappergps.roll = gpsOrientationMsgIn.roll;
	mappergps.yaw = gpsOrientationMsgIn.yaw;
	gps_orientation_time = gpsOrientationMsgIn.header.stamp;

}

void ICPMapper::gotGPSFix(const gps_common::GPSFix& gpsMsgIn)
{
	mappergps.latitude = gpsMsgIn.latitude;
	mappergps.longitude = gpsMsgIn.longitude;
	mappergps.altitude = gpsMsgIn.altitude;
// 	mappergps.easting = gpsMsgIn.err_horz;
// 	mappergps.northing = gpsMsgIn.err_vert;
	mappergps.pitch = gpsMsgIn.pitch;
	mappergps.roll = gpsMsgIn.roll;

	mappergps.timestamp = gpsMsgIn.header.stamp;
	mappergps.satellites_used = gpsMsgIn.status.satellites_used;
	
	gps_time = gpsMsgIn.header.stamp;
// 	cerr<<"------------------received gps "<<gps_time<<endl;

}


void ICPMapper::gotGPSFix(const gps_common::GPSFix& gpsMsgIn,double yaw)
{
	mappergps.latitude = gpsMsgIn.latitude;
	mappergps.longitude = gpsMsgIn.longitude;
	mappergps.altitude = gpsMsgIn.altitude;
// 	mappergps.easting = gpsMsgIn.err_horz;
// 	mappergps.northing = gpsMsgIn.err_vert;
	mappergps.pitch = gpsMsgIn.pitch;
	mappergps.roll = gpsMsgIn.roll;
	mappergps.yaw = yaw;

	mappergps.timestamp = gpsMsgIn.header.stamp;
	mappergps.satellites_used = gpsMsgIn.status.satellites_used;
	
	gps_time = gpsMsgIn.header.stamp;
// 	cerr<<"------------------received gps "<<gps_time<<endl;

}

void ICPMapper::gotGPSNavSatFix(const sensor_msgs::NavSatFix& gpsMsgIn)
{
	
	mappergps.latitude = gpsMsgIn.latitude;
	mappergps.longitude = gpsMsgIn.longitude;
	mappergps.altitude = gpsMsgIn.altitude;
	mappergps.easting = 0;
	mappergps.northing = 0;
	mappergps.timestamp = gpsMsgIn.header.stamp;
	mappergps.satellites_used = gpsMsgIn.status.status;
	
	gps_time = gpsMsgIn.header.stamp;
}


void ICPMapper::gotIMU(const sensor_msgs::ImuConstPtr& msg)
{
//   LOG(INFO)<< msg->angular_velocity;
      _imuMsgQueue.push(msg);

}




void ICPMapper::startSynchronize()
{
	cerr<<"start synchronize"<<endl;
	connection = synchronizer_.registerCallback(boost::bind( &ICPMapper::gotCloudandGPS,this,_1, _2, _3));  
}


 void ICPMapper::gotCloudandGPS(const sensor_msgs::PointCloud2ConstPtr& cloudMsgIn, const rtk_gps::PositionConstPtr& gpsMsgInP, const rtk_gps::OrientationConstPtr& gpsMsgInO)
{
	cerr<<"got cloud and GPS"<<endl;
	DP* cloud= new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*cloudMsgIn));
	GPS gps;
	gps.northing = gpsMsgInP->northing;
	gps.easting = gpsMsgInP->easting;
	gps.altitude = gpsMsgInP->height;
	gps.satellites_used = gpsMsgInP->fixStatelliteNum;
	gps.status = gpsMsgInP->quality;
	gps.yaw = gpsMsgInO->yaw;
	gps.roll = gpsMsgInO->pitch;
	gps.pitch = gpsMsgInO->pitch;
	processCloudandGPS(cloud, cloudMsgIn->header.frame_id, cloudMsgIn->header.stamp, cloudMsgIn->header.seq, gps);

}



void ICPMapper::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	if(localizing)
	{
		DP* cloud= new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));
		if(buse_imu){
// 		  LOG(INFO)<<"Process with laser and imu";
		  processCloud_imu(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
		  
		}
		else{
		  if(buse_odom){
// 		    LOG(INFO)<<"Process with laser and odom";
		    processCloud_odom(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
		  }
		  else{
// 		    cerr<<"ERROR INPUT"<<endl;
			  processCloud(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
		    return;
		  }
		 
		}
// 		delete cloud;
	}
}


void ICPMapper::gotCloudSlidingMap(const sensor_msgs::PointCloud2& cloudMsgIn)
{
	if(localizing)
	{
		
		DP* cloud= new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));
		if(buse_imu){
			processCloudSlidingMap_imu(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
		}else{
			processCloudSlidingMap(cloud, cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
		}
	}
}


void ICPMapper::processCloudandGPS(ICPMapper::DP* newPointCloud, const string& scannerFrame, const ros::Time& stamp, uint32_t seq, GPS gps)
{
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
	timer t;
	
	
	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec")))
	{
		//时间是进来的点云的stamp
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);//goodCount是进来点云的列数
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
		
	}

	inputFilters.apply(*newPointCloud);
			
	
	string reason;
	// Initialize the transformation to identity if empty
	//初始化TimuToMap和TLaserToLocalMap
 	if(!icp.hasMap())
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();//boost::mutex publishLock;
		TimuToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
		TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);
		TimuToMap(2,3) = mapElevation;
		publishLock.unlock();
	}

// 	PM::TransformationParameters TimuToLaser = PM::TransformationParameters::Identity(dimp1, dimp1);

	const PM::TransformationParameters TLaserToMap = TimuToMap * TimuToLaser.inverse();

	if(Debug_ICP)
	cout<<"TLaserToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TLaserToMap<<endl;

	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts. Less than "<<minReadingPointCount);
		return;
	}
// 	if(Debug_ICP)
	cout<<"[icp_mapper] After inputFilter are "<<newPointCloud->features.cols()<<" points"<<endl;

	if(newPointCloud->descriptorExists("probabilityStatic") == false)
	{
		newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
	}
	
	if(newPointCloud->descriptorExists("probabilityDynamic") == false)
	{
		newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
	}
	
	if(newPointCloud->descriptorExists("dynamic_ratio") == false)
	{
		newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
	}
	
	cerr<<"Current State "<<mCurrentState<<endl;
	if( mCurrentState==NOT_INITIALIZED )
 	{
		
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;//endScanTime

		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToMap,true);//需不需要给Frame new一个frame point，一个local world point
		pFrame->setGPS(gps);
		mpLocalMapper->InsertFrame(pFrame);
		mCurrentState = INITIALIZING;
		cerr<<"[icp]Initialing"<<endl;
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}else 
		if(mCurrentState == INITIALIZING )
		{
			if((!(mpLocalMapper->Initialized)))	//局部地图等待初始化局部点云和位姿
			{
	// 			if(!(mpLocalMapper->getLocalMapPointsNumber())){
					cerr<<"INITIALIZING, please don't move"<<endl;
					delete newPointCloud;
					return;
			}else
			{
				mCurrentState = WORKING;
			}
		}else
			if(mCurrentState == NOT_LOCALIZEING)
			{
				if(checkLocalizing == 0)
				{
					cerr<<"SET MAP!!"<<endl;
					mpLocalMapper->getLocalMapPointsNew(DPforLocalizing);
// 					setMap(&DPforLocalizing);		
					icpInit.setMap(DPforLocalizing);
// 					 firstICP = PM::TransformationParameters::Identity(4,4);
				}
//youwenti !!!!!  是不是要迭代
// 				PM::TransformationParameters firstICP = PM::TransformationParameters::Identity(4,4);
// 				PM::TransformationParameters Ticplocal;
				try{
					Ticplocal = icpInit(*newPointCloud, firstICP);
// 					cerr<<checkLocalizing<<"Localizing Ticp"<<endl<<Ticplocal<<endl;
// 					tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticplocal, mapFrame, laserFrame, stamp));
					firstICP = Ticplocal;
					if(checkLocalizing > 5)
					{
// 						cerr<<"OK"<<checkLocalizing<<endl;
						mCurrentState = LOCALIZEING;
						TLaserToLocalMap = Ticplocal;
						Frame* pFrame =  new Frame(stamp, newPointCloud,Ticplocal,true);
						pFrame->setGPS(gps);
						mpLocalMapper->InsertFrame(pFrame);
						StopProcessNewFrame();
						return;
					}
					delete newPointCloud;
					cerr<<"checkLocalizing"<<checkLocalizing++<<endl;
						
				}
				catch(PM::ConvergenceError error)
				{
					checkLocalizing = 0;
					 firstICP = PM::TransformationParameters::Identity(4,4);
					ROS_ERROR_STREAM("Localizeing ICP failed to converge: " << error.what());
					delete newPointCloud;
					
				}
				
				return;
			}
		
	
	
	if(!(isOkForProcessNewFrame()))
	{
		cerr<<"_______"<<endl<<"Not OK For Process NewFrame, last kFid is "<<lastProcessedKeyFrame<<endl;
		delete newPointCloud;
		return ;
	}

	PM::TransformationParameters copyTLocalToMap = mpLocalMapper->getLocalTLocalICP();

	DP copyReferenceDP;  
	mpLocalMapper->getLocalMapPointsNew(copyReferenceDP);
	ROS_INFO_STREAM("[Points] LocalMap has  " << copyReferenceDP.features.cols() << " Points");
// 	cerr<<"[icp_mapper]Local Map Points = "<<copyReferenceDP.features.cols()<<endl;
	setMap(&copyReferenceDP);
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 << " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp, TicpLocal,TScannerToLocal;
		TicpLocal = icp(*newPointCloud, TLaserToLocalMap);
		Ticp = copyTLocalToMap*TicpLocal ;	//for visualization
				

		if(Debug_ICP)
		{
			cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;
			cerr<<"[icp mapper]Ticp:\n" << Ticp<<endl;
		}
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		
// 		#ifdef DEBUG_ICP
// 		if(Debug_ICP)
		cerr<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
// 		#endif
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			return;
		}
		
		Frame* pFrame = new Frame(stamp, newPointCloud, TicpLocal,false);
		pFrame->setGPS(gps);
		if( ((estimatedOverlap<KFthreOverlap)&&(isOKforCreateNewKeyFrame()) && ((pFrame->mnId - lastProcessedKeyFrame )>30) && (copyReferenceDP.features.cols()>5000) )||
			((copyReferenceDP.features.cols()>MaxLocalMappoints)&&(isOKforCreateNewKeyFrame()) ) ||
			( (abs(TicpLocal(1,3)) > halfRoadWidth ) &&(isOKforCreateNewKeyFrame()) )
			|| ( (TicpLocal.col(3).head(2).norm() > maxLocalMapLength) && (isOKforCreateNewKeyFrame())))	//multi sension 的时候，这里是重新开始的，应该不需要设置值，Frame的Id重新开始好了
		{
			if(estimatedOverlap<KFthreOverlap)
			{
				recordKF<<pFrame->mnId<<"  "<<"Overlap"<<endl;
			}else if(copyReferenceDP.features.cols()>MaxLocalMappoints)
			{
				recordKF<<pFrame->mnId<<"  "<<"MaxLocalMappoints"<<endl;
			}else if(abs(TicpLocal(1,3)) > halfRoadWidth)
			{
				recordKF<<pFrame->mnId<<"  "<<"T y Over"<<endl;
			}else
			{
				recordKF<<pFrame->mnId<<"  "<<"Too Long"<<endl;
			}
// 			Frame* pFrame = new Frame(stamp, mCurrentDPInFrame, TicpLocal, mCurrentDPInLocalWorld,true);
			pFrame->SetKF();
// 			pFrame->setGPS(gps);
			TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);
			lastProcessedKeyFrame = pFrame->mnId;
			mpLocalMapper->InsertFrame(pFrame);
			StopProcessNewFrame();
// 			mpMap->mFramePose_Record.push_back(mTemp_FramePose_Record);
// 			mTemp_FramePose_Record.clear();
			mpMap->mFrame_Record.push_back(mTemp_Frame_Record);
			mTemp_Frame_Record.clear();
// 			cerr<<"new local map has "<<copyReferenceDP.features.cols()<<endl;

		}else
		{
			
			mpLocalMapper->InsertFrame(pFrame);
			TLaserToLocalMap = TicpLocal;
// 			mTemp_FramePose_Record.push_back(pFrame->mLocalTicp);
			mTemp_Frame_Record.push_back(pFrame);
		}
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TimuToMap = Ticp * TimuToLaser;
		// Publish tf
		if(publishMapTf == true)
		{
// 			cout<<Ticp<<endl;
			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
		}

		publishLock.unlock();
		
// 		ROS_DEBUG_STREAM("TimuToMap:\n" << TimuToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers()) {
          nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
          msg.child_frame_id = "velodyne";
		  odomPub.publish(msg);
                       }
	
// 		// Publish error on odometry
// 		if (odomErrorPub.getNumSubscribers())
// 			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TimuToMap, mapFrame, stamp));

	}
	catch (PM::ConvergenceError error)
// 	catch(exception &error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
	if(realTimeRatio < 80)
		ROS_INFO_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_WARN_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
}	





void ICPMapper::processCloud_imu(DP* newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{

 
	// IMPORTANT:  We need to receive the point clouds in local coordinates (scanner or robot)
    //shall we?
	//during static initialization, the robot would not move, so maybe its ok for us to leave alone these scans
	if(!mimu_preintegration->checkCoarseInitialized())
		return;
      
      
	timer t;
	
	//Change List: remove local mapping thread and do local mapping in the main function


	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts. Less than "<<minReadingPointCount);
		return;
	}
	
	PM::TransformationParameters iso3_preint_cur = PM::TransformationParameters::Identity(4,4);
	PM::TransformationParameters iso3_preint_last = PM::TransformationParameters::Identity(4,4);

    //what if we keep all of the IMU data?  
	bool matched_cur_imu = false, matched_last_imu = false;
	int curStampInIMUVector = -1, lastStampInIMUVector = -1;
	if(mCurrentState != WORKING){
		matched_last_imu = true;
		lastStampInIMUVector = -1; // perhaps the first laser frame
	}

	double curLaserTime = stamp.toSec();
	double lastLaserTime = lastPoinCloudTime.toSec();
	
// 	cout <<"curlaser time " << curLaserTime<<endl <<"last laser time " << lastLaserTime << endl;
	for(int j = mimu_preintegration->v_NavStates.size()-1; j >= 0; j--){
		NavState& pNavState  = mimu_preintegration->v_NavStates[j];
// 		cout <<"find navstate @ "<< setprecision(16) << pNavState.Get_Time() << endl;
		if(pNavState.Get_Time() > (curLaserTime + 0.01))	continue;
		
		//should be the imu state for laser frame
		if( (fabs(pNavState.Get_Time() - curLaserTime) < 0.01) && (!matched_cur_imu)){
			iso3_preint_cur.block<3,3>(0,0) = pNavState.Get_R().matrix().cast<float>();
			iso3_preint_cur.block<3,1>(0,3) = pNavState.Get_P().cast<float>();
			if(!matched_last_imu)
				iso3_preint_last = iso3_preint_cur; // in case of iso3_preint_last can not get a value when mCurrentState != WORKING
			matched_cur_imu = true;
			curStampInIMUVector = j;
			
		}else{
			if( ((pNavState.Get_Time() - curLaserTime) < -0.01) &&  (!matched_cur_imu)){
				iso3_preint_cur.block<3,3>(0,0) = pNavState.Get_R().matrix().cast<float>();
				iso3_preint_cur.block<3,1>(0,3) = pNavState.Get_P().cast<float>();
				if(!matched_last_imu)
					iso3_preint_last = iso3_preint_cur; // in case of iso3_preint_last can not get a value when mCurrentState != WORKING
				matched_cur_imu = true;
				curStampInIMUVector = j;
			}
		}
		
		if( (fabs(pNavState.Get_Time() - lastLaserTime) < 0.01) && (!matched_last_imu)){
			iso3_preint_last.block<3,3>(0,0) = pNavState.Get_R().matrix().cast<float>();
			iso3_preint_last.block<3,1>(0,3) = pNavState.Get_P().cast<float>();
			matched_last_imu = true;
			lastStampInIMUVector = j;
			if(previousFrame){
				previousFrame->setNavState(pNavState);
				previousFrame->setIMUId(lastStampInIMUVector);
			}
			break;
		}else{
			if( ((pNavState.Get_Time() - lastLaserTime) < -0.01) &&  (!matched_last_imu)){
				iso3_preint_last.block<3,3>(0,0) = pNavState.Get_R().matrix().cast<float>();
				iso3_preint_last.block<3,1>(0,3) = pNavState.Get_P().cast<float>();
				
				matched_last_imu = true;
				lastStampInIMUVector = j;
				if(previousFrame){
					previousFrame->setNavState(pNavState);
					previousFrame->setIMUId(lastStampInIMUVector);
				}
			}
		}
		
		if(matched_cur_imu && matched_last_imu)
			break;
		
		if(!(ros::ok()))
			break;
		
	}
	
	vector<vill::IMUData> imudata_in_last_frame;
	if(lastStampInIMUVector >=0 && curStampInIMUVector >= 0){
// 		cout << "[icp mapper] insert imu from "<< mimu_preintegration->lIMUdata[lastLaserTime]
		
		for(int j = lastStampInIMUVector; j < curStampInIMUVector; j++){
			imudata_in_last_frame.push_back(mimu_preintegration->lIMUdata[j]);
		}
// 		if(previousFrame){
// 			previousFrame->SetIMUdata(imudata_in_last_frame);
// 		}
	}else{
		cout <<"can not find matched imu state "<< matched_cur_imu <<" " << matched_last_imu << endl;
	}
	
// 	cout <<"[icp mapper] cur imu \n" << iso3_preint_cur << "\nlast  imu " << iso3_preint_last << endl;

	





	eMappingState _CurrentState ;
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  _CurrentState = mCurrentState;
	}
	
// 	vill::IMUPreintegrator imupre;
// 	vector<vill::IMUData> vimudatas;
// 	vector<vill::IMUData> vcombinedatas;
// 	bool imuInitialized = mpreintegration_optimizer->GetIMUInited();
	
// 	if( _CurrentState==NOT_INITIALIZED ){//the first laser frame
// 	  vimudatas = mimu_preintegration->getIMUdataVector();
// 	  mimu_preintegration->resetPreintegrator(); //TODO
// 	  deltaIMUdata.insert(deltaIMUdata.end(), vimudatas.begin(),vimudatas.end());
// 	  LOG(INFO)<<"reset"<<endl;
// 	}else{
// 	  //get imu data//TODO perhaps could be replaced
// 	  imupre = mimu_preintegration->getEvaluatedOdomPreintegration();
// 	  //get imu vector data, perhaps data is not synchronized
// 	  vimudatas = mimu_preintegration->getIMUdataVector();
// 	  mimu_preintegration->resetPreintegrator();
// 	  vcombinedatas.assign(deltaIMUdata.begin(),deltaIMUdata.end());
// 	  deltaIMUdata.clear();
// 	  while((!vimudatas.empty()) && (vimudatas.back()._t>=stamp.toSec())){
// 	    deltaIMUdata.insert(deltaIMUdata.begin(), vimudatas.back());
// 	    vimudatas.pop_back();
// 	  }
// 	  vcombinedatas.insert(vcombinedatas.end(),vimudatas.begin(),vimudatas.end());
// 
// 	}
	
	
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") 
		&& newPointCloud->descriptorExists("stamps_nsec")))
	{
		
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);//goodCount是进来点云的列数
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
		
	}

	inputFilters.apply(*newPointCloud);
	cout <<"after filtering " << newPointCloud->features.cols() <<" points " << endl;		
	
	string reason;
	// Initialize the transformation to gravity aligned coordinates
	//初始化TimuToMap和TLaserToLocalMap
 	if(mCurrentState != WORKING)
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();//boost::mutex publishLock;
// 		TimuToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
		TimuToMap = iso3_preint_cur;	//T^n_b0
		
// 		//TEST
		//间断跑（卡成小片段跑slam，ss为最后一帧（相对于上一个局部地图的那一帧），读进来的pose是上一个局部地图的pose）
// 		std::stringstream ss("2.41608 -1.08953 -0.0410187 0.955772 -0.00712788 -0.00232251 -0.294012");
// 		double x, y, z, qw, qx, qy, qz;
// 		ss >> x >> y >> z >> qw >> qx >> qy >> qz;
// 		Vector t(x, y, z);
// 		Quaternion q(qw, qx, qy, qz);
// 		TimuToMap.col(3).head(3) = t.cast<float>();
// 		TimuToMap.block(0,0,3,3) = q.toRotationMatrix().cast<float>();
// 		
// 		ifstream f("/home/turtle/catkin_ws/src/laser_mapping/KeyFrame2000-2339/30/GlobalPose.txt");
// 		PM::TransformationParameters TlastKeyframe = PM::TransformationParameters::Identity(4,4);
// 		for(int iii = 0; iii < 4; iii ++){
// 		  for(int jjj = 0;  jjj < 4; jjj++){
// 		    f >> TlastKeyframe(iii,jjj) ;
// 		  }
// 		}
// 		cout << "TlastKeyframe\n" << TlastKeyframe<<endl;
// 		cout <<"Tlastframe_to_lastKeyFrame\n" << TimuToMap<<endl;
// 		TimuToMap = TlastKeyframe * TimuToMap;
		
// 		TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);
		TLaserToMap = TimuToMap * TimuToLaser.inverse();
		cout <<"Initialize TLaserToMap\n"<< TLaserToMap << endl;
		TLaserToLocalMap = TLaserToMap;
		publishLock.unlock();
	}

// 	PM::TransformationParameters TimuToLaser = PM::TransformationParameters::Identity(dimp1, dimp1);

// 	TLaserToMap = TimuToMap * TimuToLaser.inverse();

	if(Debug_ICP)
		LOG(INFO)<<"TLaserToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TLaserToMap<<endl;


// 	if(Debug_ICP)
// 	LOG(INFO)<<"[icp_mapper] After inputFilter are "<<newPointCloud->features.cols()<<" points"<<endl;

	if(newPointCloud->descriptorExists("probabilityStatic") == false)
	{
		newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
	}
	
	if(newPointCloud->descriptorExists("probabilityDynamic") == false)
	{
		newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
	}
// 	newPointCloud->descriptors.rows()
	
	if(newPointCloud->descriptorExists("dynamic_ratio") == false)
	{
		newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
	}
	
// 	LOG(DEBUG)<<"Current State "<<mCurrentState<<endl;
	if( _CurrentState == NOT_INITIALIZED  || !reInitialized )
 	{
		
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;//endScanTime
// 		TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);

// 		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToLocalMap,true);//需不需要给Frame new一个frame point，一个local world point
		Frame* pFrame ;
		if(reInitialized)
			pFrame = new Frame(stamp, newPointCloud,TLaserToMap,true);//需不需要给Frame new一个frame point，一个local world point
		else
			pFrame = new Frame(stamp, newPointCloud,TLaserToMap,true);
// 		pFrame->SetIMUPreint(imupre);
		
// 		pFrame->UpdateNavStatePVRFromTwc(SE3d(TLaserToMap.cast<double>()), vill::ConfigParam::GetSE3Tbl());
		
		pFrame->SetTicp(TLaserToMap);
// 		mimu_preintegration->setCurLaserPose(TLaserToMap);
// 		pFrame->SetIMUdata(vcombinedatas);
// 		pFrame->SetNavStateBiasAcc(mimu_preintegration->getBiasa());
// 		pFrame->SetNavStateBiasGyr(mimu_preintegration->getBiasg());
		
		
// 		PM::TransformationParameters TNav = PM::TransformationParameters::Identity(4,4);
// 		NavState curNav = pFrame->GetNavState();
// 		TNav.block<3,3>(0,0) = curNav.Get_R().matrix().cast<float>();
// 		TNav.block<3,1>(0,3) = curNav.Get_P().cast<float>();
// 		tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TNav, mapFrame, "Nav", ros::Time::now()));
// 		  pFrame->ComputePreInt();
		
		{
		  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
		  previousFrame = pFrame;
		  pFrame->previousF = nullptr;
		}
// 		cout<<pFrame->mnId<<endl;
// 		cout<<"trackingimu pose: "<<pFrame->GetNavState().Get_P().transpose()<<endl
// 		<<"trackingimu velo: "<<pFrame->GetNavState().Get_V().transpose()<<endl;
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(fabs(stamp.sec - gps_time.sec) < 0.05){//此时获得的gps信号
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			LOG(INFO)<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		
		mpLocalMapper->InsertFrame(pFrame);
		mpreintegration_optimizer->insertFrame(pFrame);
		
		_CurrentState = INITIALIZING;
		if(!reInitialized)
			_CurrentState = WORKING;

		{
		  boost::mutex::scoped_lock lock(mMutexState);
		  mCurrentState = INITIALIZING;
		  if(!reInitialized)
			mCurrentState = WORKING;
		
		}
		
		if(!reInitialized)
			reInitialized = true;
		
		LOG(INFO)<<"[icp]Initialing"<<" "<<_CurrentState<<" "<<mCurrentState<<endl;
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}else 
		if(_CurrentState == INITIALIZING )
		{
			if((!(mpLocalMapper->Initialized)))	//局部地图等待初始化局部点云和位姿
			{
	// 			if(!(mpLocalMapper->getLocalMapPointsNumber())){
					LOG(INFO)<<"INITIALIZING, please don't move"<<endl;
					delete newPointCloud;
					return;
			}else
			{
			    {
			      boost::mutex::scoped_lock lock(mMutexState);
			      mCurrentState = WORKING;
			      _CurrentState = WORKING;
			    }
			    LOG(INFO)<<"WORKING";
			}
		}
		
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  mCurrentState = _CurrentState;
	}
	

	PM::TransformationParameters copyTLocalToMap = mpLocalMapper->getLocalTLocalICP();

	DP copyReferenceDP;  
	mpLocalMapper->getLocalMapPointsNew(copyReferenceDP);
	ROS_DEBUG_STREAM("[Points] LocalMap has  " << copyReferenceDP.features.cols() << " Points");
// 	cerr<<"[icp_mapper]Local Map Points = "<<copyReferenceDP.features.cols()<<endl;
	setMap(&copyReferenceDP);
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 << " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp, TicpLocal, TLaserToLocalMapPre;

/*		
		Eigen::Matrix3f BaseToMapRotation_pre =  imupre.getDeltaR().cast<float>();
		Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
		iso3_preint.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
		float deltat = stamp.toSec()-previousFrame->mstamp.toSec();
		iso3_preint.col(3).head(3) = imupre.getDeltaP().cast<float>() +( 0.5 * mimu_preintegration->getGravity() * (deltat * deltat)).cast<float>();
	    
	*/	
		PM::TransformationParameters deltaIMUPose = iso3_preint_last.inverse() * iso3_preint_cur;
		Eigen::Matrix3f BaseToMapRotation_pre = deltaIMUPose.block<3,3>(0,0);
		Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
		deltaIMUPose.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
		
		TLaserToLocalMapPre = TLaserToLocalMap * ( TimuToLaser * deltaIMUPose * TimuToLaser.inverse() );

		
		TicpLocal = icp(*newPointCloud, TLaserToLocalMapPre);

		Ticp = copyTLocalToMap*TicpLocal ;	//for visualization
				
		Eigen::Matrix3f BaseToMapRotation = Ticp.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngle(BaseToMapRotation);    // RotationMatrix to AxisAngle
		Ticp.block(0,0,3,3) = BaseToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix
		
		Eigen::Matrix3f BaseToMapRotationdelta = TicpLocal.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngledelta(BaseToMapRotationdelta);    // RotationMatrix to AxisAngle
		TicpLocal.block(0,0,3,3) = BaseToMapAxisAngledelta.toRotationMatrix();  // AxisAngle      to RotationMatrix

		if(Debug_ICP)
		{
			cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;

		}
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		
// 		#ifdef DEBUG_ICP
		if(Debug_ICP)
		LOG(INFO)<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
// 		#endif
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			//save preintegration //TODO
			return;
		}
		
		Frame* pFrame = new Frame(stamp, newPointCloud, TicpLocal,false);
		pFrame->SetIMUdata(imudata_in_last_frame);
// 		pFrame->SetInitialNavStateAndBias(previousFrame->GetNavState());
		if(curStampInIMUVector >= 0){
			pFrame->setNavState(mimu_preintegration->v_NavStates[curStampInIMUVector]);
			pFrame->setIMUId(curStampInIMUVector);
		}
// 		pFrame->SetIMUdata(vcombinedatas);
		SE3d se3Ticp = SE3d(Ticp.cast<double>());
		pFrame->SetTicp(Ticp);
// 		cout << "copyTLocalToMap\n"<<copyTLocalToMap << endl << TicpLocal<< endl;
// 		cout <<"Frame " << pFrame->mnId <<" with pose \n"<< pFrame->GetGlobalPose()<< endl;
// 		pFrame->SetIMUPreint(imupre);
		{
		  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
// 		  pFrame->SetInitialNavStateAndBias(previousFrame->GetNavState());
		  pFrame->previousF = previousFrame;
		  pFrame->previousF_KF = previousFrame;
		  previousFrame = pFrame;
		}
		
// 		if(mpreintegration_optimizer->checkBiasUpdated()){
// // 		  pFrame->SetNavStateDeltaBa(mpreintegration_optimizer->getdbiasa());//TEST  bias only be adjusted in optimization process?
// // 		  pFrame->SetNavStateDeltaBg(mpreintegration_optimizer->getdbiasg());
// // 		  pFrame->SetNavStateVel(mpreintegration_optimizer->getVelo());
// 		  mpreintegration_optimizer->setBiasUpdated(false);
// 		  pFrame->UpdateNavStatePVRFromTwc(se3Ticp, vill::ConfigParam::GetSE3Tbl());	//TEST
// // 		  pFrame->ComputePreInt();
// // 		  pFrame->UpdateNavState(pFrame->GetIMUPreint(),LIV::imu_preintegration::getGravity());	//TEST
// 		}
// // 		else{
// 		  pFrame->ComputePreInt();
// // 		  cout<<"trackingimu pose: "<<pFrame->GetNavState().Get_P().transpose()<<endl
// // 		  <<"trackingimu velo: "<<pFrame->GetNavState().Get_V().transpose()<<endl;
// // 		  cout<<"preint v: "<<pFrame->GetIMUPreint().getDeltaV().transpose()<<endl
// // 		  <<"preint p: "<<pFrame->GetIMUPreint().getDeltaP().transpose()<<endl;
// 		  pFrame->UpdateNavState(pFrame->GetIMUPreint(),LIV::imu_preintegration::getGravity());
// 		  cout<<"trackingimu pose: "<<pFrame->GetNavState().Get_P().transpose()<<endl
// 		  <<"trackingimu velo: "<<pFrame->GetNavState().Get_V().transpose()<<endl;
// 		}

		
		//save pose for initialization
// 		if( !mpLocalMapper->GetIMUInited())
		
// 		pFrame->UpdateNavStatePVRFromTcw(se3Ticp, vill::ConfigParam::GetSE3Tbl());
		

// 		mpreintegration_optimizer->insertFrameswithIMU(pFrame);

		
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(fabs(stamp.sec - gps_time.sec) < 0.05){
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			cerr<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		
		mpreintegration_optimizer->insertFrame(pFrame);
		mpreintegration_optimizer->processLaserFrames();
		
		if( ((estimatedOverlap<KFthreOverlap)&&(isOKforCreateNewKeyFrame()) && ((pFrame->mnId - lastProcessedKeyFrame )>20) && (copyReferenceDP.features.cols()>5000) )||
			((copyReferenceDP.features.cols()>MaxLocalMappoints)&&(isOKforCreateNewKeyFrame()) ) ||
			( (abs(TicpLocal(1,3)) > halfRoadWidth ) &&(isOKforCreateNewKeyFrame()) )
			|| ( (TicpLocal.col(3).head(2).norm() > maxLocalMapLength) && (isOKforCreateNewKeyFrame())
				|| ( (BaseToMapAxisAngledelta.angle()> maxAngle) && (isOKforCreateNewKeyFrame()) )
			) 
		|| ( (BaseToMapAxisAngledelta.angle()> 0.6) && (isOKforCreateNewKeyFrame()) ) )	//multi sension 的时候，这里是重新开始的，应该不需要设置值，Frame的Id重新开始好了
		{
			if(estimatedOverlap<KFthreOverlap)
			{
				recordKF<<pFrame->mnId<<"  "<<"Overlap"<<endl;
			}else if(copyReferenceDP.features.cols()>MaxLocalMappoints)
			{
				recordKF<<pFrame->mnId<<"  "<<"MaxLocalMappoints"<<endl;
			}else if(abs(TicpLocal(1,3)) > halfRoadWidth)
			{
				recordKF<<pFrame->mnId<<"  "<<"T y Over"<<endl;
			}else
			{
				recordKF<<pFrame->mnId<<"  "<<"Too Long"<<endl;
			}
// 			Frame* pFrame = new Frame(stamp, mCurrentDPInFrame, TicpLocal, mCurrentDPInLocalWorld,true);
			pFrame->SetKF();
			TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);
			lastProcessedKeyFrame = pFrame->mnId;
			mpLocalMapper->InsertFrame(pFrame);
			StopProcessNewFrame();
// 			mpMap->mFramePose_Record.push_back(mTemp_FramePose_Record);
// 			mTemp_FramePose_Record.clear();
			mpMap->mFrame_Record.push_back(mTemp_Frame_Record);
			mTemp_Frame_Record.clear();
// 			cerr<<"new local map has "<<copyReferenceDP.features.cols()<<endl;

		}else
		{
			
			mpLocalMapper->InsertFrame(pFrame);
			TLaserToLocalMap = TicpLocal;
			
// 			mTemp_FramePose_Record.push_back(pFrame->mLocalTicp);
			mTemp_Frame_Record.push_back(pFrame);
		}
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TimuToMap = Ticp * TimuToLaser;
		TLaserToMap = Ticp;
// 		mimu_preintegration->setCurLaserPose(TLaserToMap);
// 		// Publish tf
		
		//TEST 检查Nav的状态是否正确
// 		PM::TransformationParameters TNav = PM::TransformationParameters::Identity(4,4);
// 		NavState curNav = pFrame->GetNavState();
// 		TNav.block<3,3>(0,0) = curNav.Get_R().matrix().cast<float>();
// 		TNav.block<3,1>(0,3) = curNav.Get_P().cast<float>();
		if(publishMapTf == true)
		{
// 			cout<<Ticp<<endl;
			// tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
// 			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TNav, mapFrame, "Nav", ros::Time::now()));
		}

		publishLock.unlock();
		
// 		ROS_DEBUG_STREAM("TimuToMap:\n" << TimuToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers()) {
              nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
              msg.child_frame_id = "velodyne";
			  odomPub.publish(msg);
       }
	
// 		// Publish error on odometry
// 		if (odomErrorPub.getNumSubscribers())
// 			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TimuToMap, mapFrame, stamp));

	}
	catch (PM::ConvergenceError error)
// 	catch(exception &error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	LOG(INFO)<<"[TIME] Total ICP took: " << t.elapsed() << " [s]";
	if(realTimeRatio < 80)
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
}


void ICPMapper::processCloud_odom(ICPMapper::DP* newPointCloud, const string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{
      
	timer t;
	
	if(!(isOkForProcessNewFrame()))
	{
		cerr<<"_______"<<endl<<"Not OK For Process NewFrame, last kFid is "<<lastProcessedKeyFrame<<endl;
		delete newPointCloud;
		return ;
	}

	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts. Less than "<<minReadingPointCount);
		return;
	}
	
	PM::TransformationParameters iso3_preint = PM::TransformationParameters::Identity(4,4);


	eMappingState _CurrentState ;
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  _CurrentState = mCurrentState;
	  cout<<"Current State "<<_CurrentState<<" "<<mCurrentState<<endl;
	}
	

		
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec")))
	{
		//时间是进来的点云的stamp
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);//goodCount是进来点云的列数
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
		
	}

	inputFilters.apply(*newPointCloud);
			
	
	string reason;
	// Initialize the transformation to identity if empty
	//初始化TimuToMap和TLaserToLocalMap
 	if(!icp.hasMap())
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();//boost::mutex publishLock;
		TLaserToMap = TodomToMap * TodomToLaser.inverse();
		TLaserToLocalMap = TLaserToMap;
		publishLock.unlock();
	}

// 	PM::TransformationParameters TimuToLaser = PM::TransformationParameters::Identity(dimp1, dimp1);

// 	TLaserToMap = TimuToMap * TimuToLaser.inverse();

	if(Debug_ICP)
		LOG(INFO)<<"TLaserToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TLaserToMap<<endl;


// 	if(Debug_ICP)
// 	LOG(INFO)<<"[icp_mapper] After inputFilter are "<<newPointCloud->features.cols()<<" points"<<endl;

	if(newPointCloud->descriptorExists("probabilityStatic") == false)
	{
		newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
	}
	
	if(newPointCloud->descriptorExists("probabilityDynamic") == false)
	{
		newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
	}
// 	newPointCloud->descriptors.rows()
	
	if(newPointCloud->descriptorExists("dynamic_ratio") == false)
	{
		newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
	}
	
// 	LOG(DEBUG)<<"Current State "<<mCurrentState<<endl;
	if( _CurrentState == NOT_INITIALIZED )
 	{
		
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;//endScanTime
// 		TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);

// 		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToLocalMap,true);//需不需要给Frame new一个frame point，一个local world point
		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToMap,true);//需不需要给Frame new一个frame point，一个local world point
		
		{
		  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
		  previousFrame = pFrame;
		}
		
		previousOdom = modom_preintegration->getEvaluatedOdomPreintegration();


// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(stamp.sec == gps_time.sec){//此时获得的gps信号
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			LOG(INFO)<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		
		mpLocalMapper->InsertFrame(pFrame);
		
		_CurrentState = INITIALIZING;
		
		{
		  boost::mutex::scoped_lock lock(mMutexState);
		  mCurrentState = INITIALIZING;
		}
		
		LOG(INFO)<<"[icp]Initialing"<<" "<<_CurrentState<<" "<<mCurrentState<<endl;
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}else 
		if(_CurrentState == INITIALIZING )
		{
			if((!(mpLocalMapper->Initialized)))	//局部地图等待初始化局部点云和位姿
			{
	// 			if(!(mpLocalMapper->getLocalMapPointsNumber())){
					LOG(INFO)<<"INITIALIZING, please don't move"<<endl;
					delete newPointCloud;
					return;
			}else
			{
			    {
			      boost::mutex::scoped_lock lock(mMutexState);
			      mCurrentState = WORKING;
			    }
			    LOG(INFO)<<"WORKING";
			}
		}else
			if(_CurrentState == NOT_LOCALIZEING)// soppose the localizing will be called after slam
			{
			  mpMap->getWholeMapPoints(DPforLocalizing);
			  setMap(&DPforLocalizing);
			  {
			    boost::mutex::scoped_lock lock(mMutexState);
			    mCurrentState = LOCALIZEING;
			  }
			} 
			if(_CurrentState == LOCALIZEING){
			  
			  try 
			  {
			      // Apply ICP
			      PM::TransformationParameters Ticp, TicpLocal, TScannerToMapPre;     
			      
			      Eigen::Matrix3f BaseToMapRotation_pre = iso3_preint.block<3,3>(0,0);
			      Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
			      iso3_preint.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
			      
			      TScannerToMapPre = TLaserToMap * ( iso3_preint * TimuToLaser.inverse() );
			      
			      Ticp = icp(*newPointCloud, TScannerToMapPre);
							
			      Eigen::Matrix3f BaseToMapRotation = Ticp.block(0,0,3,3);
			      Eigen::AngleAxisf BaseToMapAxisAngle(BaseToMapRotation);    // RotationMatrix to AxisAngle
			      Ticp.block(0,0,3,3) = BaseToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix
					

				if(Debug_ICP)
				{
					cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;
		// 			cerr<<"[icp mapper]Ticp:\n" << Ticp<<endl;
				}
					
					// Ensure minimum overlap between scans
				const double estimatedOverlap = icp.errorMinimizer->getOverlap();
					
			// 		#ifdef DEBUG_ICP
				if(Debug_ICP){
				    LOG(INFO)<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
				}
			// 		#endif
				if (estimatedOverlap < minOverlap)
				{
					ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
					//save preintegration //TODO
					return;
				}
					
				Frame* pFrame = new Frame(stamp, newPointCloud, TicpLocal,false);

				SE3d se3Ticp = SE3d(Ticp.cast<double>());
				pFrame->SetTicp(Ticp);
			
			// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
				  if(stamp.sec == gps_time.sec){
					  mappergps.status = 1;
				  }else{
					  mappergps.status = 0;
					  cerr<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
				  }
				  pFrame->setGPS(mappergps);
			
				  
	  // 			mTemp_FramePose_Record.push_back(pFrame->mLocalTicp);
				  mTemp_Frame_Record.push_back(pFrame);
					
					
				  // Compute tf
				  publishStamp = stamp;
				  publishLock.lock();
				  TimuToMap = Ticp * TimuToLaser;
				  TLaserToMap = Ticp;
				  // Publish tf
		  		if(publishMapTf == true)
				  {
		  // 			cout<<Ticp<<endl;
					  tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
				  }

				  publishLock.unlock();
					
					// Publish odometry
				  if (odomPub.getNumSubscribers()) {
                      nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
                      msg.child_frame_id = "velodyne";
					  odomPub.publish(msg);
                                   }

				}
			  catch (PM::ConvergenceError error)
		  // 	catch(exception &error)
			  {
				  ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
				  return;
			  }
			  
				
			}
			
		
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  mCurrentState = _CurrentState;
	}
	

	PM::TransformationParameters copyTLocalToMap = mpLocalMapper->getLocalTLocalICP();

	DP copyReferenceDP;  
	mpLocalMapper->getLocalMapPointsNew(copyReferenceDP);
	ROS_DEBUG_STREAM("[Points] LocalMap has  " << copyReferenceDP.features.cols() << " Points");

	setMap(&copyReferenceDP);
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 << " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp, TicpLocal, TLaserToLocalMapPre;
		
		OdomPreintegrator curOdom = modom_preintegration->getEvaluatedOdomPreintegration();
		
		iso3_preint.block<3,3>(0,0) = curOdom.getDeltaR().cast<float>();
		iso3_preint.block<3,1>(0,3) = curOdom.getDeltaP().cast<float>();

		Eigen::Matrix3f BaseToMapRotation_pre = iso3_preint.block<3,3>(0,0);
		Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
		iso3_preint.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
		
		cout<<iso3_preint<<endl;
		
		TLaserToLocalMapPre = TLaserToLocalMap * ( iso3_preint * TodomToLaser.inverse() );
// 		TLaserToLocalMapPre = copyTLocalToMap.inverse() *  (TimuToLaser * iso3_preint * TimuToLaser.inverse() );
		
		TicpLocal = icp(*newPointCloud, TLaserToLocalMapPre);

		Ticp = copyTLocalToMap*TicpLocal ;	//for visualization
				
		Eigen::Matrix3f BaseToMapRotation = Ticp.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngle(BaseToMapRotation);    // RotationMatrix to AxisAngle
		Ticp.block(0,0,3,3) = BaseToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix
		
		Eigen::Matrix3f BaseToMapRotationdelta = TicpLocal.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngledelta(BaseToMapRotationdelta);    // RotationMatrix to AxisAngle
		TicpLocal.block(0,0,3,3) = BaseToMapAxisAngledelta.toRotationMatrix();  // AxisAngle      to RotationMatrix

		if(Debug_ICP)
		{
			cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;
// 			cerr<<"[icp mapper]Ticp:\n" << Ticp<<endl;
		}
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		
// 		#ifdef DEBUG_ICP
		if(Debug_ICP)
		LOG(INFO)<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
// 		#endif
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			//save preintegration //TODO
			return;
		}
		
		Frame* pFrame = new Frame(stamp, newPointCloud, TicpLocal,false);
		SE3d se3Ticp = SE3d(Ticp.cast<double>());
		pFrame->SetTicp(Ticp);

		
		
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(stamp.sec == gps_time.sec){
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			cerr<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		if( ((estimatedOverlap<KFthreOverlap)&&(isOKforCreateNewKeyFrame()) && ((pFrame->mnId - lastProcessedKeyFrame )>20) && (copyReferenceDP.features.cols()>5000) )||
			((copyReferenceDP.features.cols()>MaxLocalMappoints)&&(isOKforCreateNewKeyFrame()) ) ||
			( (abs(TicpLocal(1,3)) > halfRoadWidth ) &&(isOKforCreateNewKeyFrame()) )
			|| ( (TicpLocal.col(3).head(2).norm() > maxLocalMapLength) && (isOKforCreateNewKeyFrame()) ) 
		|| ( (BaseToMapAxisAngledelta.angle()> maxAngle) && (isOKforCreateNewKeyFrame()) ) )	//multi sension 的时候，这里是重新开始的，应该不需要设置值，Frame的Id重新开始好了
		{
			if(estimatedOverlap<KFthreOverlap)
			{
				recordKF<<pFrame->mnId<<"  "<<"Overlap"<<endl;
			}else if(copyReferenceDP.features.cols()>MaxLocalMappoints)
			{
				recordKF<<pFrame->mnId<<"  "<<"MaxLocalMappoints"<<endl;
			}else if(abs(TicpLocal(1,3)) > halfRoadWidth)
			{
				recordKF<<pFrame->mnId<<"  "<<"T y Over"<<endl;
			}else
			{
				recordKF<<pFrame->mnId<<"  "<<"Too Long"<<endl;
			}
// 			Frame* pFrame = new Frame(stamp, mCurrentDPInFrame, TicpLocal, mCurrentDPInLocalWorld,true);
			pFrame->SetKF();
			TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);
			lastProcessedKeyFrame = pFrame->mnId;
			mpLocalMapper->InsertFrame(pFrame);
			StopProcessNewFrame();
// 			mpMap->mFramePose_Record.push_back(mTemp_FramePose_Record);
// 			mTemp_FramePose_Record.clear();
			mpMap->mFrame_Record.push_back(mTemp_Frame_Record);
			mTemp_Frame_Record.clear();
// 			cerr<<"new local map has "<<copyReferenceDP.features.cols()<<endl;

		}else
		{
			
			mpLocalMapper->InsertFrame(pFrame);
			TLaserToLocalMap = TicpLocal;
			
// 			mTemp_FramePose_Record.push_back(pFrame->mLocalTicp);
			mTemp_Frame_Record.push_back(pFrame);
		}
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TodomToMap = Ticp * TodomToLaser;
		TLaserToMap = Ticp;
		// Publish tf
		if(publishMapTf == true)
		{
// 			cout<<Ticp<<endl;
			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
		}

		publishLock.unlock();
		
// 		ROS_DEBUG_STREAM("TimuToMap:\n" << TimuToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers()) {
              nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
              msg.child_frame_id = "velodyne";
			  odomPub.publish(msg);
       }	
	
// 		// Publish error on odometry
// 		if (odomErrorPub.getNumSubscribers())
// 			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TimuToMap, mapFrame, stamp));

	}
	catch (PM::ConvergenceError error)
// 	catch(exception &error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	LOG(INFO)<<"[TIME] Total ICP took: " << t.elapsed() << " [s]";
	if(realTimeRatio < 80)
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
}

void ICPMapper::processCloud(DP* newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq){
      
	timer t;
	
// 	if(!(isOkForProcessNewFrame()))
// 	{
// 		cerr<<"_______"<<endl<<"Not OK For Process NewFrame, last kFid is "<<lastProcessedKeyFrame<<endl;
// 		delete newPointCloud;
// 		return ;
// 	}

	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts. Less than "<<minReadingPointCount);
		return;
	}
	
	PM::TransformationParameters iso3_preint = PM::TransformationParameters::Identity(4,4);


	eMappingState _CurrentState ;
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  _CurrentState = mCurrentState;
	 // cout<<"Current State "<<_CurrentState<<" "<<mCurrentState<<endl;
	}
	

		
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec")))
	{
		//时间是进来的点云的stamp
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);//goodCount是进来点云的列数
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
		
	}

	inputFilters.apply(*newPointCloud);
			
	
	string reason;
	// Initialize the transformation to identity if empty
	//初始化TimuToMap和TLaserToLocalMap
 	if(mCurrentState != WORKING)
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();//boost::mutex publishLock;
		TLaserToMap = TodomToMap * TodomToLaser.inverse();
		TLaserToLocalMap = TLaserToMap;
		publishLock.unlock();
	}

// 	PM::TransformationParameters TimuToLaser = PM::TransformationParameters::Identity(dimp1, dimp1);

// 	TLaserToMap = TimuToMap * TimuToLaser.inverse();

	if(Debug_ICP)
		LOG(INFO)<<"TLaserToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TLaserToMap<<endl;


// 	if(Debug_ICP)
// 	LOG(INFO)<<"[icp_mapper] After inputFilter are "<<newPointCloud->features.cols()<<" points"<<endl;

	if(newPointCloud->descriptorExists("probabilityStatic") == false)
	{
		newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
	}
	
	if(newPointCloud->descriptorExists("probabilityDynamic") == false)
	{
		newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
	}
// 	newPointCloud->descriptors.rows()
	
	if(newPointCloud->descriptorExists("dynamic_ratio") == false)
	{
		newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
	}
	
// 	LOG(DEBUG)<<"Current State "<<mCurrentState<<endl;
	if( _CurrentState == NOT_INITIALIZED )
 	{
		
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;//endScanTime
// 		TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);

// 		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToLocalMap,true);//需不需要给Frame new一个frame point，一个local world point
		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToMap,true);//需不需要给Frame new一个frame point，一个local world point
		
		{
		  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
		  previousFrame = pFrame;
		}
		
// 		previousOdom = modom_preintegration->getEvaluatedOdomPreintegration();


// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(fabs(stamp.sec - gps_time.sec) < 0.05){//此时获得的gps信号
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			LOG(INFO)<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		
		mpLocalMapper->InsertFrame(pFrame);
		
		_CurrentState = INITIALIZING;
		
		{
		  boost::mutex::scoped_lock lock(mMutexState);
		  mCurrentState = INITIALIZING;
		}
		
		LOG(INFO)<<"[icp]Initialing"<<" "<<_CurrentState<<" "<<mCurrentState<<endl;
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}else 
		if(_CurrentState == INITIALIZING )
		{
			if((!(mpLocalMapper->Initialized)))	//局部地图等待初始化局部点云和位姿
			{
	// 			if(!(mpLocalMapper->getLocalMapPointsNumber())){
					LOG(INFO)<<"INITIALIZING, please don't move"<<endl;
					delete newPointCloud;
					return;
			}else
			{
			    {
			      boost::mutex::scoped_lock lock(mMutexState);
			      mCurrentState = WORKING;
			      _CurrentState = WORKING;
			    }
			    LOG(INFO)<<"WORKING";
			}
		}else
			if(_CurrentState == NOT_LOCALIZEING)// soppose the localizing will be called after slam
			{
			  mpMap->getWholeMapPoints(DPforLocalizing);
			  setMap(&DPforLocalizing);
			  {
			    boost::mutex::scoped_lock lock(mMutexState);
			    mCurrentState = LOCALIZEING;
			  }
			} 
			if(_CurrentState == LOCALIZEING){
			  
			  try 
			  {
			      // Apply ICP
			      PM::TransformationParameters Ticp, TicpLocal, TScannerToMapPre;     
			      
			      Eigen::Matrix3f BaseToMapRotation_pre = iso3_preint.block<3,3>(0,0);
			      Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
			      iso3_preint.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
			      
			      TScannerToMapPre = TLaserToMap * ( iso3_preint * TimuToLaser.inverse() );
			      
			      Ticp = icp(*newPointCloud, TScannerToMapPre);
							
			      Eigen::Matrix3f BaseToMapRotation = Ticp.block(0,0,3,3);
			      Eigen::AngleAxisf BaseToMapAxisAngle(BaseToMapRotation);    // RotationMatrix to AxisAngle
			      Ticp.block(0,0,3,3) = BaseToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix
					

				if(Debug_ICP)
				{
					cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;
		// 			cerr<<"[icp mapper]Ticp:\n" << Ticp<<endl;
				}
					
					// Ensure minimum overlap between scans
				const double estimatedOverlap = icp.errorMinimizer->getOverlap();
					
			// 		#ifdef DEBUG_ICP
				if(Debug_ICP){
				    LOG(INFO)<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
				}
			// 		#endif
				if (estimatedOverlap < minOverlap)
				{
					ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
					//save preintegration //TODO
					return;
				}
					
				Frame* pFrame = new Frame(stamp, newPointCloud, TicpLocal,false);

				SE3d se3Ticp = SE3d(Ticp.cast<double>());
				pFrame->SetTicp(Ticp);
			
			// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
				  if(fabs(stamp.sec - gps_time.sec) < 0.03){
					  mappergps.status = 1;
				  }else{
					  mappergps.status = 0;
					  cerr<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
				  }
				  pFrame->setGPS(mappergps);
			
				  
	  // 			mTemp_FramePose_Record.push_back(pFrame->mLocalTicp);
				  mTemp_Frame_Record.push_back(pFrame);
					
					
				  // Compute tf
				  publishStamp = stamp;
				  publishLock.lock();
				  TimuToMap = Ticp * TimuToLaser;
				  TLaserToMap = Ticp;
				  // Publish tf
		  		if(publishMapTf == true)
				  {
		  // 			cout<<Ticp<<endl;
					  tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
				  }

				  publishLock.unlock();
					
					// Publish odometry
				 if (odomPub.getNumSubscribers()) {
                      nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
                      msg.child_frame_id = "velodyne";
					  odomPub.publish(msg);
                                   }

				}
			  catch (PM::ConvergenceError error)
		  // 	catch(exception &error)
			  {
				  ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
				  return;
			  }
			  
				
			}
			
		
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  mCurrentState = _CurrentState;
	}
	

	PM::TransformationParameters copyTLocalToMap = mpLocalMapper->getLocalTLocalICP();

	DP copyReferenceDP;  
	mpLocalMapper->getLocalMapPointsNew(copyReferenceDP);
	ROS_DEBUG_STREAM("[Points] LocalMap has  " << copyReferenceDP.features.cols() << " Points");

	setMap(&copyReferenceDP);
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 << " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp, TicpLocal, TLaserToLocalMapPre;
		
// 		OdomPreintegrator curOdom = modom_preintegration->getEvaluatedOdomPreintegration();
// 		
// 		iso3_preint.block<3,3>(0,0) = curOdom.getDeltaR().cast<float>();
// 		iso3_preint.block<3,1>(0,3) = curOdom.getDeltaP().cast<float>();
// 
// 		Eigen::Matrix3f BaseToMapRotation_pre = iso3_preint.block<3,3>(0,0);
// 		Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
// 		iso3_preint.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
// 		
// 		cout<<iso3_preint<<endl;
		
		TLaserToLocalMapPre = TLaserToLocalMap ;
// 		TLaserToLocalMapPre = copyTLocalToMap.inverse() *  (TimuToLaser * iso3_preint * TimuToLaser.inverse() );
		cout << "new pointcloud " << newPointCloud->features.cols() << endl;
		cout << "map pointcloud " << copyReferenceDP.features.cols() << endl;
		TicpLocal = icp(*newPointCloud, TLaserToLocalMapPre);

		Ticp = copyTLocalToMap*TicpLocal ;	//for visualization
				
		Eigen::Matrix3f BaseToMapRotation = Ticp.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngle(BaseToMapRotation);    // RotationMatrix to AxisAngle
		Ticp.block(0,0,3,3) = BaseToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix
		
		Eigen::Matrix3f BaseToMapRotationdelta = TicpLocal.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngledelta(BaseToMapRotationdelta);    // RotationMatrix to AxisAngle
		TicpLocal.block(0,0,3,3) = BaseToMapAxisAngledelta.toRotationMatrix();  // AxisAngle      to RotationMatrix

		if(Debug_ICP)
		{
			cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;
// 			cerr<<"[icp mapper]Ticp:\n" << Ticp<<endl;
		}
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		
// 		#ifdef DEBUG_ICP
		if(Debug_ICP)
		LOG(INFO)<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
// 		#endif
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			//save preintegration //TODO
			return;
		}
		
		Frame* pFrame = new Frame(stamp, newPointCloud, TicpLocal,false);
		SE3d se3Ticp = SE3d(Ticp.cast<double>());
		pFrame->SetTicp(Ticp);

		
		
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(fabs(stamp.sec - gps_time.sec) < 0.03){
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			cerr<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		if( ((estimatedOverlap<KFthreOverlap)&&(isOKforCreateNewKeyFrame()) && ((pFrame->mnId - lastProcessedKeyFrame )>20) && (copyReferenceDP.features.cols()>5000) )||
			((copyReferenceDP.features.cols()>MaxLocalMappoints)&&(isOKforCreateNewKeyFrame()) ) ||
			( (abs(TicpLocal(1,3)) > halfRoadWidth ) &&(isOKforCreateNewKeyFrame()) )
			|| ( (TicpLocal.col(3).head(2).norm() > maxLocalMapLength) && (isOKforCreateNewKeyFrame()) ) 
		|| ( (BaseToMapAxisAngledelta.angle()> maxAngle) && (isOKforCreateNewKeyFrame()) ) )	//multi sension 的时候，这里是重新开始的，应该不需要设置值，Frame的Id重新开始好了
		{
			if(estimatedOverlap<KFthreOverlap)
			{
				recordKF<<pFrame->mnId<<"  "<<"Overlap"<<endl;
			}else if(copyReferenceDP.features.cols()>MaxLocalMappoints)
			{
				recordKF<<pFrame->mnId<<"  "<<"MaxLocalMappoints"<<endl;
			}else if(abs(TicpLocal(1,3)) > halfRoadWidth)
			{
				recordKF<<pFrame->mnId<<"  "<<"T y Over"<<endl;
			}else if((TicpLocal.col(3).head(2).norm() > maxLocalMapLength))
			{
				recordKF<<pFrame->mnId<<"  "<<"Too Long"<<endl;
			}else if((BaseToMapAxisAngledelta.angle()> maxAngle)){
				recordKF<<pFrame->mnId<<"  "<<"angle too large"<<endl;
			}else{
				recordKF<<pFrame->mnId<<"  "<<"???"<<endl;
			}
// 			Frame* pFrame = new Frame(stamp, mCurrentDPInFrame, TicpLocal, mCurrentDPInLocalWorld,true);
			pFrame->SetKF();
			TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);
			lastProcessedKeyFrame = pFrame->mnId;
			mpLocalMapper->InsertFrame(pFrame);
			StopProcessNewFrame();
// 			mpMap->mFramePose_Record.push_back(mTemp_FramePose_Record);
// 			mTemp_FramePose_Record.clear();
			mpMap->mFrame_Record.push_back(mTemp_Frame_Record);
			mTemp_Frame_Record.clear();
// 			cerr<<"new local map has "<<copyReferenceDP.features.cols()<<endl;

		}else
		{
			
			mpLocalMapper->InsertFrame(pFrame);
			TLaserToLocalMap = TicpLocal;
			
// 			mTemp_FramePose_Record.push_back(pFrame->mLocalTicp);
			mTemp_Frame_Record.push_back(pFrame);
		}
		
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TodomToMap = Ticp * TodomToLaser;
		TLaserToMap = Ticp;
		// Publish tf
		if(publishMapTf == true)
		{
// 			cout<<Ticp<<endl;
			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
		}

		publishLock.unlock();
		
// 		ROS_DEBUG_STREAM("TimuToMap:\n" << TimuToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers()) {
                      nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
                      msg.child_frame_id = "velodyne";
					  odomPub.publish(msg);
                                   }
	
// 		// Publish error on odometry
// 		if (odomErrorPub.getNumSubscribers())
// 			odomErrorPub.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(TimuToMap, mapFrame, stamp));

	}
	catch (PM::ConvergenceError error)
// 	catch(exception &error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	LOG(INFO)<<"[TIME] Total ICP took: " << t.elapsed() << " [s]";
	if(realTimeRatio < 80)
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;

}


void ICPMapper::processCloudSlidingMap_imu(DP* newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{

      if(!mimu_preintegration->checkCoarseInitialized())
	return;
      
      
	timer t;
	
	if(!(isOkForProcessNewFrame()))
	{
		cerr<<"_______"<<endl<<"Not OK For Process NewFrame, last kFid is "<<lastProcessedKeyFrame<<endl;
		delete newPointCloud;
		return ;
	}

	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts. Less than "<<minReadingPointCount);
		return;
	}
	
	PM::TransformationParameters iso3_preint = PM::TransformationParameters::Identity(4,4);
	// try{
	// 	iso3_preint = PointMatcher_ros::eigenMatrixToDim<float>(
	// 			PointMatcher_ros::transformListenerToEigenMatrix<float>(
	// 			tfListener,
	// 			laserFrame,
	// 			imuFrame,
	// 			ros::Time::now()
	// 		),4);
	  
	// }catch(tf2::ExtrapolationException){
	//   LOG(INFO)<<"synchronization wrong..."<<endl;
	//   return;
	// }

      
    list<double>::iterator time_it = mimu_preintegration->vT_times.begin();

	while(ros::ok()){
		if(fabs(*time_it - stamp.toSec()) < 0.003){
			iso3_preint = mimu_preintegration->vT_laser_imu.front();
			break;
		}else{
			if(*time_it < stamp.toSec()){
				time_it++;
				mimu_preintegration->vT_times.pop_front();
				mimu_preintegration->vT_laser_imu.pop_front();
			}else{
				ROS_ERROR_STREAM("synchronization wrong " << stamp << "-" << *time_it);
				cout << *time_it - stamp.toSec() << endl;
				mpLocalMapper->setProcessedOneFrame(true);
				return;
			}
		}
	}


	eMappingState _CurrentState ;
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  _CurrentState = mCurrentState;
	  cout<<"Current State "<<_CurrentState<<" "<<mCurrentState<<endl;
	}
	
	vill::IMUPreintegrator imupre;
	vector<vill::IMUData> vimudatas;
	vector<vill::IMUData> vcombinedatas;
// 	bool imuInitialized = mpreintegration_optimizer->GetIMUInited();
	
	if( _CurrentState==NOT_INITIALIZED ){//the first laser frame
	  vimudatas = mimu_preintegration->getIMUdataVector();
	  mimu_preintegration->resetPreintegrator();
	  deltaIMUdata.assign(vimudatas.begin(),vimudatas.end());
	  LOG(INFO)<<"reset"<<endl;
	}else{
	  //get imu data//TODO perhaps could be replaced
	  imupre = mimu_preintegration->getEvaluatedOdomPreintegration();
	  //get imu vector data, perhaps data is not synchronized
	  vimudatas = mimu_preintegration->getIMUdataVector();
	  mimu_preintegration->resetPreintegrator();
	  vcombinedatas.assign(deltaIMUdata.begin(),deltaIMUdata.end());
	  deltaIMUdata.clear();
	  while((!vimudatas.empty()) && (vimudatas.back()._t>=stamp.toSec())){
	    deltaIMUdata.insert(deltaIMUdata.begin(), vimudatas.back());
	    vimudatas.pop_back();
	  }
	  vcombinedatas.insert(vcombinedatas.end(),vimudatas.begin(),vimudatas.end());

	}
	
	
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec")))
	{
		//时间是进来的点云的stamp
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);//goodCount是进来点云的列数
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
		
	}

	inputFilters.apply(*newPointCloud);
			
	
	string reason;
	// Initialize the transformation to identity if empty
	//初始化TimuToMap和TLaserToLocalMap
 	if(!icp.hasMap())
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();//boost::mutex publishLock;
// 		TimuToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
		TimuToMap = mimu_preintegration->getInitialT();	//T^n_b0

		
		
// 		TimuToMap(2,3) = mapElevation;
		TLaserToMap = TimuToMap * TimuToLaser.inverse();
		TLaserToLocalMap = TLaserToMap;
		publishLock.unlock();
	}



	if(Debug_ICP)
		LOG(INFO)<<"TLaserToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TLaserToMap<<endl;



	if(newPointCloud->descriptorExists("probabilityStatic") == false)
	{
		newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
	}
	
	if(newPointCloud->descriptorExists("probabilityDynamic") == false)
	{
		newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
	}
// 	newPointCloud->descriptors.rows()
	
	if(newPointCloud->descriptorExists("dynamic_ratio") == false)
	{
		newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
	}
	
// 	LOG(DEBUG)<<"Current State "<<mCurrentState<<endl;
	if( _CurrentState == NOT_INITIALIZED )
 	{
		
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;//endScanTime
// 		TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);

// 		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToLocalMap,true);//需不需要给Frame new一个frame point，一个local world point
		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToMap,true);//需不需要给Frame new一个frame point，一个local world point
// 		pFrame->SetIMUPreint(imupre);
		
		pFrame->UpdateNavStatePVRFromTwc(SE3d(TLaserToMap.cast<double>()), vill::ConfigParam::GetSE3Tbl());
		
		pFrame->SetTicp(TLaserToMap);
		mimu_preintegration->setCurLaserPose(TLaserToMap);
		pFrame->SetIMUdata(vcombinedatas);
		pFrame->SetNavStateBiasAcc(mimu_preintegration->getBiasa());
		pFrame->SetNavStateBiasGyr(mimu_preintegration->getBiasg());
// 		  pFrame->ComputePreInt();
		
		{
		  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
		  previousFrame = pFrame;
		}
		cout<<pFrame->mnId<<endl;
		cout<<"trackingimu pose: "<<pFrame->GetNavState().Get_P().transpose()<<endl
		<<"trackingimu velo: "<<pFrame->GetNavState().Get_V().transpose()<<endl;
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(stamp.sec == gps_time.sec){//此时获得的gps信号
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			LOG(INFO)<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		
		mpLocalMapper->InsertFrame(pFrame);
		
		_CurrentState = INITIALIZING;
		
		{
		  boost::mutex::scoped_lock lock(mMutexState);
		  mCurrentState = INITIALIZING;
		}
		
		LOG(INFO)<<"[icp]Initialing"<<" "<<_CurrentState<<" "<<mCurrentState<<endl;
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}else 
		if(_CurrentState == INITIALIZING )
		{
			if((!(mpLocalMapper->Initialized)))	//局部地图等待初始化局部点云和位姿
			{
	// 			if(!(mpLocalMapper->getLocalMapPointsNumber())){
					LOG(INFO)<<"INITIALIZING, please don't move"<<endl;
					delete newPointCloud;
					return;
			}else
			{
			    {
			      boost::mutex::scoped_lock lock(mMutexState);
			      mCurrentState = WORKING;
			    }
			    LOG(INFO)<<"WORKING";
			}
		}
		
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  mCurrentState = _CurrentState;
	}
	

// 	PM::TransformationParameters copyTLocalToMap = mpLocalMapper->getLocalTLocalICP();

	DP copyReferenceDP;  
	mpLocalMapper->getLocalMapPointsNew(copyReferenceDP);
	ROS_DEBUG_STREAM("[Points] LocalMap has  " << copyReferenceDP.features.cols() << " Points");
// 	cerr<<"[icp_mapper]Local Map Points = "<<copyReferenceDP.features.cols()<<endl;
	setMap(&copyReferenceDP);
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 
		<< " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp, TicpLocal, TLaserToLocalMapPre;

		Eigen::Matrix3f BaseToMapRotation_pre = iso3_preint.block<3,3>(0,0);
		Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
		iso3_preint.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
		
		TLaserToLocalMapPre = TLaserToMap * ( iso3_preint * TimuToLaser.inverse() );
// 		TLaserToLocalMapPre = copyTLocalToMap.inverse() *  (TimuToLaser * iso3_preint * TimuToLaser.inverse() );
		
		Ticp = icp(*newPointCloud, TLaserToLocalMapPre);

				
		Eigen::Matrix3f BaseToMapRotation = Ticp.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngle(BaseToMapRotation);    // RotationMatrix to AxisAngle
		Ticp.block(0,0,3,3) = BaseToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix
		

		if(Debug_ICP)
		{
			cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;
// 			cerr<<"[icp mapper]Ticp:\n" << Ticp<<endl;
		}
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		
// 		#ifdef DEBUG_ICP
		if(Debug_ICP)
		LOG(INFO)<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
// 		#endif
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			//save preintegration //TODO
			return;
		}
		
		Frame* pFrame = new Frame(stamp, newPointCloud, Ticp,false);
		pFrame->SetIMUdata(vcombinedatas);
		SE3d se3Ticp = SE3d(Ticp.cast<double>());
		pFrame->SetTicp(Ticp);
// 		pFrame->SetIMUPreint(imupre);
		{
		  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
		  pFrame->SetInitialNavStateAndBias(previousFrame->GetNavState());
		  pFrame->previousF = previousFrame;
		  pFrame->previousF_KF = previousFrame;
		  previousFrame = pFrame;
		}
		
		if(mpreintegration_optimizer->checkBiasUpdated()){
// 		  pFrame->SetNavStateDeltaBa(mpreintegration_optimizer->getdbiasa());//TEST  bias only be adjusted in optimization process?
// 		  pFrame->SetNavStateDeltaBg(mpreintegration_optimizer->getdbiasg());
// 		  pFrame->SetNavStateVel(mpreintegration_optimizer->getVelo());
		  mpreintegration_optimizer->setBiasUpdated(false);
		  pFrame->UpdateNavStatePVRFromTwc(se3Ticp, vill::ConfigParam::GetSE3Tbl());	//TEST
// 		  pFrame->ComputePreInt();
// 		  pFrame->UpdateNavState(pFrame->GetIMUPreint(),LIV::imu_preintegration::getGravity());	//TEST
		}
// 		else{
		  pFrame->ComputePreInt();
// 		  cout<<"trackingimu pose: "<<pFrame->GetNavState().Get_P().transpose()<<endl
// 		  <<"trackingimu velo: "<<pFrame->GetNavState().Get_V().transpose()<<endl;
// 		  cout<<"preint v: "<<pFrame->GetIMUPreint().getDeltaV().transpose()<<endl
// 		  <<"preint p: "<<pFrame->GetIMUPreint().getDeltaP().transpose()<<endl;
		  pFrame->UpdateNavState(pFrame->GetIMUPreint(),LIV::imu_preintegration::getGravity());
// 		  cout<<"trackingimu pose: "<<pFrame->GetNavState().Get_P().transpose()<<endl
// 		  <<"trackingimu velo: "<<pFrame->GetNavState().Get_V().transpose()<<endl;
// 		}

		
		//save pose for initialization
// 		if( !mpLocalMapper->GetIMUInited())
		
// 		pFrame->UpdateNavStatePVRFromTcw(se3Ticp, vill::ConfigParam::GetSE3Tbl());
		

		mimu_preintegration->setCurLaserPose(TLaserToMap);
		mpreintegration_optimizer->insertFrameswithIMU(pFrame);

		
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(stamp.sec == gps_time.sec){
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			cerr<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);

			
		mpLocalMapper->InsertFrame(pFrame);
// 		TLaserToLocalMap = TicpLocal;

// 		mTemp_Frame_Record.push_back(pFrame);
		
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TimuToMap = Ticp * TimuToLaser;
		TLaserToMap = Ticp;
		// Publish tf
		
		PM::TransformationParameters TNav = PM::TransformationParameters::Identity(4,4);
		NavState curNav = pFrame->GetNavState();
		TNav.block<3,3>(0,0) = curNav.Get_R().matrix().cast<float>();
		TNav.block<3,1>(0,3) = curNav.Get_P().cast<float>();
		
		
		if(publishMapTf == true)
		{
// 			cout<<Ticp<<endl;
			// tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TNav, mapFrame, "Nav", ros::Time::now()));
		  
		}

		publishLock.unlock();
		
// 		ROS_DEBUG_STREAM("TimuToMap:\n" << TimuToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers()) {
                      nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
                      msg.child_frame_id = "velodyne";
					  odomPub.publish(msg);
                                   }
	
	}
	catch (PM::ConvergenceError error)
// 	catch(exception &error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	LOG(INFO)<<"[TIME] Total ICP took: " << t.elapsed() << " [s]";
	if(realTimeRatio < 80)
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
}


void ICPMapper::processCloudSlidingMap(ICPMapper::DP* newPointCloud, const string& scannerFrame, const ros::Time& stamp, uint32_t seq)
{
      
	timer t;
	
	if(!(isOkForProcessNewFrame()))
	{
		cerr<<"_______"<<endl<<"Not OK For Process NewFrame, last kFid is "<<lastProcessedKeyFrame<<endl;
		delete newPointCloud;
		return ;
	}

	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)
	{
		ROS_ERROR("I found no good points in the cloud");
		return;
	}
	
	const int ptsCount = newPointCloud->features.cols();
	if(ptsCount < minReadingPointCount)
	{
		ROS_ERROR_STREAM("Not enough points in newPointCloud: only " << ptsCount << " pts. Less than "<<minReadingPointCount);
		return;
	}
	
	PM::TransformationParameters iso3_preint = PM::TransformationParameters::Identity(4,4);


	eMappingState _CurrentState ;
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  _CurrentState = mCurrentState;
	  cout<<"Current State "<<_CurrentState<<" "<<mCurrentState<<endl;
	}
	
	
	// Dimension of the point cloud, important since we handle 2D and 3D
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec")))
	{
		//时间是进来的点云的stamp
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);//goodCount是进来点云的列数
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);
		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
		
	}

	inputFilters.apply(*newPointCloud);
			
	
	string reason;
	// Initialize the transformation to identity if empty
	//初始化TimuToMap和TLaserToLocalMap
 	if(!icp.hasMap())
 	{
		// we need to know the dimensionality of the point cloud to initialize properly
		publishLock.lock();//boost::mutex publishLock;
		TimuToMap = PM::TransformationParameters::Identity(dimp1, dimp1);
// 		TimuToMap = mimu_preintegration->getInitialT();	//T^n_b0

		
		
// 		TimuToMap(2,3) = mapElevation;
		TLaserToMap = TimuToMap * TimuToLaser.inverse();
		TLaserToLocalMap = TLaserToMap;
		publishLock.unlock();
	}



	if(Debug_ICP)
		LOG(INFO)<<"TLaserToMap (" << scannerFrame << " to " << mapFrame << "):\n" << TLaserToMap<<endl;



	if(newPointCloud->descriptorExists("probabilityStatic") == false)
	{
		newPointCloud->addDescriptor("probabilityStatic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorStatic));
	}
	
	if(newPointCloud->descriptorExists("probabilityDynamic") == false)
	{
		newPointCloud->addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newPointCloud->features.cols(), priorDyn));
	}
// 	newPointCloud->descriptors.rows()
	
	if(newPointCloud->descriptorExists("dynamic_ratio") == false)
	{
		newPointCloud->addDescriptor("dynamic_ratio", PM::Matrix::Zero(1, newPointCloud->features.cols()));
	}
	
// 	LOG(DEBUG)<<"Current State "<<mCurrentState<<endl;
	if( _CurrentState == NOT_INITIALIZED )
 	{
		
		ROS_INFO_STREAM("Creating an initial map");
		mapCreationTime = stamp;//endScanTime
// 		TLaserToLocalMap = PM::TransformationParameters::Identity(dimp1, dimp1);

// 		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToLocalMap,true);//需不需要给Frame new一个frame point，一个local world point
		Frame* pFrame =  new Frame(stamp, newPointCloud,TLaserToMap,true);//需不需要给Frame new一个frame point，一个local world point
// 		pFrame->SetIMUPreint(imupre);
		
// 		pFrame->UpdateNavStatePVRFromTwc(SE3d(TLaserToMap.cast<double>()), vill::ConfigParam::GetSE3Tbl());
		
		pFrame->SetTicp(TLaserToMap);
// 		mimu_preintegration->setCurLaserPose(TLaserToMap);
// 		pFrame->SetIMUdata(vcombinedatas);
// 		pFrame->SetNavStateBiasAcc(mimu_preintegration->getBiasa());
// 		pFrame->SetNavStateBiasGyr(mimu_preintegration->getBiasg());
// 		  pFrame->ComputePreInt();
		
		{
		  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
		  previousFrame = pFrame;
		}
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(stamp.sec == gps_time.sec){//此时获得的gps信号
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			LOG(INFO)<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);
		
		mpLocalMapper->InsertFrame(pFrame);
		
		_CurrentState = INITIALIZING;
		
		{
		  boost::mutex::scoped_lock lock(mMutexState);
		  mCurrentState = INITIALIZING;
		}
		
		LOG(INFO)<<"[icp]Initialing"<<" "<<_CurrentState<<" "<<mCurrentState<<endl;
		// we must not delete newPointCloud because we just stored it in the mapPointCloud
		return;
	}else 
		if(_CurrentState == INITIALIZING )
		{
			if((!(mpLocalMapper->Initialized)))	//局部地图等待初始化局部点云和位姿
			{
	// 			if(!(mpLocalMapper->getLocalMapPointsNumber())){
					LOG(INFO)<<"INITIALIZING, please don't move"<<endl;
					delete newPointCloud;
					return;
			}else
			{
			    {
			      boost::mutex::scoped_lock lock(mMutexState);
			      mCurrentState = WORKING;
			    }
			    LOG(INFO)<<"WORKING";
			}
		}
		
	{
	  boost::mutex::scoped_lock lock(mMutexState);
	  mCurrentState = _CurrentState;
	}
	

// 	PM::TransformationParameters copyTLocalToMap = mpLocalMapper->getLocalTLocalICP();

	DP copyReferenceDP;  
	mpLocalMapper->getLocalMapPointsNew(copyReferenceDP);
	ROS_DEBUG_STREAM("[Points] LocalMap has  " << copyReferenceDP.features.cols() << " Points");
// 	cerr<<"[icp_mapper]Local Map Points = "<<copyReferenceDP.features.cols()<<endl;
	setMap(&copyReferenceDP);
	
	// Check dimension
	if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
	{
		ROS_ERROR_STREAM("Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1 
		<< " while map is " << icp.getInternalMap().features.rows()-1);
		return;
	}
	
	try 
	{
		// Apply ICP
		PM::TransformationParameters Ticp, TicpLocal, TLaserToLocalMapPre;

// 		Eigen::Matrix3f BaseToMapRotation_pre = iso3_preint.block<3,3>(0,0);
// 		Eigen::AngleAxisf BaseToMapAxisAngle_pre(BaseToMapRotation_pre); 
// 		iso3_preint.block(0,0,3,3) = BaseToMapAxisAngle_pre.toRotationMatrix();
		
		TLaserToLocalMapPre = TLaserToMap /** ( iso3_preint * TimuToLaser.inverse() )*/;
// 		TLaserToLocalMapPre = copyTLocalToMap.inverse() *  (TimuToLaser * iso3_preint * TimuToLaser.inverse() );
		
		Ticp = icp(*newPointCloud, TLaserToLocalMapPre);

				
		Eigen::Matrix3f BaseToMapRotation = Ticp.block(0,0,3,3);
		Eigen::AngleAxisf BaseToMapAxisAngle(BaseToMapRotation);    // RotationMatrix to AxisAngle
		Ticp.block(0,0,3,3) = BaseToMapAxisAngle.toRotationMatrix();  // AxisAngle      to RotationMatrix
		

		if(Debug_ICP)
		{
			cerr<<"[after icp] mappoint "<<icp.getInternalMap().features.cols()<<endl;
// 			cerr<<"[icp mapper]Ticp:\n" << Ticp<<endl;
		}
		
		// Ensure minimum overlap between scans
		const double estimatedOverlap = icp.errorMinimizer->getOverlap();
		
// 		#ifdef DEBUG_ICP
		if(Debug_ICP)
		LOG(INFO)<<"[icp mapper] Overlap: " << estimatedOverlap<<endl;
// 		#endif
		if (estimatedOverlap < minOverlap)
		{
			ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction!");
			//save preintegration //TODO
			return;
		}
		
		Frame* pFrame = new Frame(stamp, newPointCloud, Ticp,false);
		SE3d se3Ticp = SE3d(Ticp.cast<double>());
		pFrame->SetTicp(Ticp);

		{
		  previousFrame = pFrame;
		}
		



		
		//save pose for initialization
// 		if( !mpLocalMapper->GetIMUInited())
		
// 		pFrame->UpdateNavStatePVRFromTcw(se3Ticp, vill::ConfigParam::GetSE3Tbl());
		

		
// 		if((stamp.sec == gps_position_time.sec) && (stamp.sec == gps_orientation_time.sec)){
		if(stamp.sec == gps_time.sec){
			mappergps.status = 1;
		}else{
			mappergps.status = 0;
			cerr<<"frame "<<pFrame->mnId<<"  in time "<<pFrame->mstamp<<"does not have gps"<<endl;
		}
		pFrame->setGPS(mappergps);

			
		mpLocalMapper->InsertFrame(pFrame);
// 		TLaserToLocalMap = TicpLocal;

// 		mTemp_Frame_Record.push_back(pFrame);
		
		
		// Compute tf
		publishStamp = stamp;
		publishLock.lock();
		TimuToMap = Ticp * TimuToLaser;
		TLaserToMap = Ticp;
		// Publish tf
		
		
		if(publishMapTf == true)
		{
// 			cout<<Ticp<<endl;
			tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(Ticp, mapFrame, laserFrame, stamp));
		  
		}

		publishLock.unlock();
		
// 		ROS_DEBUG_STREAM("TimuToMap:\n" << TimuToMap);

		// Publish odometry
		if (odomPub.getNumSubscribers()) {
                      nav_msgs::Odometry msg = PointMatcher_ros::eigenMatrixToOdomMsg<float>(Ticp, mapFrame, ros::Time::now());
                      msg.child_frame_id = "velodyne";
					  odomPub.publish(msg);
                                   }
	
	}
	catch (PM::ConvergenceError error)
// 	catch(exception &error)
	{
		ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
		return;
	}
	
	//Statistics about time and real-time capability
	int realTimeRatio = 100*t.elapsed() / (stamp.toSec()-lastPoinCloudTime.toSec());
	LOG(INFO)<<"[TIME] Total ICP took: " << t.elapsed() << " [s]";
	if(realTimeRatio < 80)
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");
	else
		ROS_DEBUG_STREAM("[TIME] Real-time capability: " << realTimeRatio << "%");

	lastPoinCloudTime = stamp;
}


void ICPMapper::processNewMapIfAvailable()
{
	#if BOOST_VERSION >= 104100
	if (mapBuildingInProgress && mapBuildingFuture.has_value())
	{
		ROS_INFO_STREAM("[processNewMapIfAvailable] New map available");
		setMap(mapBuildingFuture.get());
		mapBuildingInProgress = false;
	}
	#endif // BOOST_VERSION >= 104100
}

void ICPMapper::setMap(DP* newPointCloud)
{
	// delete old map
// 	#ifdef DEBUG_ICP
	if(Debug_ICP)
	cerr<<"[icp mapper]SetMap"<<endl;
// 	#endif
	if (mapPointCloud)
		delete mapPointCloud;
	
	
	// set new map
// 	mapPointCloud = newPointCloud;
	mapPointCloud = new DP(newPointCloud->features,newPointCloud->featureLabels
			,newPointCloud->descriptors, newPointCloud->descriptorLabels/*,
			newPointCloud->times, newPointCloud->timeLabels*/);
// 	#ifdef DEBUG_ICP
	if(Debug_ICP)
	cerr << "[setMap] copying map to ICP"<<mapPointCloud->features.cols() << endl;
// #endif
	icp.setMap(*mapPointCloud);
	
// 	#ifdef DEBUG_ICP
	if(Debug_ICP)
	cerr << "[setMap] end of set map" << endl;
// #endif
	// Publish map point cloud
	// FIXME this crash when used without descriptor
// 	if (mapPub.getNumSubscribers())
// 		mapPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, mapFrame, mapCreationTime));
}

void ICPMapper::SetLocalMapper(Velodyne_SLAM::LocalMapping* pLocalMapper)
{
	mpLocalMapper = pLocalMapper;
}

bool ICPMapper::isOKforCreateNewKeyFrame()
{
	boost::mutex::scoped_lock lock(mMutexCreateNewKeyFrame);
	return OKForCreateNewKeyFrame;
}

void ICPMapper::ResetStopCreateNewKeyFrame()
{
	boost::mutex::scoped_lock lock(mMutexCreateNewKeyFrame);
	OKForCreateNewKeyFrame = true;
}
void ICPMapper::SetStopCreateNewKeyFrame()
{
	boost::mutex::scoped_lock lock(mMutexCreateNewKeyFrame);
	OKForCreateNewKeyFrame = false;

}

void ICPMapper::OkForProcessNewFrame()
{
	boost::mutex::scoped_lock lock(mMutexCreateNewFrame);
	OKProcessNewFrame = true;
}

void ICPMapper::StopProcessNewFrame()
{
	boost::mutex::scoped_lock lock(mMutexCreateNewFrame);
	OKProcessNewFrame = false;
}

bool ICPMapper::isOkForProcessNewFrame()
{
	boost::mutex::scoped_lock lock(mMutexCreateNewFrame);
	return OKProcessNewFrame;
}

bool ICPMapper::loadPosetxt(string posetxt, PointMatcher< float >::TransformationParameters& pose)
{
	ifstream fT;
	fT.open(posetxt);
	for(int i = 0;i < 4; i++)
	{
		double a[4];
		fT >> a[0]>>a[1]>> a[2]>>a[3];
		pose(i,0) = a[0];	pose(i,1) = a[1];	pose(i,2) = a[2];	pose(i,3) = a[3];
	}
	return true;
}

void ICPMapper::setImuPre(LIV::imu_preintegration* pimuPre)
{
  mimu_preintegration = pimuPre;

}

void ICPMapper::setOdoPre(LIV::odom_preintegration* podoPre)
{
  modom_preintegration = podoPre;
}

void ICPMapper::setPreintegrationOpt(LIV::preintegration_opt* ppreintegration_opt)
{
  mpreintegration_optimizer = ppreintegration_opt;
}


int ICPMapper::getPreviousFrameId()
{
  boost::mutex::scoped_lock lock(mMutexPreviousFrame);
  if(previousFrame)
	  return previousFrame->mnId;
  else
	  return -1;

}

void ICPMapper::gotState(const laser_mapping::Localizing_Mode& msg)
{
//   boost::mutex::scoped_lock lock(mMutexState);
//   if(msg.localization_mode == 1)
//     mCurrentState = NOT_LOCALIZEING;
//   else{
//     mCurrentState = NOT_INITIALIZED;
//   }
  ros::shutdown();

}


bool ICPMapper::initializationforReSLAM()
{
	
	
	PM::TransformationParameters currentRefT = PM::TransformationParameters::Identity(4,4);
	try{
	  currentRefT = PointMatcher_ros::eigenMatrixToDim<float>(
			  PointMatcher_ros::transformListenerToEigenMatrix<float>(
			  tfListener,
			  "/reference",
			  laserFrame,
			  ros::Time::now()
		  ),4);
	    
	  }catch(tf2::TransformException){
	    LOG(INFO)<<"please sending transformation..."<<endl;
	    return false;  
	}
	
	while(!(mpLocalMapper->initializationforReSLAM()) && ros::ok()){};

	PM::TransformationParameters currentRefKFT = mpLocalMapper->getLocalTLocalICP(); 
	TodomToMap = currentRefKFT * currentRefT * TodomToLaser;
	TLaserToMap = currentRefKFT * currentRefT;
	TLaserToLocalMap = currentRefT;
	mCurrentState = WORKING;
	
	reInitialized = false;
	
	cout<<TodomToMap<<endl<<TLaserToMap<<endl<<TLaserToLocalMap<<endl;
	
}

void ICPMapper::stop()
{
	mpMap->mFrame_Record.push_back(mTemp_Frame_Record);
}



}
