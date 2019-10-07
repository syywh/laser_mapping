#include "Map.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <boost/pending/property.hpp>
#include <boost/graph/graph_concepts.hpp>
#include "glog/logging.h"

namespace Velodyne_SLAM {
Map::Map():transformation(PM::get().REG(Transformation).create("RigidTransformation"))
{
	max_KF_ID = 0;
	mnMaxKFid = 0;
	mbMapUpdated= false;
	mWholeMapPoints = nullptr;
	wholePosePublisher = nh.advertise<visualization_msgs::MarkerArray>("wholePoses", 2, true);
// 	int time = (int)ros::Time::now().sec;
// 	stringstream s;
// 	s<<time;
// 	filename = ros::package::getPath("laser_slam")+"/Log/map"+s.str();+".txt";
// 	fMap.open(filename);
	
	string configFileName;
	configFileName =ros::package::getPath("laser_mapping")+ "/cfg/map_post_filters_forVIsual.yaml";
		ifstream ifs(configFileName.c_str());
		if (ifs.good())
		{
			mapPostFilters_Visual=PM::DataPointsFilters(ifs);;//ICPChainBase function
// 			fMap<<"Load Map Post Filter for Visual OK"<<endl;
		}
		ifs.close();
	configFileName =ros::package::getPath("laser_mapping")+ "/cfg/map_post_filters.yaml";
		 ifs.open(configFileName.c_str());
		if (ifs.good())
		{
			mapPostFilters=PM::DataPointsFilters(ifs);;//ICPChainBase function
// 			fMap<<"Load Map Post Filter for Visual OK"<<endl;
		}
	beginID = 0;
	mCurrentKF = nullptr;
	
	string name;
	name = "staticKept";
	if (ros::param::get(std::string("~")+name, static_th_for_keeping))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << static_th_for_keeping);
	}
	else{
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << static_th_for_keeping);
		static_th_for_keeping = 0.8;
		
	}
	
}

void Map::AddKeyFrame(KeyFrame* pKeyFrame)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	mspKeyFrames.push_back(pKeyFrame);
	cout << "map add keyframe "<<pKeyFrame->mnId << endl;
	if(!mCurrentKF)
	{
		mCurrentKF = pKeyFrame;
		if((mCurrentKF->mLocalMapPoints)){
			
			AddMapPoints();
		}
		else
			cerr<<"KeyFrame"<<pKeyFrame->mnId<<" dosen't have DP"<<endl;
		return;
	}
		
	if((mCurrentKF->mLocalMapPoints)){
		
		AddMapPoints();
	}
	else
		cerr<<"KeyFrame"<<pKeyFrame->mnId<<" dosen't have DP"<<endl;
	mCurrentKF = pKeyFrame;
	if(pKeyFrame->mnId>mnMaxKFid)
		mnMaxKFid=pKeyFrame->mnId;
// 	fMap<<pKeyFrame->mnId<<endl<<pKeyFrame->getPose()<<endl;
// 	mbMapUpdated=true;//update the map when the mappoints is concatenated


}

void Map::AddKeyFrameWithoutAddMapPoints(KeyFrame* pKeyFrame)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	mspKeyFrames.push_back(pKeyFrame);
	
	if(!mCurrentKF)
	{
		mCurrentKF = pKeyFrame;
		return;
	}
	
	if(pKeyFrame->mnId>mnMaxKFid)
		mnMaxKFid=pKeyFrame->mnId;

}



void Map::AddMapPoints()
{
// 	mspMapPoints.insert(pMapPoints);
	cerr<<"ADD MAP POINTS"<<endl;
	DP* temp = mCurrentKF->mLocalMapPoints;
	if((mspKeyFrames.size()==1) || (!mWholeMapPoints))
	{
		if(mWholeMapPoints)
			delete mWholeMapPoints;
		mWholeMapPoints = new DP(temp->features,temp->featureLabels
			,temp->descriptors, temp->descriptorLabels/*,
			temp->times, temp->timeLabels*/);
		DP transformedDP = transformation->compute(*mWholeMapPoints, mCurrentKF->getPose());
		int cnt = 0;
		int staticStarting = transformedDP.getDescriptorStartingRow("probabilityStatic");
		
		for(int i = 0; i < transformedDP.features.cols(); i++){
			if(transformedDP.descriptors(staticStarting, i) > static_th_for_keeping){
				mWholeMapPoints->setColFrom(cnt,transformedDP, i);
				cnt++;
			}
		}
		mWholeMapPoints->conservativeResize(cnt);
	}else{

		DP t = transformation->compute(*temp, mCurrentKF->getPose());
		int cnt = 0;
		int staticStarting = t.getDescriptorStartingRow("probabilityStatic");
		DP t_static = t.createSimilarEmpty();
		for(int i = 0; i < t.features.cols(); i++){
			if(t.descriptors(staticStarting, i) > static_th_for_keeping){
				t_static.setColFrom(cnt,t, i);
				cnt++;
			}
		}
		t_static.conservativeResize(cnt);
		mWholeMapPoints->concatenate(t_static);
		
	//	delete t;
	}
// 	mapPostFilters.apply(*mWholeMapPoints);
	mbMapUpdated = true;
	

}

void Map::clear()
{

    for(vector<Frame*>::iterator sit=mspFrames.begin(), send=mspFrames.end(); sit!=send; sit++)
        delete *sit;


    mspFrames.clear();
    mnMaxKFid = 0;

}

bool Map::isMapUpdated()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	return mbMapUpdated;
}

void Map::ResetUpdated()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	mbMapUpdated = false;
}


void Map::getWholeMapPoints(Map::DP& tDP)
{
	boost::mutex::scoped_lock lock(mMutexMap);
// 	tDP = DP(mWholeMapPoints->features,mWholeMapPoints->featureLabels
// 			,mWholeMapPoints->descriptors, mWholeMapPoints->descriptorLabels,
// 			mWholeMapPoints->times, mWholeMapPoints->timeLabels);
	tDP = *mWholeMapPoints;
}

void Map::filterMapPoints()
{
	mapPostFilters_Visual.apply(*mWholeMapPoints);
}


KeyFrame* Map::getLatestKeyFrame()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	return (mspKeyFrames.back());
	
}

int Map::getKeyFrameNum()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	return (mspKeyFrames.size());
}

void Map::getAllKeyFrames(std::vector< KeyFrame* >& pv)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	pv.clear();
	pv = mspKeyFrames;
// 	pv = mspKeyFrames.assign();
}

void Map::updateMap(std::vector< KeyFrame* > pv)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	KeyFrame* tempkf = pv[beginID];
	DP* temp = tempkf->mLocalMapPoints;

// 	cerr<<"0"<<endl;
	if(mWholeMapPoints)
		delete mWholeMapPoints;
	mWholeMapPoints = new DP(temp->features,temp->featureLabels
		,temp->descriptors, temp->descriptorLabels);
	
	
	DP transformedDP = transformation->compute(*mWholeMapPoints, tempkf->getPose());
	int cnt = 0;
	int staticStarting = transformedDP.getDescriptorStartingRow("probabilityStatic");
	for(int i = 0; i < transformedDP.features.cols(); i++){
			if(transformedDP.descriptors(staticStarting, i) > static_th_for_keeping){
				mWholeMapPoints->setColFrom(cnt,transformedDP, i);
				cnt++;
			}
		}
	mWholeMapPoints->conservativeResize(cnt);
	
	for(size_t tt =beginID+1;tt<(pv.size()-1); tt++)
	{
		temp = pv[tt]->mLocalMapPoints;
		DP* t = new DP(temp->features,temp->featureLabels
			,temp->descriptors, temp->descriptorLabels/*,
			temp->times, temp->timeLabels*/);
		*t = transformation->compute(*t, pv[tt]->getPose());
		
		int cnt = 0;
		int staticStarting = t->getDescriptorStartingRow("probabilityStatic");
		DP t_static = t->createSimilarEmpty();
		
		for(int i = 0; i < t->features.cols(); i++){
			if(t->descriptors(staticStarting, i) > static_th_for_keeping){
				t_static.setColFrom(cnt,*t, i);
				cnt++;
			}
		}
		t_static.conservativeResize(cnt);
		mWholeMapPoints->concatenate(t_static);
		delete t;	
	}
// 	cerr<<"2"<<endl;
// 	mapPostFilters.apply(*mWholeMapPoints);
	mbMapUpdated = true;
}

KeyFrame* Map::getKeyFrame(long unsigned int id)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	if(id < mspKeyFrames.size())
		return mspKeyFrames[(int)id];
	else{
		cout <<"out of map bound"<< endl;
		return nullptr;
	}

}


void Map::saveMap(bool saveDP)
{
  //在程序结束的时候调用
  ofstream fPosetxt;
  cerr<<"beginID "<<beginID<<" allKeyFrames "<<mspKeyFrames.size()<<endl;
      for(size_t i = beginID; i<mspKeyFrames.size(); i++)
      {
	      stringstream temps;
	      string tempstring;
	      if(!(ros::param::get("~FramesSavingPath",tempstring)))
	         tempstring = ros::package::getPath("laser_mapping")+"/KeyFrame/";
	      temps<<"mkdir "<<tempstring<<mspKeyFrames[i]->mnId;
	      try{
		    system(temps.str().c_str());
	      }catch(runtime_error &e)
	      {
		  cerr<<"All Ready Exit! "<<e.what()<<endl;
	      }
	      if(saveDP){
		stringstream tempDPs;
		cerr<<mspKeyFrames[i]->mnId<<endl;
		tempDPs<<tempstring<<mspKeyFrames[i]->mnId<<"/DataPoints.vtk";
		mspKeyFrames[i]->mLocalMapPoints->save(tempDPs.str());
	      }
	      
	      cerr<<"KeyFrame "<<mspKeyFrames[i]->mnId<<endl;
	      stringstream poses;
	      poses<<tempstring<<mspKeyFrames[i]->mnId<<"/GlobalPose.txt";
	      fPosetxt.open(poses.str());
	      fPosetxt<<mspKeyFrames[i]->getPose()<<endl;
	      fPosetxt.close();
	      
	      stringstream gpsxml;
	      gpsxml<<tempstring<<mspKeyFrames[i]->mnId<<"/gps.xml";
	      boost::property_tree::ptree pgps;
	      GPS gps = mspKeyFrames[i]->gps;

	      pgps.put("gps.altitude", gps.altitude);
	      pgps.put("gps.northing", gps.northing);
	      pgps.put("gps.easting", gps.easting);
	      pgps.put("gps.roll", gps.roll);
	      pgps.put("gps.pitch", gps.pitch);
	      pgps.put("gps.yaw", gps.yaw);
	      pgps.put("gps.timestamp", gps.timestamp);
		  pgps.put("gps.latitude", gps.latitude);
		  pgps.put("gps.longitude", gps.longitude);
	      pgps.put("gps.status.status", gps.status);
	      pgps.put("gps.status.satellites_used",gps.satellites_used);
	      
	       boost::property_tree::xml_writer_settings<string> setting(' ', 2);
	       boost::property_tree::write_xml(gpsxml.str(), pgps, std::locale(), setting);
	      
	      
	      //再调用一下saveEdge
      }
      
      DP wholeMP;
	  getWholeMapPoints(wholeMP);
	  string tempstring;
	      if(!(ros::param::get("~FramesSavingPath",tempstring)))
	         tempstring = ros::package::getPath("laser_mapping")+"/KeyFrame/";
	  wholeMP.save(tempstring+"/wholeMap.vtk");

}

void Map::setBeginID(int num)
{
	beginID = num;
}

void Map::saveWholeDP(string name){
	mWholeMapPoints->save(name.c_str());
}

void Map::save(const std::string& filename)
// void Velodyne_SLAM::Map::save
{
	cerr<<"saving map begin ID  "<<beginID<<endl;
	cerr<<"all keyframes "<<mspKeyFrames.size()<<endl;
    boost::property_tree::ptree p_map;
    p_map.put("map.property.node_count", mspKeyFrames.size());

    boost::property_tree::ptree p_node_list;
	int maxKF = mspKeyFrames.size()-1;
	KeyFrame* endkf = mspKeyFrames[maxKF];
	bool circle = false;
    for (auto node_it = (mspKeyFrames.begin()); node_it != mspKeyFrames.end(); ++node_it)
    {
      boost::property_tree::ptree p_node;
      p_node.put("id", (*node_it)->mnId);
      p_node.put("inMap", true);	//TEST
//       p_node.put("id", (*node_it)->mnFrameId);	//TEST frame 改进版
//       p_node.put("is_Keyframe", true);	//TEST 
      boost::property_tree::ptree p_neighbour_list;
       map<long unsigned int, bool> tempReachable = (*node_it)->neighbours_isReachable;
	  
	  cout << (*node_it)->mnId <<"-";
      for (auto neighbour_it = (*node_it)->neighbours.begin(); neighbour_it != (*node_it)->neighbours.end(); ++neighbour_it)
      {
		boost::property_tree::ptree p_neighbour;
// 		p_neighbour.put("id", mspKeyFrames[neighbour_it->first]->mnFrameId);//TEST 
		p_neighbour.put("id", neighbour_it->first);
		cout << neighbour_it->first <<"-";
	//         Transform T(neighbour_it->second.cast<double>());
		PM::TransformationParameters pose_matrix = neighbour_it->second;
	// 	Eigen::Isometry3d iso3(pose_matrix.cast<double>());
		Eigen::Isometry3d iso3;
		iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
		iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
		Eigen::Isometry3d T(iso3);
// 		Transform Tinv = T.inverse();

		Eigen::Quaterniond q(T.rotation());
		Vector t(T.translation());
	
		std::stringstream ss;
// 		std::stringstream ssinv;
		ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
// 		ssinv<<tinv[0]<<" "<< tinv[1] << " " << tinv[2] << " " << qinv.w() << " " << qinv.x() << " " << qinv.y() << " " << qinv.z();
		p_neighbour.put("transform", ss.str());
		p_neighbour.put("reachable", tempReachable[neighbour_it->first]);
// 		bool reachable = p_neighbour_it->second.get<bool>("reachable");
		p_neighbour_list.add_child("neighbour", p_neighbour);
		
		if(((*node_it)->mnId == 0) && (neighbour_it->first == maxKF)){
			cout << "circle" << endl;
			circle = true;
		}
		

      }
      if(((*node_it)->mnId == 0) && (!circle)){
		  boost::property_tree::ptree p_neighbour;
		  p_neighbour.put("id", endkf->mnId);
		  PM::TransformationParameters pose_matrix = endkf->getPose();
			Eigen::Isometry3d iso3;
			iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
			iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
			Eigen::Isometry3d T(iso3);
	// 		Transform Tinv = T.inverse();

			Eigen::Quaterniond q(T.rotation());
			Vector t(T.translation());
		
			std::stringstream ss;
	// 		std::stringstream ssinv;
			ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
	// 		ssinv<<tinv[0]<<" "<< tinv[1] << " " << tinv[2] << " " << qinv.w() << " " << qinv.x() << " " << qinv.y() << " " << qinv.z();
			p_neighbour.put("transform", ss.str());
			p_neighbour.put("reachable", true);
	// 		bool reachable = p_neighbour_it->second.get<bool>("reachable");
			p_neighbour_list.add_child("neighbour", p_neighbour);
	  }
	  if(((*node_it)->mnId == maxKF) && (!circle) ){
		  boost::property_tree::ptree p_neighbour;
		  p_neighbour.put("id", endkf->mnId);
		  PM::TransformationParameters pose_matrix = endkf->getPose().inverse();
			Eigen::Isometry3d iso3;
			iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
			iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
			Eigen::Isometry3d T(iso3);
	// 		Transform Tinv = T.inverse();

			Eigen::Quaterniond q(T.rotation());
			Vector t(T.translation());
		
			std::stringstream ss;
	// 		std::stringstream ssinv;
			ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
	// 		ssinv<<tinv[0]<<" "<< tinv[1] << " " << tinv[2] << " " << qinv.w() << " " << qinv.x() << " " << qinv.y() << " " << qinv.z();
			p_neighbour.put("transform", ss.str());
			p_neighbour.put("reachable", true);
	// 		bool reachable = p_neighbour_it->second.get<bool>("reachable");
			p_neighbour_list.add_child("neighbour", p_neighbour);
		  
	}
	cout << endl;
      p_node.add_child("neighbour_list", p_neighbour_list);
      p_node_list.add_child("node", p_node);

    }

    p_map.add_child("map.node_list", p_node_list);

    boost::property_tree::xml_writer_settings<string> setting(' ', 2);
    boost::property_tree::write_xml(filename, p_map, std::locale(), setting);
    string wholeMap = ros::package::getPath("laser_mapping")+"/Map/wholeMap.vtk";
//     cerr<<"1"<<endl;
    mWholeMapPoints->save(wholeMap);
//     cerr<<"2"<<endl;
    //TEST save frames
    boost::property_tree::ptree p_frame_map;
    boost::property_tree::ptree p_frames_list;
    cerr<<" "<<mspKeyFrames.size()<<endl;
    cerr<<" "<<max_KF_ID<<endl;
    cerr<<"is frames right "<<mFrame_Record.size()<<" "<<(double)(mspKeyFrames.size()-max_KF_ID)<<endl;//beginID ->mspKeyFrames intial size
    for (auto node_it = (mspKeyFrames.begin() + beginID); node_it != (mspKeyFrames.end()-1); ++node_it){
	    boost::property_tree::ptree p_node;
	    p_node.put("keyframe_id", (*node_it)->mnId);
	    p_node.put("keyframe_frame_id", (*node_it)->mnFrameId );
// 	    cerr<<"keyframe "<< (*node_it)->mnId<<endl;
	    boost::property_tree::ptree p_neighbour_list;
	     for (auto neighbour_it = mFrame_Record[(*node_it)->mnId-max_KF_ID].begin(); neighbour_it != mFrame_Record[(*node_it)->mnId-max_KF_ID].end(); ++neighbour_it){
			boost::property_tree::ptree p_neighbour;
			p_neighbour.put("id", (*neighbour_it)->mnId);
// 			cerr<<"frame "<< (*neighbour_it)->mnId;
			p_neighbour.put("time", (*neighbour_it)->mstamp);
			PM::TransformationParameters pose_matrix =  (*neighbour_it)->mLocalTicp;
			// 	Eigen::Isometry3d iso3(pose_matrix.cast<double>());
			Eigen::Isometry3d iso3;
			iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
			iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
			Eigen::Isometry3d T(iso3);
			Eigen::Quaterniond q(T.rotation());
			Vector t(T.translation());

			std::stringstream ss;
			ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
			p_neighbour.put("transform", ss.str());
			GPS gps = (*neighbour_it)->gps;
			p_neighbour.put("gps.northing", gps.northing );
			p_neighbour.put("gps.easting", gps.easting );
			p_neighbour.put("gps.altitude", gps.altitude);
			p_neighbour.put("gps.roll", gps.roll);
			p_neighbour.put("gps.pitch", gps.pitch);
			p_neighbour.put("gps.yaw", gps.yaw);
			p_neighbour.put("gps.status", gps.status);
		        p_neighbour_list.add_child("neighbour_frame", p_neighbour);
	}
	p_node.add_child("neighbor_frame_list", p_neighbour_list);
	p_frames_list.add_child("keyframes",p_node);
// 	cerr<<endl;
}
	p_frame_map.add_child("map.frame_list", p_frames_list);
	stringstream frame;
	frame<<filename<<"_frames.xml";
	boost::property_tree::write_xml(frame.str(), p_frame_map, std::locale(), setting);
//     p_frames.put("id", );
  }

void Map::savewithGlobalPose(const string& filename)
{
///##########################
	cerr<<"saving map begin ID  "<<beginID<<endl;
	cerr<<"all keyframes "<<mspKeyFrames.size()<<endl;
    boost::property_tree::ptree p_map;
    p_map.put("map.property.node_count", mspKeyFrames.size());

    boost::property_tree::ptree p_node_list;
	int maxKF = mspKeyFrames.size()-1;
	KeyFrame* endkf = mspKeyFrames[maxKF];
	bool circle = false;
    for (auto node_it = (mspKeyFrames.begin()); node_it != mspKeyFrames.end(); ++node_it)
    {
      boost::property_tree::ptree p_node;
      p_node.put("id", (*node_it)->mnId);
      p_node.put("inMap", true);	//TEST
//       p_node.put("id", (*node_it)->mnFrameId);	//TEST frame 改进版
//       p_node.put("is_Keyframe", true);	//TEST 
      boost::property_tree::ptree p_neighbour_list;
       map<long unsigned int, bool> tempReachable = (*node_it)->neighbours_isReachable;
	  
// 	  cout << (*node_it)->mnId <<"-";
      for (auto neighbour_it = (*node_it)->neighbours.begin(); neighbour_it != (*node_it)->neighbours.end(); ++neighbour_it)
      {
		boost::property_tree::ptree p_neighbour;
// 		p_neighbour.put("id", mspKeyFrames[neighbour_it->first]->mnFrameId);//TEST 
		p_neighbour.put("id", neighbour_it->first);
// 		/*cout*/ << neighbour_it->first <<"-";
	//         Transform T(neighbour_it->second.cast<double>());
		PM::TransformationParameters pose_matrix = neighbour_it->second;
	// 	Eigen::Isometry3d iso3(pose_matrix.cast<double>());
		Eigen::Isometry3d iso3;
		iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
		iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
		Eigen::Isometry3d T(iso3);
// 		Transform Tinv = T.inverse();

		Eigen::Quaterniond q(T.rotation());
		Vector t(T.translation());
	
		std::stringstream ss;
// 		std::stringstream ssinv;
		ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
// 		ssinv<<tinv[0]<<" "<< tinv[1] << " " << tinv[2] << " " << qinv.w() << " " << qinv.x() << " " << qinv.y() << " " << qinv.z();
		p_neighbour.put("transform", ss.str());
		p_neighbour.put("reachable", true);
// 		bool reachable = p_neighbour_it->second.get<bool>("reachable");
		p_neighbour_list.add_child("neighbour", p_neighbour);
		
		if(((*node_it)->mnId == 0) && (fabs(neighbour_it->first-(*node_it)->mnId) >1)){
			cout << "circle" << endl;
			circle = true;
		}
		
		if(((*node_it)->mnId == maxKF) && (fabs(neighbour_it->first-(*node_it)->mnId) >1)){
			cout << "circle" << endl;
			circle = true;
		}
		

      }
      if(((*node_it)->mnId == 0) && (!circle)){
		  LOG(WARNING) << "Add false loop to the xml for 0 vertex";
		  boost::property_tree::ptree p_neighbour;
		  p_neighbour.put("id", endkf->mnId);
		  PM::TransformationParameters pose_matrix = endkf->getPose();
			Eigen::Isometry3d iso3;
			iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
			iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
			Eigen::Isometry3d T(iso3);
	// 		Transform Tinv = T.inverse();

			Eigen::Quaterniond q(T.rotation());
			Vector t(T.translation());
		
			std::stringstream ss;
	// 		std::stringstream ssinv;
			ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
	// 		ssinv<<tinv[0]<<" "<< tinv[1] << " " << tinv[2] << " " << qinv.w() << " " << qinv.x() << " " << qinv.y() << " " << qinv.z();
			p_neighbour.put("transform", ss.str());
			p_neighbour.put("reachable", true);
	// 		bool reachable = p_neighbour_it->second.get<bool>("reachable");
			p_neighbour_list.add_child("neighbour", p_neighbour);
	  }
	  if(((*node_it)->mnId == maxKF) && (!circle) ){
		  LOG(WARNING) << "Add false loop to the xml for " << maxKF <<" vertex";
		  boost::property_tree::ptree p_neighbour;
		  p_neighbour.put("id", 0);
		  PM::TransformationParameters pose_matrix = endkf->getPose().inverse();
			Eigen::Isometry3d iso3;
			iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
			iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
			Eigen::Isometry3d T(iso3);
	// 		Transform Tinv = T.inverse();

			Eigen::Quaterniond q(T.rotation());
			Vector t(T.translation());
		
			std::stringstream ss;
	// 		std::stringstream ssinv;
			ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
	// 		ssinv<<tinv[0]<<" "<< tinv[1] << " " << tinv[2] << " " << qinv.w() << " " << qinv.x() << " " << qinv.y() << " " << qinv.z();
			p_neighbour.put("transform", ss.str());
			p_neighbour.put("reachable", true);
	// 		bool reachable = p_neighbour_it->second.get<bool>("reachable");
			p_neighbour_list.add_child("neighbour", p_neighbour);
		  
	}
// 	cout << endl;
      p_node.add_child("neighbour_list", p_neighbour_list);
      p_node_list.add_child("node", p_node);

    }
///##########################
    p_map.add_child("map.node_list", p_node_list);

    boost::property_tree::xml_writer_settings<string> setting(' ', 2);
    boost::property_tree::write_xml(filename, p_map, std::locale(), setting);
	
    string wholeMap = ros::package::getPath("laser_mapping")+"/Map/wholeMap.vtk";
	string wholeMap_ply = ros::package::getPath("laser_mapping")+"/Map/wholeMap.ply";
//     cerr<<"1"<<endl;
// 	cout <<"save map to "<< wholeMap << endl;
	cerr<<mWholeMapPoints->features.cols()<<" points"<<endl;
//      mWholeMapPoints->save(wholeMap);
// 	mWholeMapPoints->save(wholeMap_ply);
// 	cout <<"save map points done"<< endl;

	
//     cerr<<"2"<<endl;
    //TEST save frames
    boost::property_tree::ptree p_frame_map;
    boost::property_tree::ptree p_frames_list;
    cerr<<" "<<mspKeyFrames.size()<<endl;
    cerr<<" "<<max_KF_ID<<endl;
    cerr<<"is frames right "<<mFrame_Record.size()<<" "<<(double)(mspKeyFrames.size()-max_KF_ID)<<endl;//beginID ->mspKeyFrames intial size
    for (auto node_it = (mspKeyFrames.begin() + max_KF_ID); node_it != (mspKeyFrames.end()-1); ++node_it){
	    boost::property_tree::ptree p_node;
	    p_node.put("keyframe_id", (*node_it)->mnId);
	    p_node.put("keyframe_frame_id", (*node_it)->mnFrameId );
// 	    cerr<<"keyframe "<< (*node_it)->mnId<<endl;
	    boost::property_tree::ptree p_neighbour_list;
	     for (auto neighbour_it = mFrame_Record[(*node_it)->mnId-max_KF_ID].begin(); neighbour_it != mFrame_Record[(*node_it)->mnId-max_KF_ID].end(); ++neighbour_it){
			boost::property_tree::ptree p_neighbour;
			p_neighbour.put("id", (*neighbour_it)->mnId);
// 			cerr<<"frame "<< (*neighbour_it)->mnId;
			p_neighbour.put("time", (*neighbour_it)->mstamp);
			PM::TransformationParameters pose_matrix =  (*neighbour_it)->mLocalTicp;
			// 	Eigen::Isometry3d iso3(pose_matrix.cast<double>());
			Eigen::Isometry3d iso3;
			iso3 = pose_matrix.block<3,3>(0,0).cast<double>();
			iso3.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
			Eigen::Isometry3d T(iso3);
			Eigen::Quaterniond q(T.rotation());
			Vector t(T.translation());

			std::stringstream ss;
			ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
			p_neighbour.put("transform", ss.str());
			GPS gps = (*neighbour_it)->gps;
			p_neighbour.put("gps.timestamp", gps.timestamp );
			p_neighbour.put("gps.northing", gps.northing );
			p_neighbour.put("gps.easting", gps.easting );
			p_neighbour.put("gps.altitude", gps.altitude);
			p_neighbour.put("gps.latitude", gps.latitude);
			p_neighbour.put("gps.longitude", gps.longitude);
			p_neighbour.put("gps.roll", gps.roll);
			p_neighbour.put("gps.pitch", gps.pitch);
			p_neighbour.put("gps.yaw", gps.yaw);
			p_neighbour.put("gps.satellites_used", gps.satellites_used);
			p_neighbour.put("gps.status", gps.status);
		        p_neighbour_list.add_child("neighbour_frame", p_neighbour);
	}
	p_node.add_child("neighbor_frame_list", p_neighbour_list);
	p_frames_list.add_child("keyframes",p_node);
// 	cerr<<endl;
}
	p_frame_map.add_child("map.frame_list", p_frames_list);
	stringstream frame;
	frame<<filename<<"_frames.xml";
	boost::property_tree::write_xml(frame.str(), p_frame_map, std::locale(), setting);
//     p_frames.put("id", );
}

  
bool Map::load(const string& filename)
{
     boost::property_tree::ptree pxml;
      boost::property_tree::read_xml(filename, pxml);
      int Num_KF = pxml.get<int>("map.property.node_count");
      cerr<<"Num of KeyFrame in Map "<<Num_KF<<endl;
	  
	 
      
      string KFbase; 
	  if(!ros::param::get("~FramesReadingPath", KFbase)){
		  cerr<<"ERROR loading mappoints"<<endl;
		  return false;
	}
		  
 
      boost::property_tree::ptree pMap = pxml.get_child("map.node_list");
	  
	  set<int> Keyframes_id_with_pose;
     
      // 从Map.xml里面读取localMap的id
      for (auto p_node_it = pMap.begin(); p_node_it != pMap.end(); ++p_node_it)
      {
		KeyFrame* newKeyFrame = new KeyFrame();
		int node_id = p_node_it->second.get<int>("id");
		cerr<<endl<<endl<<node_id<<endl;	//first是node
		newKeyFrame->mnId = node_id;
		stringstream nodestring;
		nodestring<<KFbase<<newKeyFrame->mnId<<"/DataPoints.vtk";
		newKeyFrame->mLocalMapPoints = new DP(DP::load(nodestring.str()));

		PM::TransformationParameters tempT = PM::TransformationParameters::Identity(4,4);

		bool setPose = false;//used for setting pose for node
		if(node_id == 0)
		{
			setPose = true;
			newKeyFrame->SetPose(tempT);///////////////////////
		}
		
		 boost::property_tree::ptree p_neighbour_list = p_node_it->second.get_child("neighbour_list");
		 for (auto p_neighbour_it = p_neighbour_list.begin(); p_neighbour_it != p_neighbour_list.end(); ++p_neighbour_it)
		 {
			 pair<long unsigned int, PM::TransformationParameters> temp_neighbor;
			 pair<long unsigned int, bool> temp_neighbor_reachable;
			int neighbour_id = p_neighbour_it->second.get<int>("id");
			assert(neighbour_id >= 0 && neighbour_id < nodes_.size());

			temp_neighbor.first = neighbour_id;
			temp_neighbor_reachable.first = neighbour_id;
			    
			std::string transform_str = p_neighbour_it->second.get<std::string>("transform");
			PM::TransformationParameters temp_T = PM::TransformationParameters::Identity(4, 4);
			std::stringstream ss(transform_str);
			double x, y, z, qw, qx, qy, qz;
			ss >> x >> y >> z >> qw >> qx >> qy >> qz;
			Vector t(x, y, z);
			Eigen::Quaterniond q(qw, qx, qy, qz);
			temp_T.col(3).head(3) = t.cast<float>();
			temp_T.block(0,0,3,3) = q.toRotationMatrix().cast<float>();
			temp_neighbor.second = temp_T;
			cout<<temp_neighbor.second<<endl;
			newKeyFrame->neighbours.insert(temp_neighbor);
			bool reachable = p_neighbour_it->second.get<bool>("reachable");
			temp_neighbor_reachable.second = reachable;
			newKeyFrame->neighbours_isReachable.insert(temp_neighbor_reachable);
			
// 			if((neighbour_id == (node_id-1))&&(!setPose))
			if(!setPose)
			{
				set<int>::iterator neighbor_found = Keyframes_id_with_pose.find(neighbour_id);
				if(neighbor_found != Keyframes_id_with_pose.end()){
					PM::TransformationParameters tempT = mspKeyFrames[*neighbor_found]->getPose();
					tempT = tempT*temp_T.inverse();//Tneigh * deltaT
					newKeyFrame->SetPose(tempT);
					setPose = true;
				}
			}
			
		}
		if(setPose){
			AddKeyFrame(newKeyFrame);
			Keyframes_id_with_pose.insert(newKeyFrame->mnId);
		}
		else{
			stringstream posestring;
			posestring<<KFbase<<newKeyFrame->mnId<<"/GlobalPose.txt";
			ifstream fT;
			fT.open(posestring.str().c_str());
			PM::TransformationParameters tempT = PM::TransformationParameters::Identity(4,4);
			for(int i = 0;i < 4; i++)
			{
				double a[4];
				fT >> a[0]>>a[1]>> a[2]>>a[3];
				tempT(i,0) = a[0];	tempT(i,1) = a[1];	tempT(i,2) = a[2];	tempT(i,3) = a[3];
			}
			newKeyFrame->SetPose(tempT);
			cerr<<"error!!! shouldn't exist"<<endl<<endl<<endl;
			AddKeyFrame(newKeyFrame);
		}
	}
      
         max_KF_ID = getKeyFrameNum();
		 KeyFrame::setNextId(max_KF_ID);
		 cout<<"total load "<<max_KF_ID<<" submaps"<<endl;
		 beginID = 0;
		 
      return true;
      
}/*
bool Map::linkNodes()
{
	KeyFrame* center = getKeyFrame(fixedNode);	//定义id = fixedNode是起点，有pose
	
	GPS centergps;
	center->getGPS(centergps);
// 	
// 	if((centergps.status == 0) && (optimizeWithGPS))	
// 	{
// 		cerr<<"center gps status "<<centergps.status<<endl;
// 		cerr<<"Error! fixed node "<<center->mnId<<" does not have gps value, please select a fixed node with gps value for global localization"<<endl;
// 		return false;
// 	}
	
	set<long unsigned int> nodes_centers;		//id	待搜索位
	set<long unsigned int> nodes_with_poses;	//id  labels 位
	
	nodes_centers.insert(fixedNode);
	nodes_with_poses.insert(fixedNode);
	
	vKeyFrames.push_back(center);

// 	center->setGeoXYZ(Rotationg_o, OriPose);
	
	set<long unsigned int>::iterator center_it = nodes_centers.begin();
	for( ; center_it != nodes_centers.end(); center_it++ ){
		//搜索临域
		center = getKeyFrame( *center_it );

		map<long unsigned int, PM::TransformationParameters>::iterator neighbor_it = center->neighbours.begin();

		for( ; neighbor_it != center->neighbours.end(); neighbor_it++ ){

			if( nodes_with_poses.find( neighbor_it->first) == nodes_with_poses.end() ){ 
				
				GPS mgps;
				getKeyFrame(neighbor_it->first)->getGPS(mgps);
				
				
				PM::TransformationParameters tempT ;
				tempT = center->getPose() * neighbor_it->second;

				getKeyFrame(neighbor_it->first)->SetPose(tempT);
				
				nodes_centers.insert(neighbor_it->first);
				nodes_with_poses.insert(neighbor_it->first);
				

// 				getKeyFrame(neighbor_it->first)->setGeoXYZ(Rotationg_o, OriPose);
// 				if( !(mgps.status)) continue;
// 				getKeyFrame(neighbor_it->first)->setGeoENA(OriENA);
				vKeyFrames.push_back(getKeyFrame(neighbor_it->first));


			}
		}

		nodes_centers.erase(center_it);
		
	}
	cerr<<"There is "<<vKeyFrames.size()<<" nodes in Map with gps value"<<endl;
// 	if( vKeyFrames.size() != MapSize ){
// 		ROS_ERROR_STREAM("link error! ");
// 		return false;
// 	}
	return true;
}*/
// /*
// bool Map::load(const string& filename, long unsigned int pfixedNode)
// {
// 	fixedNode = pfixedNode;
// 	
//       boost::property_tree::ptree pxml;
//       boost::property_tree::read_xml(filename, pxml);
//       int Num_KF = pxml.get<int>("map.property.node_count");
//       cerr<<"[loading map] Num of KeyFrame in Map "<<Num_KF<<endl;
//       MapSize = Num_KF;
//       
//       string KFbase = ros::package::getPath("offline_optimization")+"/KeyFrame/frames/";
//  
//       boost::property_tree::ptree pMap = pxml.get_child("map.node_list");
//       
//       bool FIXED = false;
//      
//       for (auto p_node_it = pMap.begin(); p_node_it != pMap.end(); ++p_node_it)
//       {
// 	        KeyFrame* newKeyFrame = new KeyFrame();
// 	        int node_id = p_node_it->second.get<int>("id");
// 		cerr<<endl<<endl<<node_id<<endl;	//first是node
// 		newKeyFrame->mnId = node_id;
// 		
// 		pair<long unsigned int, KeyFrame*> pair_id_kf = 
// 			pair<long unsigned int, KeyFrame*>(newKeyFrame->mnId, newKeyFrame);
// // 		int isInMap = p_node_it->second.get_child("").count("newNode");
// 		stringstream historystring;
// 		historystring<<KFbase<<node_id<<"/history.txt";
// 		if( (access(historystring.str().c_str(), 0 )  == 0 )){
// // 			newKeyFrame->setNewNode(true);//FIXME  别忘了改回去
// 		}
// 		
// 		newKeyFrame->setNewNode(true);
// 		stringstream nodestring;
// 		nodestring<<KFbase<<newKeyFrame->mnId<<"/DataPoints.vtk";
// 		newKeyFrame->mLocalMapPoints = new DP(DP::load(nodestring.str()));
// 		
// 		stringstream gpsstring;
// 		gpsstring<<KFbase<<newKeyFrame->mnId<<"/gps.xml";
// 		
// 		if( ( access(gpsstring.str().c_str(), 0 ) ) == 0 ){
// 			
// 			boost::property_tree::ptree pgps;
// 			boost::property_tree::read_xml( gpsstring.str(), pgps );
// 			
// 			 boost::property_tree::ptree pstatus = pgps.get_child("gps.status");
// 			
// // 			 if( pstatus.get<int>("status") != 0 ){
// 				cerr<<"loading node "<<newKeyFrame->mnId<<" with gps"<<endl;
// // 				newKeyFrame->setGPS(pgps.get<double>("gps.longitude"), pgps.get<double>("gps.latitude"), pstatus.get<int>("status") );
// 				newKeyFrame->setGPSENA( pgps.get<double>("gps.easting"), pgps.get<double>("gps.northing"), pgps.get<double>("gps.altitude"), pgps.get<double>("gps.status.status") );
// 			
// // 			}
// 			if( node_id == fixedNode) {
// 				
// 				PM::TransformationParameters tempT = PM::TransformationParameters::Identity(4,4);
// 				newKeyFrame->SetPose(tempT);//TODO, change to gps
// 				
// 				GPS mgps;
// 				newKeyFrame->getGPS(mgps);
// // 				geoXYZ mgeo;
// // 				CoorConvert::transformGPStoGeoXYZ(mgps, mgeo);
// 				// set easting northing altitude
// 				OriENA(0) = mgps.easting;	OriENA(1) = mgps.northing;		OriENA(2) = mgps.altitude;
// 				cerr<<"OriENA "<<OriENA(0)<<" "<<OriENA(1)<<" "<<OriENA(2)<<endl;
// 				FIXED = true;
// 			}
// 		}else{
// 				cerr<<"node "<<newKeyFrame->mnId<<" doesn't have gps value"<<endl;
// 			
// 		}/*processing gps.xml*/
// 		
// 		 boost::property_tree::ptree p_neighbour_list = p_node_it->second.get_child("neighbour_list");
// 		 for (auto p_neighbour_it = p_neighbour_list.begin(); p_neighbour_it != p_neighbour_list.end(); ++p_neighbour_it)
// 		 {
// 			 pair<long unsigned int, PM::TransformationParameters> temp_neighbor;
// 			 pair<long unsigned int, bool> temp_neighbor_reachable;
// 			int neighbour_id = p_neighbour_it->second.get<int>("id");
// 			assert(neighbour_id >= 0 && neighbour_id < nodes_.size());
// 			temp_neighbor.first = neighbour_id;
// 			temp_neighbor_reachable.first = neighbour_id;
// 			    
// 			std::string transform_str = p_neighbour_it->second.get<std::string>("transform");
// 			PM::TransformationParameters temp_T = PM::TransformationParameters::Identity(4, 4);
// 			std::stringstream ss(transform_str);
// 			double x, y, z, qw, qx, qy, qz;
// 			ss >> x >> y >> z >> qw >> qx >> qy >> qz;
// 			Vector t(x, y, z);
// 			Quaternion q(qw, qx, qy, qz);
// 			temp_T.col(3).head(3) = t.cast<float>();
// 			temp_T.block(0,0,3,3) = q.toRotationMatrix().cast<float>();
// 			temp_neighbor.second = temp_T;
// // 			cout<<temp_neighbor.second<<endl;
// 			newKeyFrame->neighbours.insert(temp_neighbor);
// 			bool reachable = p_neighbour_it->second.get<bool>("reachable");
// // 			temp_neighbor_reachable.second = reachable;
// 			temp_neighbor_reachable.second = true;	//FIXME reachable
// 			newKeyFrame->neighbours_isReachable.insert(temp_neighbor_reachable);
// 			
// 		}
// 
// 		mKeyFrames.insert(pair_id_kf);	//for quick index
// 	}
// 	if( !FIXED ){
// 		ROS_ERROR_STREAM("map without fixed node "<< fixedNode);
// 		return false;
// 	}
// 	
// 
//             return true;
// }
// */
bool Map::loadwithoutDP(const string& filename)
{
      
      boost::property_tree::ptree pxml;
      boost::property_tree::read_xml(filename, pxml);
      int Num_KF = pxml.get<int>("map.property.node_count");
      cerr<<"Num of KeyFrame in Map "<<Num_KF<<endl;
      boost::property_tree::ptree pMap = pxml.get_child("map.node_list"); 
      string KFbase = ros::package::getPath("laser_mapping")+"/KeyFrame/";
      max_KF_ID = 0;

      for (auto p_node_it = pMap.begin(); p_node_it != pMap.end(); ++p_node_it)
      {
	      KeyFrame* newKeyFrame = new KeyFrame();
	       int node_id = p_node_it->second.get<int>("id");
		cerr<<endl<<endl<<node_id<<endl;	//first是node
		newKeyFrame->mnId = node_id;
		if(max_KF_ID<newKeyFrame->mnId)
			max_KF_ID = node_id;
// 		cerr<<"data() "<<(p_node_it->second.get_child("")).count("id")<<endl;
		stringstream posestring;
		posestring<<KFbase<<newKeyFrame->mnId<<"/GlobalPose.txt";
		ifstream fT;
		fT.open(posestring.str().c_str());
		PM::TransformationParameters tempT = PM::TransformationParameters::Identity(4,4);
		for(int i = 0;i < 4; i++)
		{
			double a[4];
			fT >> a[0]>>a[1]>> a[2]>>a[3];
			tempT(i,0) = a[0];	tempT(i,1) = a[1];	tempT(i,2) = a[2];	tempT(i,3) = a[3];
		}

		newKeyFrame->SetPose(tempT);
		
		 boost::property_tree::ptree p_neighbour_list = p_node_it->second.get_child("neighbour_list");
		 for (auto p_neighbour_it = p_neighbour_list.begin(); p_neighbour_it != p_neighbour_list.end(); ++p_neighbour_it)
		 {
			 pair<long unsigned int, PM::TransformationParameters> temp_neighbor;
			 pair<long unsigned int, bool> temp_neighbor_reachable;
			int neighbour_id = p_neighbour_it->second.get<int>("id");
			assert(neighbour_id >= 0 && neighbour_id < nodes_.size());
			temp_neighbor.first = neighbour_id;
			temp_neighbor_reachable.first = neighbour_id;
			    
			std::string transform_str = p_neighbour_it->second.get<std::string>("transform");
			PM::TransformationParameters temp_T = PM::TransformationParameters::Identity(4, 4);
			std::stringstream ss(transform_str);
			double x, y, z, qw, qx, qy, qz;
			ss >> x >> y >> z >> qw >> qx >> qy >> qz;
			Vector t(x, y, z);
			Eigen::Quaterniond q(qw, qx, qy, qz);
			temp_T.col(3).head(3) = t.cast<float>();
			temp_T.block(0,0,3,3) = q.toRotationMatrix().cast<float>();
			temp_neighbor.second = temp_T;
			cout<<temp_neighbor.second<<endl;
			newKeyFrame->neighbours.insert(temp_neighbor);
			bool reachable = p_neighbour_it->second.get<bool>("reachable");
			temp_neighbor_reachable.second = reachable;
			newKeyFrame->neighbours_isReachable.insert(temp_neighbor_reachable);
			
		}
		AddKeyFrame(newKeyFrame);
	}
	


       beginID = getKeyFrameNum();//勿改
// 	beginID = Num_KF;
// 	Velodyne_SLAM::KeyFrame::nNextId = Num_KF;
       max_KF_ID++;
       Velodyne_SLAM::KeyFrame::setNextId(max_KF_ID);
//        cerr<<"xin lei"<<endl;
//        KeyFrame* newkf = new KeyFrame();
//        cerr<<"kankan "<<newkf->mnId<<endl;
       
      return true;
}

bool Map::LoadfromXML(const string& filename)
{
	boost::property_tree::ptree pxml;
	boost::property_tree::read_xml(filename, pxml);
	boost::property_tree::ptree pMap = pxml.get_child("map.node_list");	//<!-get_child would return a child ptree from the parent node  
	
	int node_count = pxml.get<int>("map.property.node_count");
	ROS_INFO_STREAM(node_count<<" nodes in xml");
	
	string node_data_base = ros::package::getPath("laser_mapping")+"/KeyFrame/";
	int count_pointer_in_mspKF = 0;
	set<int > nodes_waiting_for_T;
	set<KeyFrame*> nodes_for_SLAM;	//之后捋一遍，排序forSLAM
	long unsigned int fixedFirstId;
	//<!-read from the xml to construct the mspKeyFrames and a map connecting the mnId and the pointer to the KeyFrame in mspKeyFrames
	for(auto p_node_it = pMap.begin(); p_node_it!=pMap.end(); ++p_node_it){
		
		KeyFrame* newKeyFrame = new KeyFrame();
		int node_id = p_node_it->second.get<int>("id");
		newKeyFrame->mnId = node_id;
		mmnId_to_vPointer.insert(pair<long unsigned int, long unsigned int>(node_id, count_pointer_in_mspKF++));
		cerr<<"node "<<node_id;
		
		int isInMap = p_node_it->second.get_child("").count("inMap");
		if( isInMap == 0 ){
			stringstream node_data_string;
			node_data_string<<node_data_base<<newKeyFrame->mnId<<"/DataPoints.vtk";
			newKeyFrame->mLocalMapPoints = new DP(DP::load(node_data_string.str()));
			nodes_for_SLAM.insert(newKeyFrame);
		}
		
		/*TODO waiting for gps
		 * 
		 * 
		*/
		
		PM::TransformationParameters keyframeT = PM::TransformationParameters::Identity(4,4);
		bool setPose = false;	//<!- used for setting pose in case encounter some error
		if(node_id == 0){//TODO 这个初始的KeyFrame用什么判断，id == 0吗
			//TODO to change for gps 
			newKeyFrame->SetPose(keyframeT);
			fixedFirstId = (count_pointer_in_mspKF-1);
		}
		
		boost::property_tree::ptree p_neighbour_list = p_node_it->second.get_child("neighbour_list");
		cerr<<" has "<<p_neighbour_list.count("neighbour")<<" neighbors"<<endl;
		for(auto p_neighbour_it = p_neighbour_list.begin(); p_neighbour_it != p_neighbour_list.end(); ++p_neighbour_it){
			
			pair<long unsigned int, PM::TransformationParameters> temp_neighbor;
			pair<long unsigned int, bool> temp_neighbor_reachable;
			
			int neighbour_id = p_neighbour_it->second.get<int>("id");
// 			assert(neighbour_id >= 0 && neighbour_id < nodes_.size());	//<!- 不晓得id是啥样子的诶
			temp_neighbor.first = neighbour_id;
			temp_neighbor_reachable.first = neighbour_id;
			
			std::string transform_str = p_neighbour_it->second.get<std::string>("transform");
			PM::TransformationParameters temp_T = PM::TransformationParameters::Identity(4, 4);
			std::stringstream ss(transform_str);	double x, y, z, qw, qx, qy, qz;
			ss >> x >> y >> z >> qw >> qx >> qy >> qz;
			Vector t(x, y, z);	Eigen::Quaterniond q(qw, qx, qy, qz);
			temp_T.col(3).head(3) = t.cast<float>();	temp_T.block(0,0,3,3) = q.toRotationMatrix().cast<float>();
			temp_neighbor.second = temp_T;	
// 			cout<<temp_neighbor.second<<endl;
			newKeyFrame->neighbours.insert(temp_neighbor);
			bool reachable = p_neighbour_it->second.get<bool>("reachable");
			temp_neighbor_reachable.second = reachable;
			newKeyFrame->neighbours_isReachable.insert(temp_neighbor_reachable);
			
			/*
// 			if(!setPose ){
// 				map<long unsigned int , long unsigned int>::iterator id_to_ptr_it = mmnId_to_vPointer.find(neighbour_id);
// 				if((id_to_ptr_it!=mmnId_to_vPointer.end()) && (mspKeyFrames[id_to_ptr_it->second]->isPoseSetted() )){				
// 					PM::TransformationParameters tempT = mspKeyFrames[id_to_ptr_it->second]->getPose();
// 					tempT = tempT*temp_T.inverse();
// 					newKeyFrame->SetPose(tempT);
// 					setPose = true;
// 				}
// 			}	
*/
		}/*neighbor list*/
		AddKeyFrameWithoutAddMapPoints(newKeyFrame);
		if((!setPose) && (isInMap)){
			nodes_waiting_for_T.insert(newKeyFrame->mnId);
		}
	}/*node list*/
	computeTransformation(nodes_waiting_for_T, fixedFirstId);
	cerr<<"transformation compute ok"<<endl;
	ROS_INFO_STREAM(nodes_for_SLAM.size()<<" new nodes waiting for SLAM");
	
	return true;

}

void Map::computeTransformation(std::set< int >& nodes_waiting_for_T, long unsigned int fixedFirstId) //<-是mspKeyFrame的id
{
	set< KeyFrame* > waiting_for_back;
	vector< KeyFrame* > base_node;   //<-which has position
	base_node.push_back( mspKeyFrames[fixedFirstId] );

	map<long unsigned int, PM::TransformationParameters>::iterator neighbour_it;
	for( int i = 0; i<base_node.size(); i++ ){
		neighbour_it = base_node[i]->neighbours.begin();
		for(; neighbour_it!=base_node[i]->neighbours.end(); neighbour_it++ ){
			if( nodes_waiting_for_T.find(neighbour_it->first) != nodes_waiting_for_T.end() ){
				PM::TransformationParameters tempT = base_node[i]->getPose() * (neighbour_it->second);
				mspKeyFrames[mmnId_to_vPointer.find(neighbour_it->first)->second]->SetPose(tempT);
				nodes_waiting_for_T.erase(neighbour_it->first);
				base_node.push_back(mspKeyFrames[mmnId_to_vPointer.find(neighbour_it->first)->second]);
				if ( nodes_waiting_for_T.size() == 0) break;
			}
			if ( nodes_waiting_for_T.size() == 0) break;
		}
		if ( nodes_waiting_for_T.size() == 0) break;
	}
	

}

bool Map::loadDPforInit(int id)
{
	int id_inmsp = mspKeyFrames.size()-1;
	for(; id_inmsp>=0;id_inmsp--){
		if(mspKeyFrames[id_inmsp]->mnId ==  id)
			break;
	}
	KeyFrame* pKF = mspKeyFrames[id_inmsp];
	mCurrentKF = pKF;
	string KFbase = ros::package::getPath("laser_mapping")+"/KeyFrame/";
	stringstream nodestring;
	nodestring<<KFbase<<id<<"/DataPoints.vtk";
	pKF->mLocalMapPoints = new DP(DP::load(nodestring.str()));
	
	//initial whole map points
// 	if(mWholeMapPoints)
// 		delete mWholeMapPoints;
// 	
// 	mWholeMapPoints = new DP(pKF->mLocalMapPoints->features,pKF->mLocalMapPoints->featureLabels
// 		,pKF->mLocalMapPoints->descriptors, pKF->mLocalMapPoints->descriptorLabels,
// 		pKF->mLocalMapPoints->times, pKF->mLocalMapPoints->timeLabels);
	
// 	*mWholeMapPoints = transformation->compute(*mWholeMapPoints, pKF->getPose());

}

void Map::showNodesPose()
{
	visualization_msgs::MarkerArray NodesTexts;	
	visualization_msgs::Marker points, text;
	points.header.frame_id = text.header.frame_id = "/map";
	points.header.stamp =text.header.stamp = ros::Time::now();
	points.ns =  "Nodes";	text.ns = "Nodes_id";
	points.action = text.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = text.pose.orientation.w = 1.0f;
	points.id = 0;	points.type = visualization_msgs::Marker::POINTS;	points.scale.x = points.scale.y = 1;
	points.color.a = 1.0;  points.color.r = 1.0f;
	text.id = 1;	text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;  text.scale.z = 5; 
	text.color.a = 1.0;   text.color.r = 1.0f;		
	int ID = 1;
	
	for( int i = 0; i < mspKeyFrames.size(); i++ ){
		KeyFrame* pKF = mspKeyFrames[i];
		PM::TransformationParameters tempT = pKF->getPose();
		geometry_msgs::Point p;
		p.x = tempT(0,3);	p.y = tempT(1,3);	p.z = tempT(2,3);
		points.points.push_back(p);
		text.text = to_string(pKF->mnId);
		visualization_msgs::Marker newtext = text;
		newtext.id = ID++;
		newtext.pose.position.x = p.x;	newtext.pose.position.y = p.y;	newtext.pose.position.z = p.z;
		NodesTexts.markers.push_back(newtext);
	}
	NodesTexts.markers.push_back( points );
	wholePosePublisher.publish(NodesTexts);

}


  
}
