#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/PoseStamped.h"


#include "glog/logging.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "fstream"

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"

#include "pointmatcher/PointMatcher.h"

#include "Map.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

int main(int argc, char **argv){
	
    ros::init(argc, argv, "construct_map_by_txt");
    ros::start();
	
	ros::NodeHandle nh("~");
	
	std::string data_path, trajectory_file_name, extrinsic_file_name;
	int keyframe_num;
	ros::param::get("~/submap_folder", data_path);
	ros::param::get("~/trajectory_file", trajectory_file_name);
	ros::param::get("~/folder_num", keyframe_num);
	ros::param::get("~/extrinsic_G_L", extrinsic_file_name);
	
// 	int num_kf = atoi(keyframe_num.c_str());
	
	std::cout <<"---Load data from " << data_path << std::endl;
	std::cout <<"---Load trajectory from " << trajectory_file_name << std::endl;
	std::cout <<"---Load folder_num " << keyframe_num << std::endl;
	std::cout <<"---Load extrinsic_G_L from " << extrinsic_file_name << std::endl;
	
	std::ifstream filters_name(ros::package::getPath("laser_mapping")+"/cfg/map_post_filters_forVIsual.yaml");
	PM::DataPointsFilters mapPostFilters = PM::DataPointsFilters(filters_name);
	
	unique_ptr<PM::Transformation> transformation(PM::get().REG(Transformation).create("RigidTransformation"));
	
	std::ifstream trajectory_file(trajectory_file_name);
	
	std::ifstream fTGL(extrinsic_file_name);
	PM::TransformationParameters TGL = PM::TransformationParameters::Identity(4,4);
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++){
			fTGL >> TGL(i,j);
		}
	std::cout <<"TGL:\n"<< TGL <<std::endl;
	
	DP wholemap;
	
	int keyframe_id = 0;
	
	bool init = false;
	
	bool read_pose = true;
	double time,tx,ty,tz,qx,qy,qz,qw;
	
	while(!(trajectory_file.eof())){
		DP sumap = DP::load(data_path+"/"+std::to_string(keyframe_id)+"/DataPoints.vtk");
		
		std::string gps_xml_name = data_path+"/"+std::to_string(keyframe_id)+"/gps.xml";
		boost::property_tree::ptree gpsxml;
		boost::property_tree::read_xml(gps_xml_name, gpsxml);
		double gpstime = gpsxml.get<double>("gps.timestamp");
		
		//read Pose
		if(read_pose)
			trajectory_file >> time >> tx >>ty>>tz>>qx>>qy>>qz>>qw;
		
		if(time-gpstime > 1){
			keyframe_id++;
			read_pose = false;
			continue;
		}
		
		read_pose = true;
		
		PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);
		T(0,3) = tx; T(1,3) = ty;	T(2,3) = tz;
		Eigen::Quaternionf q(qw, qx,qy,qz);
		T.block<3,3>(0,0) = q.toRotationMatrix();
		
// 		sumap = transformation->compute(sumap, TGL);
		sumap = transformation->compute(sumap, T);
		
		if(!init){
			wholemap = sumap;
			init = true;
		}
		else
			wholemap.concatenate(sumap);
		keyframe_id++;
		
		if(keyframe_id >= keyframe_num) break;
		
		if(trajectory_file.eof()) break;
	}
	
	std::cout <<"whole map points " <<wholemap.features.cols()<<std::endl;
	mapPostFilters.apply(wholemap);
	std::cout <<"whole map points filtered " <<wholemap.features.cols()<<std::endl;
	
	wholemap.save(data_path+"/wholemap_opt.vtk");
	
}