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

int main(int argc, char **argv){
	
    ros::init(argc, argv, "opt_laser_gps");
    ros::start();
	
	ros::NodeHandle nh("~");
	
	std::string data_path;
	ros::param::get("~/data_path", data_path);
	
	std::cout <<"---Load data from " << data_path << std::endl;
	
	Velodyne_SLAM::Map mMap;
	
	mMap.loadwithoutDP(data_path, data_path+"/map.xml",0);
	mMap.linkNodes();
	
	//save frame poses and timestamp
	mMap.drawTracjectory(data_path);
	

	
	
	
		
	
}