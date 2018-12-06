#include <ros/ros.h>
#include<iostream>
#include<fstream>
#include<ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <nabo/nabo.h>

#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <pointmatcher/Timer.h>
#include <Eigen/Core>


// #include <ma
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
using namespace std;
int main(int argc, char **argv){
	
	ros::init(argc, argv, "showpoints");
	ros::start();
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("point_map",1);
	ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("point1",1);
	ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("point2",1);
	unique_ptr<PM::Transformation> transformation(PM::get().REG(Transformation).create("RigidTransformation"));
	

// 	data.removeDescriptor("normals");
	DP data = DP::load(argv[1]);
	
	
	PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);
	T.block<3,3>(0,0) = (Eigen::AngleAxisf(3.14/2, Eigen::Vector3f::UnitX()) *
					Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
					Eigen::AngleAxisf(-3.14/7, Eigen::Vector3f::UnitZ())).toRotationMatrix();
	
// 	data = transformation->compute(data, T);
// 	cerr <<data.features.cols()<<endl;
// 	for(int i = 0; i < data.descriptors.rows(); i++){
// 	  cout<<data.features(0,i)<<"  "<<data.features(1,i)<<"  "<<data.features(2,i) << endl;
// 	}
// 	

	ros::Rate r(0.5);
	  
	while(ros::ok()){
	    pub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data, "/map", ros::Time::now()));

	    r.sleep();
	}
}	