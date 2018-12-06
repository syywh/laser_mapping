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
	
	ros::init(argc, argv, "merge_map");
	ros::start();
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("point_map",1);
	ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("point1",1);
	ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("point2",1);
	unique_ptr<PM::Transformation> transformation(PM::get().REG(Transformation).create("RigidTransformation"));
	

	ifstream inputf("/home/turtle/catkin_ws/src/dynamic_SLAM/cfg/input_filters.yaml");
	PM::DataPointsFilters inputFilters(inputf);
	
	ifstream outputf("/home/turtle/catkin_ws/src/dynamic_SLAM/cfg/map_post_filters.yaml");
	PM::DataPointsFilters outputfilters(outputf);
	
// 	data.removeDescriptor("normals");

	DP map0 = DP::load(argv[1]);
	cerr <<map0.features.cols()<<endl;
	
	DP map1 = DP::load(argv[2]);
	cerr <<map1.features.cols()<<endl;
	
	inputFilters.apply(map0);
	inputFilters.apply(map1);
	
	pub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(map0, "/map", ros::Time::now()));
	pub1.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(map1, "/map", ros::Time::now()));

	ifstream f("/home/turtle/catkin_ws/src/dynamic_SLAM/initPose.txt");
	PM::TransformationParameters TlastKeyframe = PM::TransformationParameters::Identity(4,4);
	for(int iii = 0; iii < 4; iii ++){
	  for(int jjj = 0;  jjj < 4; jjj++){
	    f >> TlastKeyframe(iii,jjj) ;
	  }
	}
	cout<<TlastKeyframe<<endl;
	

// 	Eigen::AngleAxis<float> ax;
// 	ax.axis() = Eigen::Vector3f(0, 0, 1);
// 	ax.angle() = 3.14/6;
// // 	;
// 	cout<<ax.matrix()<<endl<<endl;
// 	PM::TransformationParameters delta = PM::TransformationParameters::Identity(4,4);
// 	delta.block<3,3>(0,0) = ax.matrix();
// 	delta(1,3) = -30;
// 	delta(0,3) = 150;
// 	TlastKeyframe = TlastKeyframe * delta;
// 	cout<<TlastKeyframe<<endl;
	
	DP newmap = transformation->compute(map1, TlastKeyframe);
	pub2.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(newmap, "/map", ros::Time::now()));

	
	PM::ICPSequence icp_sequence;
	ifstream ff("/home/turtle/catkin_ws/src/dynamic_SLAM/cfg/icp_merge.yaml");
	icp_sequence.loadFromYaml(ff);
	icp_sequence.setMap(map0);
	TlastKeyframe = icp_sequence(map1, TlastKeyframe);
	cout<<TlastKeyframe<<endl;
	
	newmap = transformation->compute(map1, TlastKeyframe);
// 	newmap = transformation->compute(map1, TlastKeyframe);
	 cout<<newmap.features.cols()<<endl;

	  cout<<TlastKeyframe<<endl;
	  
	  map0.concatenate(newmap);
	  cout<<map0.features.cols()<<endl;
// 	  outputfilters.apply(map0);
	   cout<<map0.features.cols()<<endl;
	  map0.save("/home/turtle/catkin_ws/src/dynamic_SLAM/done.vtk");
	  cout<<"done"<<endl;
	ros::Rate r(0.5);
	  
	while(ros::ok()){
	    pub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(newmap, "/map", ros::Time::now()));
	    pub1.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(map0, "/map", ros::Time::now()));

	    r.sleep();
	}
}	