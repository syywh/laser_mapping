#include "ros/ros.h"
// #include "ros/console.h"
#include<iostream>
#include<fstream>
#include<ros/package.h>
#include<boost/thread.hpp>
#include <sensor_msgs/PointCloud2.h>

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "pointmatcher/PointMatcher.h"

using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "compute_relativeT_submaps");
	ros::start();
	
	ros::NodeHandle n;
	
	ros::Publisher p_first = n.advertise<sensor_msgs::PointCloud2>("/first",1,true);
	ros::Publisher p_second = n.advertise<sensor_msgs::PointCloud2>("/second",1,true);
	
	PM::ICPSequence icp;
	unique_ptr<PM::Transformation> transformation(PM::get().REG(Transformation).create("RigidTransformation"));
	
	string base_dir;
	ros::param::get("~directory", base_dir);
	cout <<"dir "<< base_dir << endl;
	
	string rectify_file_name;
	ros::param::get("~rectify_file",rectify_file_name);
	
	string icp_file_name;
	ros::param::get("~icp_file", icp_file_name);
	ifstream ifs(icp_file_name.c_str());
	if (ifs.good())
	{
		icp.loadFromYaml(ifs);//ICPChainBase function
	}else{
		cout <<"Can not load icp file"<<endl;
		return -1;
	}
	
	int first_id, second_id;
	ros::param::get("~submap1", first_id);
	ros::param::get("~submap2", second_id);
	
	cout <<"compute "<< first_id <<" -- "<< second_id << endl;
	
	DP data_first = DP::load(base_dir+"/"+to_string(first_id)+"/DataPoints.vtk");
	DP data_second = DP::load(base_dir+"/"+to_string(second_id)+"/DataPoints.vtk");
	
	ifstream f_first(base_dir+"/"+to_string(first_id)+"/GlobalPose.txt");
	ifstream f_second(base_dir+"/"+to_string(second_id)+"/GlobalPose.txt");
	
	ifstream f_delta(rectify_file_name);
	
	PM::TransformationParameters T_first = PM::TransformationParameters::Identity(4,4); 
	PM::TransformationParameters T_second = T_first;
	PM::TransformationParameters rectify_T = PM::TransformationParameters::Identity(4,4);
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
		{
			f_first >> T_first(i,j);
			f_second >> T_second(i,j);
			f_delta >> rectify_T(i,j);
		}
	
// 	data_first = transformation->compute(data_first, T_first);
// 	data_second = transformation->compute(data_second, T_second);
// 	cout << T_first << endl<< T_second<< endl;
	cout << rectify_T << endl;
	
	
	

	icp.setMap(data_first);
	PM::TransformationParameters deltaT = T_first.inverse() * T_second;
	deltaT(2,3) = 0;
	deltaT = deltaT * rectify_T;
	
	PM::TransformationParameters computed_deltaT = deltaT;
	computed_deltaT = icp(data_second, deltaT);
// 	computed_deltaT = icp(data_second, rectify_T);
	
	cout <<"compute deltaT \n" << computed_deltaT << endl;
	
	Eigen::Quaternion<float> q(computed_deltaT.block<3,3>(0,0));
	cout<<computed_deltaT(0,3)<<" "<<computed_deltaT(1,3)<<" "<<computed_deltaT(2,3)
	<<" "<< q.w()<<" " << q.x()<<" " << q.y()<<" "<<q.z()<< endl;
	
	PM::TransformationParameters computed_T_inv = computed_deltaT.inverse();
	Eigen::Quaternion<float> q_inv(computed_T_inv.block<3,3>(0,0));
	cout<<computed_T_inv(0,3)<<" "<<computed_T_inv(1,3)<<" "<<computed_T_inv(2,3)
	<<" "<< q.w()<<" " << q.x()<<" " << q.y()<<" "<<q.z()<< endl;
	
	
	data_second = transformation->compute(data_second, computed_deltaT);
	
	cout << T_first * computed_deltaT << endl;
	
	
	ros::Rate r(1);
	while(ros::ok()){
		p_first.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_first, "/map", ros::Time::now()));
		p_second.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_second, "/map", ros::Time::now()));
		r.sleep();
	}
	
	
}