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
	
	ros::init(argc, argv, "compute_relative_pose_twolaser");
	ros::start();
	
	ros::NodeHandle n;

	
	ros::Publisher p_first = n.advertise<sensor_msgs::PointCloud2>("/first",1,true);
	ros::Publisher p_second = n.advertise<sensor_msgs::PointCloud2>("/second",1,true);
	
	PM::ICPSequence icp;
	unique_ptr<PM::Transformation> transformation(PM::get().REG(Transformation).create("RigidTransformation"));
	
	
	
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
	
	string first_id, second_id;
	ros::param::get("~laser1", first_id);
	ros::param::get("~laser2", second_id);
	
	cout <<"compute "<< first_id <<" -- "<< second_id << endl;
	
	DP data_first = DP::load(first_id);
	DP data_second = DP::load(second_id);
	
	
	ifstream f_delta(rectify_file_name);
	

	PM::TransformationParameters rectify_T = PM::TransformationParameters::Identity(4,4);
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
		{

			f_delta >> rectify_T(i,j);
		}
	

	cout << rectify_T << endl;
	
	
	
	
	icp.setMap(data_first);
	PM::TransformationParameters deltaT = rectify_T;

	
	PM::TransformationParameters computed_deltaT = deltaT;
	computed_deltaT = icp(data_second, deltaT);
	
	Eigen::Quaternion<float> q(computed_deltaT.block<3,3>(0,0));
	cout<<computed_deltaT(0,3)<<" "<<computed_deltaT(1,3)<<" "<<computed_deltaT(2,3)
	<<" "<< q.w()<<" " << q.x()<<" " << q.y()<<" "<<q.z()<< endl;
	
	PM::TransformationParameters computed_T_inv = computed_deltaT.inverse();
	Eigen::Quaternion<float> q_inv(computed_T_inv.block<3,3>(0,0));
	cout<<computed_T_inv(0,3)<<" "<<computed_T_inv(1,3)<<" "<<computed_T_inv(2,3)
	<<" "<< q.w()<<" " << q.x()<<" " << q.y()<<" "<<q.z()<< endl;
	
	
	data_second = transformation->compute(data_second, computed_deltaT);
	
	
	ros::Rate r(1);
	while(ros::ok()){
		p_first.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_first, "/map", ros::Time::now()));
		p_second.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data_second, "/map", ros::Time::now()));
		r.sleep();
	}
	
	
}
