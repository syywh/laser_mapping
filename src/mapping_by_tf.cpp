#include "ros/ros.h"
#include "ros/console.h"
#include<iostream>
#include<fstream>
#include<ros/package.h>
#include<boost/thread.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

using namespace std;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapping_by_tf");
	ros::start();
	
	ros::NodeHandle n;
	
	ros::Publisher pointPub = n.advertise<sensor_msgs::PointCloud2>("point_map",1);
	
	unique_ptr<PM::Transformation> transformation(PM::get().REG(Transformation).create("RigidTransformation"));
	
	tf::TransformBroadcaster tfBroadcaster;
	
	string bagname;
	if(!ros::param::get("~bagname",bagname))
	{
		ROS_ERROR("Can not find bag name");
		exit(0);
	}
	
	string lasername;
	if(!ros::param::get("~cloud_in",lasername))
	{
		ROS_ERROR("Can not find lasername name");
		exit(0);
	}
	
	rosbag::Bag databag;
	databag.open(bagname, rosbag::bagmode::Read);
	
	vector<string> topics;
	topics.push_back(lasername);
	topics.push_back("/tf");
	
	rosbag::View view(databag, rosbag::TopicQuery(topics));
	
	DP wholemap;
	bool init = false;
	
	ros::Rate r(10);
	
	foreach(rosbag::MessageInstance const m, view){
	
	  if(!ros::ok())
			break;
	  
	  tf2_msgs::TFMessage::Ptr tfData = m.instantiate<tf2_msgs::TFMessage>();
	  PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);
	  double tftime;
	  if(tfData != NULL){
	    cout << "tf message " << tfData->transforms.size() << endl;
	    geometry_msgs::Transform gt;
	    for(int i = 0 ; i < tfData->transforms.size(); i++){
	      
	      if(tfData->transforms[i].child_frame_id == "velodyne_link"){
		cout <<tfData->transforms[i].child_frame_id << "  "<< tfData->transforms[i].header.stamp  << endl;
		
		gt = tfData->transforms[i].transform;
		cout << gt << endl;
		tftime = tfData->transforms[i].header.stamp.toSec();
		
	      }
	    }
	  
	    Eigen::Quaternion<float> q;
	    q.w() = gt.rotation.w;
	    q.x() = gt.rotation.x;
	    q.y() = gt.rotation.y;
	    q.z() = gt.rotation.z;
	    T.block<3,3>(0,0) = q.toRotationMatrix();
	    T.block<3,1>(0,3) = Eigen::Vector3f(gt.translation.x, gt.translation.y, gt.translation.z);
	    tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(T, "/map", "/velodyne_link", ros::Time::now()));
	    
	  }
	  
	  sensor_msgs::PointCloud2::Ptr lidarData = m.instantiate<sensor_msgs::PointCloud2>();
		if (lidarData != NULL){
		  if(fabs(lidarData->header.stamp.toSec() - tftime) > 0.3)
		  {
		    cout <<"XXX" <<endl;
		    continue;
		  }
		  DP newdata = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*lidarData);
		  pointPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(newdata, "/velodyne_link", ros::Time::now()));
		  cout << "lidar message " << lidarData->header.stamp << endl;
		  if(!init){
		    wholemap = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*lidarData);
		    init = true;
		  }else{
		    wholemap.concatenate(  transformation->compute(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*lidarData), T.inverse()) );
		  }
		  
		}
	  
	}
	
	
	wholemap.save(ros::package::getPath("laser_mapping")+"/wholemap.ply");
	
// 	while(ros::ok()){
// 	  pointPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(wholemap, "/map", ros::Time::now()));
// 	  r.sleep();
// 	}
	
	
}