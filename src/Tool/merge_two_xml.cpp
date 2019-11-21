#include "ros/ros.h"
#include<iostream>
#include<fstream>
#include "boost/property_tree/xml_parser.hpp"
#include<ros/package.h>

#include "pointmatcher_ros/point_cloud.h"

using namespace std;

typedef PointMatcher<float> PM;

int main(int argc, char **argv)
{
	//add from the end of xml1 to the beginning of xml2
	ros::init(argc, argv, "merge_two_xml");
	ros::start();
	
	ros::NodeHandle n;
	
	string base1, xml1, base2, xml2, base_merge,merged_xml,relative_file,ori_of_2;
	ros::param::get("~xml1", base1);
	xml1 = base1 + "/map.xml";
	ros::param::get("~xml2", base2);
	xml2 = base2 + "/map.xml";
	ros::param::get("~merged_xml", base_merge);
	merged_xml = base_merge + "/map.xml";
	ros::param::get("~relative_file", relative_file);
	ros::param::get("~ori_of_2", ori_of_2);
	
	PM::TransformationParameters relativeT = PM::TransformationParameters::Identity(4,4);
	PM::TransformationParameters oriof2T = PM::TransformationParameters::Identity(4,4);
	
	ifstream frelative_file(relative_file);
	ifstream fori_of_2(ori_of_2);
	
	for(int i = 0 ; i < 4; i++){
		for(int j = 0; j < 4; j++){
			frelative_file >> relativeT(i,j);
			fori_of_2 >> oriof2T(i,j);
		}
	}
	
	cout <<"relativeT\n" << relativeT<<endl;
	cout <<"ori2T\n" << oriof2T<<endl;
	
	Eigen::Quaternionf rela_q(relativeT.block<3,3>(0,0));
	Eigen::Vector3f rela_t(relativeT.block<3,1>(0,3));
	
	Eigen::Quaternionf rela_q_inv(relativeT.inverse().block<3,3>(0,0));
	Eigen::Vector3f rela_t_inv(relativeT.inverse().block<3,1>(0,3));
	
	cout <<"load from xml1 " << xml1 << endl;
	cout <<"load from xml2 " << xml2 << endl;
	cout <<"merge into xml " << merged_xml << endl;
	
	boost::property_tree::ptree pxml1, pxml2;
	boost::property_tree::read_xml(xml1, pxml1);
	boost::property_tree::read_xml(xml2, pxml2);
	boost::property_tree::ptree p_map_merge;
	
	int num_xml1 = pxml1.get<int>("map.property.node_count");
	cout << "num of keyframe in map1 " << num_xml1 << endl;
	
	int num_xml2 = pxml2.get<int>("map.property.node_count");
	cout << "num of keyframe in map2 " << num_xml2 << endl;
	
	p_map_merge.put("map.property.node_count", (num_xml1+num_xml2));
	boost::property_tree::ptree p_node_list_merge;
	
	boost::property_tree::ptree pMap1 = pxml1.get_child("map.node_list");
	for (auto p_node_it = pMap1.begin(); p_node_it != pMap1.end(); ++p_node_it)
	{
		boost::property_tree::ptree p_node_merge;//merge
		
		int node_id = p_node_it->second.get<int>("id");
		p_node_merge.put("id", node_id);
		
		boost::property_tree::ptree p_neighbour_list_merge; // merge
		boost::property_tree::ptree p_neighbour_list = p_node_it->second.get_child("neighbour_list");
		for (auto p_neighbour_it = p_neighbour_list.begin(); p_neighbour_it != p_neighbour_list.end(); ++p_neighbour_it)
		{

			int neighbour_id = p_neighbour_it->second.get<int>("id");
			std::string transform_str = p_neighbour_it->second.get<std::string>("transform");
			bool reachable = p_neighbour_it->second.get<bool>("reachable");
			
			boost::property_tree::ptree p_neighbour_merge;
			p_neighbour_merge.put("id", neighbour_id);
			p_neighbour_merge.put("transform", transform_str);
			p_neighbour_merge.put("reachable", true);
			
			p_neighbour_list_merge.add_child("neighbour", p_neighbour_merge);
		}
		
		//add connect between two map
		if(node_id == num_xml1-1){
			boost::property_tree::ptree p_neighbour_merge;
			std::stringstream ss;
			ss << rela_t[0] << " " << rela_t[1] << " " << rela_t[2] << " " << rela_q.w() 
			<< " " << rela_q.x() << " " << rela_q.y() << " " << rela_q.z();
			p_neighbour_merge.put("id", num_xml1);
			p_neighbour_merge.put("transform", ss.str());
			p_neighbour_merge.put("reachable", true);
			
			p_neighbour_list_merge.add_child("neighbour", p_neighbour_merge);
		}
		
		p_node_merge.add_child("neighbour_list", p_neighbour_list_merge);
		p_node_list_merge.add_child("node", p_node_merge);
	}
	
	
	boost::property_tree::ptree pMap2 = pxml2.get_child("map.node_list");
	for (auto p_node_it = pMap2.begin(); p_node_it != pMap2.end(); ++p_node_it)
	{
		boost::property_tree::ptree p_node_merge;//merge
		
		int node_id = p_node_it->second.get<int>("id");
		p_node_merge.put("id", node_id+num_xml1);
		
		boost::property_tree::ptree p_neighbour_list_merge; // merge
		boost::property_tree::ptree p_neighbour_list = p_node_it->second.get_child("neighbour_list");
		for (auto p_neighbour_it = p_neighbour_list.begin(); p_neighbour_it != p_neighbour_list.end(); ++p_neighbour_it)
		{

			int neighbour_id = p_neighbour_it->second.get<int>("id");
			std::string transform_str = p_neighbour_it->second.get<std::string>("transform");
			bool reachable = p_neighbour_it->second.get<bool>("reachable");
			
			boost::property_tree::ptree p_neighbour_merge;
			p_neighbour_merge.put("id", neighbour_id+num_xml1);
			p_neighbour_merge.put("transform", transform_str);
			p_neighbour_merge.put("reachable", true);
			
			p_neighbour_list_merge.add_child("neighbour", p_neighbour_merge);
		}
		
		//add connect between two map
		if(node_id == 0){
			boost::property_tree::ptree p_neighbour_merge;
			std::stringstream ss;
			ss << rela_t_inv[0] << " " << rela_t_inv[1] << " " << rela_t_inv[2] << " " << rela_q_inv.w() 
			<< " " << rela_q_inv.x() << " " << rela_q_inv.y() << " " << rela_q_inv.z();
			p_neighbour_merge.put("id", num_xml1-1);
			p_neighbour_merge.put("transform", ss.str());
			p_neighbour_merge.put("reachable", true);
			
			p_neighbour_list_merge.add_child("neighbour", p_neighbour_merge);
		}
		
		p_node_merge.add_child("neighbour_list", p_neighbour_list_merge);
		p_node_list_merge.add_child("node", p_node_merge);
		
		//compute global pose
		ifstream fpose(base2+"/frames/"+to_string(node_id)+"/GlobalPose.txt");
		PM::TransformationParameters oriT = PM::TransformationParameters::Identity(4,4);
		for(int i = 0; i < 4; i++){
			for(int j= 0; j < 4; j++){
				fpose >> oriT(i,j);
			}
		}
		oriT = oriof2T * oriT;
		ofstream fpose_merge(base_merge+"/frames/"+to_string(node_id+num_xml1)+"/GlobalPose.txt");
		fpose_merge << oriT ;
		fpose.close();
		fpose_merge.close();
	}
	p_map_merge.add_child("map.node_list", p_node_list_merge);
	boost::property_tree::xml_writer_settings<string> setting(' ', 2);
    boost::property_tree::write_xml(merged_xml, p_map_merge, std::locale(), setting);
	
}
