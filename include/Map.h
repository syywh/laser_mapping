#ifndef MAP_H
#define MAP_H

#include "Frame.h"
#include "KeyFrame.h"
#include "boost/thread.hpp"
#include "fstream"

#include<Eigen/Dense>

#include "vtr/congfig.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace Velodyne_SLAM{

class Frame;
class KeyFrame;
class Map
{
public:
	typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	Map();
	
// 	void AddFrame(Frame* pFrame);
	
	void AddKeyFrame(KeyFrame* pKeyFrame);//new 一个referenceMapoints
	void clear();
	bool isMapUpdated();
	void ResetUpdated();
	
	void getWholeMapPoints(DP& tDP);
	KeyFrame* getLatestKeyFrame();
	int getKeyFrameNum();
	void  getAllKeyFrames(std::vector<KeyFrame*>& pv);
	KeyFrame* getKeyFrame(long unsigned int id);
	
	void updateMap(std::vector<KeyFrame*> pv);
	
	unsigned int mnMaxKFid;
	DP* mWholeMapPoints;//add Local Map Points in World
	DP* mReferenceMapPoints;
	
	unique_ptr<PM::Transformation> transformation;
	ofstream fMap;
// 	vector<vector<PM::TransformationParameters> > mFramePose_Record;
	vector<vector<Frame*> > mFrame_Record;//TEST for record poses for mengmeng
	 int max_KF_ID;
	
	void saveMap(bool saveDP);
	void save(const std::string& filename);
	void savewithGlobalPose(const std::string& filename);
	void saveWholeDP(string name);
	bool load(const std::string& filename);
// 	bool load(const std::string& filename, long unsigned int pfixedNode);
	bool loadwithoutDP(const std::string& filename);
	bool LoadfromXML(const std::string& filename);
	bool loadDPforInit(int id);
	void filterMapPoints();
	void setBeginID(int num);
	void showNodesPose();
	bool linkNodes();
	
	
	
protected:
	void AddMapPoints();//change the map points when the keyframe is inserted, be careful of the dead lock!
	void AddKeyFrameWithoutAddMapPoints(KeyFrame* pKeyFrame);
	void computeTransformation(set<int >& nodes_waiting_for_T, long unsigned int fixedFirstId);
	
	std::vector<Frame*> mspFrames;			//全部的Frame
	
// 	std::set<DP*> mReferenceMapPoints;	//???我觉得还是维护Frame好一点
	std::vector<Frame* > mReferenceFrames;	//局部Frame，用以ICP
	std::vector< KeyFrame* > mspKeyFrames;	
	std::vector< KeyFrame* > vKeyFrames;	
// 	std::map< long unsigned int, KeyFrame* > mKeyFrames;
	
	
	KeyFrame* mCurrentKF;
	std::map<long unsigned int , long unsigned int > mmnId_to_vPointer;	//a map connecting the mnId and the pointer to the KeyFrame in mspKeyFrames
	ros::Publisher wholePosePublisher;
	
	boost::mutex mMutexMap;//only one lock, the whole map is changed together(KeyFrame and MapPoints)
	
// 	unsigned int mnMaxKFid;
	bool mbMapUpdated;
	PM::DataPointsFilters mapPostFilters;
	PM::DataPointsFilters mapPostFilters_Visual;
	string filename ;//for log
	int beginID;
	int fixedNode;
	float static_th_for_keeping;
	ros::NodeHandle nh;
};

	
}

#endif
