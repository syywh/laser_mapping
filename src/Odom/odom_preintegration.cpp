#include <odom_preintegration.h>


using namespace std;

namespace LIV{
odom_preintegration::odom_preintegration(ros::NodeHandle& nh, 
				         ros::MultiThreadedSpinner& spinner):
  n(nh),
  mspinner(spinner),
  _status(NOTINIT),
  _lidarMsgDelaySec(0.)
{
  if(! ros::param::get("~odom_msg_name", odom_msg_str)){
    odom_msg_str = "/thomas_robot/odom";
  }
  cerr << "======== odom msg name: " << odom_msg_str << endl;
}

odom_preintegration::~odom_preintegration()
{

}

OdomPreintegrator odom_preintegration::getOdomPreintegration(double &lastT, double &nextT)
{
    boost::mutex::scoped_lock lock(mMutexOdomPreint);
  
    OdomPreintegrator odompreint;
    odompreint.reset();

    vector<OdomConstPtr> vOdoms;
    if(!getRecentMsgs(lastT, nextT, vOdoms))
      return odompreint;
//     cout << "here we got " << vOdoms.size() << " odoms" << endl;
    
    std::vector<OdomData> vodomData;

    for(unsigned int i=0;i<vOdoms.size();i++)
    {
	OdomConstPtr odomMsg = vOdoms[i];
	double vx = odomMsg->twist.twist.linear.x;
	double vy = odomMsg->twist.twist.linear.y;
	double vz = odomMsg->twist.twist.linear.z;

	double wx = odomMsg->twist.twist.angular.x; 
	double wy = odomMsg->twist.twist.angular.y;
	double wz = odomMsg->twist.twist.angular.z;

	OdomData odomdata(vx,vy,vz,
				      wx,wy,wz,
				      odomMsg->header.stamp.toSec());
	vodomData.push_back(odomdata);
	//ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
    }

    
//     {
//         const OdomData& odom = vodomData.front();
//         double dt = odom._t - lastT;
// 	cout << odom._v.transpose() << endl;
// 	cout << odom._w.transpose() << endl;
//         odompreint.update(odom._v, odom._w, dt);
// cerr<<"3 "<<endl;
//         // Test log
//         if(dt < 0)
//         {
//             cerr<<std::fixed<<std::setprecision(3)<<"dt = "<<dt<<", this KF vs last odom time: "<<lastT<<" vs "<<odom._t<<endl;
//             std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
//         }
//     }

    // integrate each odom
    for(size_t i=0; i<vodomData.size(); i++)
    {
        const OdomData& odom = vodomData[i];
        double nextt;
        if(i==vodomData.size()-1)
            nextt = nextT;         // last odom, next is this KeyFrame
        else
            nextt = vodomData[i+1]._t;  // regular condition, next is imu data

        // delta time
        double dt = nextt - odom._t;
        // update pre-integrator
        odompreint.update(odom._v, odom._w, dt);

        // Test log
        if(dt <= 0)
        {
            cerr<<std::fixed<<std::setprecision(3)<<"dt = "<<dt<<", this vs next time: "<<odom._t<<" vs "<<nextt<<endl;
            std::cerr.unsetf ( std::ios::showbase );                // deactivate showbase
        }
    }
    
    return odompreint;
}

void odom_preintegration::Run()
{
  odom_sub = n.subscribe(odom_msg_str.c_str(),1,&odom_preintegration::gotOdom,this);
//   ros::spin();
//   mspinner.spin();

}

bool odom_preintegration::getRecentMsgs(double t0, double t1, vector<OdomConstPtr> &vodommsgs)
{
    if(_odomMsgQueue.empty())
    {
        ROS_WARN("no odom message stored, shouldn't");
        return false;
    }
    
    vodommsgs.clear();
    while(true)
    {
	if(_odomMsgQueue.empty())
	{
            break;
	}
	
	OdomConstPtr tmpodommsg = _odomMsgQueue.front();
	
// 	ROS_WARN_STREAM("Total dtime " << t1 - t0);
// 	ROS_WARN_STREAM("Last dtime " << tmpodommsg->header.stamp.toSec() - t0 << ", cur dtime " << tmpodommsg->header.stamp.toSec() - t1);
	
	if(tmpodommsg->header.stamp.toSec() >= t0 - _lidarMsgDelaySec 
	  && tmpodommsg->header.stamp.toSec() <= t1 - _lidarMsgDelaySec)
        {
	    vodommsgs.push_back(tmpodommsg);
	    _odomMsgQueue.pop();

//             _dataUnsyncCnt = 0;
        }
        else if(tmpodommsg->header.stamp.toSec() < t0 - _lidarMsgDelaySec)
	{
	    _odomMsgQueue.pop();
	}
	else
	{
	    break;
	}

    }
    
    return true;
}

void odom_preintegration::gotOdom(const OdomConstPtr& odomMsgIn)
{
    addOdomMsg(odomMsgIn);
//     cerr <<"-" << odomMsgIn.get()->pose.pose.position << endl;
}

void odom_preintegration::clearMsgs(void)
{
    _odomMsgQueue = std::queue<OdomConstPtr>();
}

void odom_preintegration::addOdomMsg(const OdomConstPtr& odommsg)
{
    _odomMsgQueue.push(odommsg);
//     ROS_WARN_STREAM("There are " << _odomMsgQueue.size() << " odoms in the queue");
}
  
}

