#ifndef ODOM_PREINTEGRATION_H
#define ODOM_PREINTEGRATION_H

#include <ros/ros.h>
// #include "sensor_msgs/Imu.h"
#include "fstream"
#include "boost/thread/future.hpp"

//Doom
#include <queue>
#include <nav_msgs/Odometry.h>
#include "Odom/odomdata.h"
#include "Odom/OdomPreintegrator.h"
#include "imu_preintegration.h"

using namespace std;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

namespace LIV{
  
  class odom_preintegration{
  
  //Doom
  enum Status{
      NOTINIT = 0,
      INIT,
      NORMAL
  };
  
  private:
    //Doom
    //add odom messages into queue
    void addOdomMsg(const OdomConstPtr &odommsg);
    //clear queue
    void clearMsgs(void);
    //got msgs in [t0, t1]
    bool getRecentMsgs(double t0, double t1, vector< OdomConstPtr >& vodommsgs);
  
  public:
    //Doom
    //get odom preintegration for [lastT, nextT]
    OdomPreintegrator getOdomPreintegration(double& lastT, double& nextT);

    string getOdomName(){
      return odom_msg_str;
    }
    
  private:
    //DingDing
    ros::NodeHandle& n;
    // Subscribers
    ros::Subscriber odom_sub;
    //Publishers
    ros::Publisher odom_pub;
    //odom message 
    string odom_msg_str;
//     ros::AsyncSpinner mspinner;
    OdomPreintegrator _odompreint;
    double _lastTime;
    OdomData _odomdata;
    bool init;
    
    //Doom
    std::queue<OdomConstPtr> _odomMsgQueue;
    ros::Time _odomMsgTimeStart;
    Status _status;
    double _lidarMsgDelaySec;
    int _dataUnsyncCnt;
    
    boost::mutex mMutexOdomPreint; 
    
    LIV::imu_preintegration* mimu_preintegration;
    
    bool buse_vel_Twist_msg;
  public:
    //DingDing
    odom_preintegration(ros::NodeHandle& nh);
    ~odom_preintegration();
    
    void Run();
    
    void resetPreintegrator();
    OdomPreintegrator getEvaluatedOdomPreintegration();
    void setImuPre(LIV::imu_preintegration* pimuPre);
    
  protected:
    //DingDing
    void gotOdom(const OdomConstPtr& odomMsgIn);
    void gotVelo(const geometry_msgs::Twist& twistMsgIn);
    
    
    
  };//end of odom_preintegration
  
}//end of namespace

#endif