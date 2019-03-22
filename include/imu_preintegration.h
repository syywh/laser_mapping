#ifndef IMU_PREINTEGRATION_H
#define IMU_PREINTEGRATION_H

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "fstream"
#include "IMU/imudata.h"
#include "boost/thread/future.hpp"

#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "pointmatcher_ros/transform.h"

#include "IMU/IMUPreintegrator.h"
#include "IMU/NavState.h"
// #include "thread"
// #include "preintegration_opt.h"

using namespace std;

namespace LIV{
//   class preintegration_opt;
  
  class imu_preintegration{
    
    typedef PointMatcher<float> PM;
    
  private:
    ros::NodeHandle& n;
    
    // Subscribers
    ros::Subscriber imu_sub;
    
    //Publishers
    ros::Publisher imu_pub;
    ros::Subscriber dbias_sub;
    ros::Subscriber velo_sub;
    
    //imu message 
    string imu_msg_str;
    
//     ros::AsyncSpinner mspinner;
    boost::mutex mMutexIMUPreint; 
    
    bool init;
    
    string laserFrame;  //launch中param  odom_frame
    
    string imuFrame;
    
    string mapFrame;
    
  public:
    imu_preintegration(ros::NodeHandle& nh);
    ~imu_preintegration();
    
    void Run();
    
    void resetPreintegrator();
    
//     void setPreintegration_opter(LIV::preintegration_opt* preintegration_);
    
    vill::IMUPreintegrator getEvaluatedOdomPreintegration();
    vector<vill::IMUData>& getIMUdataVector();
    vector<vill::IMUData>& getIMUdataVector(double fromtime, double totime);
    bool checkCoarseInitialized();
    void setCoarseInitialized(bool flag);
    
    static Vector3d getGravity();
    PM::TransformationParameters getInitialT();
    Vector3d getBiasa();
    Vector3d getBiasg();
    Vector3f getVel();
    void  setBiasa(Vector3d &mdbiasa);
    void  setBiasg(Vector3d &mdbiasg);
    void setBiasa_g(const Vector3d mdbiasa, const Vector3d mdbiasg);
    void setCurLaserPose(PM::TransformationParameters& T);
    PM::TransformationParameters getCurLaserPose();
    
    bool checkBiasUpdated();
    void setBiasUpdated(bool flag);

    string getIMUName(){
        return imu_msg_str;
    }

    void addIMUMsg(const  sensor_msgs::Imu &imuMsgIn);

    list<PM::TransformationParameters> vT_laser_imu;
    list<double> vT_times;
	vector<vill::NavState>  v_NavStates;
	vector<vill::IMUData> lIMUdata;
	
	void updateNavState(vill::NavState& oriNavState, vill::NavState& newNavState,const vill::IMUPreintegrator &imupreint);
    

    
  protected:
    void gotIMU(const sensor_msgs::Imu& imuMsgIn);
    void gotOptDbias(const geometry_msgs::Twist & dbiasMsgIn);
    void gotOptmveloFrame(const geometry_msgs::Vector3 & optVelMsgIn);
    
    void getGyroMsg(Eigen::Vector3d & gw, double time);
    
    
    vill::IMUPreintegrator imupreintegrator;
    tf::TransformBroadcaster tfBroadcaster;
    PM::TransformationParameters T, Tcur;
    
    static boost::mutex mMutexGravity;  
    Vector3d mGravity;
    static Vector3d gw;
    Vector2d pitch_roll; 	//initial state w.r.t the inertial coordination
    boost::mutex mMutexBias;  
    boost::mutex mMutexVel;  
    boost::mutex mMutexCurPose;  
    Vector3d biasa;
    Vector3d biasg;
    Vector3f mvelo;
    bool biasUpdated;
    Matrix3d initialrotation;
    PM::TransformationParameters Tinit;
    PM::TransformationParameters Tlb;
    

    float initial_time;
    
    fstream f;
    int initialization_cnt;
    
    bool CoarseInitialized;
    boost::mutex mMutexIMUCoarseInitialized;  
//     thread* CoarseInitializationThread;
    void CoarseInitialization();
//     Vector3d EstimateGravity();
    
    enum eIMUState{
        NOT_INITIALIZED=0,
        WORKING=1,
    };
    eIMUState mCurrentState;


    
//     LIV::preintegration_opt* mppreintegration_opter;
  };//end of imu_preintegration
  
}//end of namespace

#endif