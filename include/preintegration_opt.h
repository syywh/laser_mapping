#ifndef PREINTEGRATION_OPT_H
#define PREINTEGRATION_OPT_H

#include "pointmatcher/PointMatcher.h"

#include "glog/logging.h"
#include "Optimizer.h"
#include "IMU/imudata.h"
#include "IMU/IMUPreintegrator.h"
#include "imu_preintegration.h"
#include "thread"

#include "Frame.h"

using namespace std;


namespace LIV{
//   class Velodyne_SLAM::Frame;
  
  class preintegration_opt{
  typedef PointMatcher<float> PM;
  typedef PM::DataPoints DP;
  
  public:
    preintegration_opt();
    void Run();
    
    void insertFrameswithIMU(Velodyne_SLAM::Frame* pF);
    void insertFramesForInitialization(Velodyne_SLAM::Frame* pF);
//     bool isInitialized();
    
    // == initializing IMU
    void imuIniting();
    void SetIMUIniting(bool flag);
    bool GetIMUIniting();
    void SetIMUInited(bool flag);
    bool GetIMUInited();
    vector<Velodyne_SLAM::Frame* > GetFramesForInit();
    bool TryInitIMU();
    static MatrixXd TangentBasis(Vector3d &g0);
    static void RefineGravity(vector<Velodyne_SLAM::Frame*> &all_image_frame, Vector3d &g, VectorXd &x);
    static bool LinearAlignment(vector< Velodyne_SLAM::Frame* > vFrames, Vector3d & g, VectorXd &x);
     void setBiasUpdated(bool flag);
     bool checkBiasUpdated();
    Vector3d getdbiasa();
    Vector3d getdbiasg();
    Vector3d getVelo();
    
    void setimu_preintegration(LIV::imu_preintegration* pimu_preintegrator);
    
    
    ros::Publisher vel_pub;
    ros::Publisher dbias_pub;
    
//     static boost::mutex mMutexGravity;
//     static void setGravity(Vector3d& gravity);
//     static Vector3d getGravity();
    
  private:
    ros::NodeHandle n;
    boost::mutex mMutexProcessFrames;  
    vector<Velodyne_SLAM::Frame* > vFrames;
    
    boost::mutex mMutexFramesForInit;
    vector<Velodyne_SLAM::Frame*> vFramesForInit;
    Velodyne_SLAM::Frame* mCurrentFrame = nullptr;
    bool checkNewFrames();
    bool processFrames(vector<Velodyne_SLAM::Frame* >& pFs);
//     bool initialized;
    
    //Thread for Inertial initialization
    std::thread *mptIMUInitial =nullptr;
    
    Vector3d mGravityVec;
    
    boost::mutex mMutexIMUIniting;
    boost::mutex mMutexIMUInited;
    bool mbIMUIniting, mbIMUInited;
    
     boost::mutex mMutexBiasUpdated;
    bool bBiasUpdated;
    Vector3d dbiasa, dbiasg, velo;
    
    int window_width, delta;
    
    LIV::imu_preintegration* mimu_preintegrator;
    
  };
}


#endif
