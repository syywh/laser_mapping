#include "Frame.h"
#include <boost/graph/graph_concepts.hpp>

using namespace std;
using namespace PointMatcherSupport;

namespace Velodyne_SLAM {
	
long unsigned int Frame::nNextId=0;

Frame::Frame()
{}

Frame::Frame(const Frame& frame):mstamp(frame.mstamp),mFrameMapPoints(frame.mFrameMapPoints),mLocalTicp(frame.mLocalTicp),
mnId(frame.mnId),processed(false)
{
}


Frame::Frame(const ros::Time& stamp, Frame::DP* frameMapPoints, const PM::TransformationParameters pTicp, bool iskeyframe):mstamp(stamp), 
mFrameMapPoints(frameMapPoints)
, mLocalTicp(pTicp), isKeyFrame(iskeyframe),processed(false)
{
	mnId=nNextId++;
}

bool Frame::IsKF()
{
	return isKeyFrame;
}

bool Frame::SetKF()
{
	isKeyFrame = true;
}

void Frame::SetTicp(PointMatcher< float >::TransformationParameters pTicp)
{
  mTicp = pTicp;
}

PointMatcher< float >::TransformationParameters Frame::GetGlobalPose()
{
  return mTicp;
}

void Frame::SetIMUPreint(vill::IMUPreintegrator& pIMUpreint)
{
    boost::mutex::scoped_lock lock(mMutexImuPreint);
    mIMUPreint = pIMUpreint;

}


vill::IMUPreintegrator Frame::GetIMUPreint()
{
  boost::mutex::scoped_lock lock(mMutexImuPreint);
  return mIMUPreint;

}


void Frame::SetRelatedKFId(long unsigned int pRelatedKFId)
{
	mnRelatedKFId = pRelatedKFId;
}

void Frame::setGPS(GPS pgps)
{
	gps = pgps;

}

void Frame::SetIMUdata(vector< vill::IMUData >& mIMUdata)
{
    mvIMUdatas.insert(mvIMUdatas.begin(), mIMUdata.begin(), mIMUdata.end());
}

void Frame::ComputePreInt(Vector3d& bg)
{
	if(previousF == nullptr)
	  return;
	
	cout<<"previous: "<<previousF->mnId<<" current: "<<mnId<<endl;
	
	if(mvIMUdatas.empty())
	  return;
	
	mIMUPreint.reset();
	
	int beginid = 0;
	while(mvIMUdatas[beginid]._t<previousF->mstamp.toSec())
	  beginid++;
  
	{
	    const vill::IMUData &imu = mvIMUdatas[beginid];
	    double dt = std::max(0., imu._t - previousF->mstamp.toSec());
	    mIMUPreint.update(imu._g - bg, imu._a, dt);  // Acc bias not considered here
	}
	// integrate each imu
	for (size_t i = beginid; i < mvIMUdatas.size(); i++) {
	    const vill::IMUData &imu = mvIMUdatas[i];
	    double nextt;
	    if (i == mvIMUdatas.size() - 1)
		nextt = mstamp.toSec();         // last IMU, next is this KeyFrame
	    else
		nextt = mvIMUdatas[i + 1]._t;  // regular condition, next is imu data

	    // delta time
	    double dt = std::max(0., nextt - imu._t);
	    // update pre-integrator
	    mIMUPreint.update(imu._g - bg, imu._a, dt);
	}
    
}

void Frame::ComputePreInt()
{
	if(previousF_KF == nullptr){
	  cout<<"no previous frame of current frame "<<mnId<<endl;
	  return;
	}
	
	cout<<"previous: "<<previousF_KF->mnId<<" @ "<<setprecision(15)<<previousF_KF->mstamp.toSec()
	<<" current: "<<mnId<<setprecision(15)<<" @ "<<mstamp.toSec()<<endl;
	
	if(mvIMUdatas.empty())
	  return;
	
	mIMUPreint.reset();
	
	int beginid = 0;
	while(mvIMUdatas[beginid]._t<previousF_KF->mstamp.toSec()){
// 	  cout<<"--"<<setprecision(15)<<mvIMUdatas[beginid]._t<<endl;
	  beginid++;
	}
	cout<<"begin id "<<beginid<<endl;
	{
	    const vill::IMUData &imu = mvIMUdatas[beginid];
	    double dt = std::max(0., imu._t - previousF_KF->mstamp.toSec());
	    mIMUPreint.update(imu._g - mNavState.Get_BiasGyr(), imu._a-mNavState.Get_BiasAcc(), dt);  // Acc bias not considered here
	}
// 	cout<<setprecision(15)<<mvIMUdatas[beginid]._t<<" "<<mvIMUdatas.size()<<endl;
	// integrate each imu
	for (size_t i = beginid; i < mvIMUdatas.size(); i++) {
	  
	    const vill::IMUData &imu = mvIMUdatas[i];
// 	    cout<<setprecision(15)<<mvIMUdatas[i]._t<<endl;
	    if(imu._t > mstamp.toSec()){
	      cout<<"totoal process "<<(i-beginid)<<endl;
	      break;
	    }
	    
	    double nextt;
	    if (i == mvIMUdatas.size() - 1)
		nextt = mstamp.toSec();         // last IMU, next is this KeyFrame
	    else
		nextt = mvIMUdatas[i + 1]._t;  // regular condition, next is imu data

	    // delta time
	    double dt = std::max(0., nextt - imu._t);
	    // update pre-integrator
	    mIMUPreint.update(imu._g - mNavState.Get_BiasGyr(), imu._a-mNavState.Get_BiasAcc(), dt);
	}
    
}


    const vill::NavState & Frame::GetNavState(void) {
        boost::mutex::scoped_lock lock(mMutexNavState);
        return mNavState;
    }


void Frame::UpdateNavStatePVRFromTwc(const SE3d &Twc, const SE3d &Tbc) {
  
	SE3d Twb = ( Twc * Tbc.inverse() );
        Matrix3d Rwb = Twb.rotationMatrix();
        Vector3d Pwb = Twb.translation();
	
	{
		boost::mutex::scoped_lock lock(mMutexNavState);
		Matrix3d Rw1 = mNavState.Get_RotMatrix();
		Vector3d Vw1 = mNavState.Get_V();
		Vector3d Vw2 = Rwb * Rw1.transpose() * Vw1;   // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

		mNavState.Set_Pos(Pwb);
		mNavState.Set_Rot(Rwb);
		mNavState.Set_Vel(Vw2);
	}
    }
    
void Frame::SetNavStatePos(const Vector3d& pos)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_Pos(pos);
}

void Frame::SetNavStateVel(const Vector3d& vel)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_Vel(vel);

}

void Frame::SetNavStateRot(const Sophus::SO3d& rot)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_Rot(rot);
}

void Frame::SetNavStateBiasAcc(const Vector3d& ba)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_BiasAcc(ba);

}

void Frame::SetNavStateBiasGyr(const Vector3d& bg)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_BiasGyr(bg);
}

void Frame::SetNavStateDeltaBa(const Vector3d& dba)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_DeltaBiasAcc(dba);
}

void Frame::SetNavStateDeltaBg(const Vector3d& dbg)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_DeltaBiasGyr(dbg);
}

void Frame::SetNavStateRot(const Matrix3d& rot)
{
  boost::mutex::scoped_lock lock(mMutexNavState);
  mNavState.Set_Rot(rot);
}


void Frame::UpdatePoseFromNS(const SE3d& Tbc)
{
        Matrix3d Rbc_ = Tbc.rotationMatrix().cast<double>();
        Vector3d Pbc_ = Tbc.translation();

        Matrix3d Rwb_ = mNavState.Get_RotMatrix();
        Vector3d Pwb_ = mNavState.Get_P();

        Matrix3d Rcw_ = (Rwb_ * Rbc_).transpose();
        Vector3d Pwc_ = Rwb_ * Pbc_ + Pwb_;
        Vector3d Pcw_ = -Rcw_ * Pwc_;

        SE3d Tcw_ = SE3d(Rcw_, Pcw_).inverse();
	
	PM::TransformationParameters tempT = mTicp;
	tempT.block<3,3>(0,0) = Tcw_.rotationMatrix().cast<float>();
	tempT.block<3,1>(0,3) = Tcw_.translation().cast<float>();

        SetTicp(tempT);
}


bool Frame::isProcessed()
{
  boost::mutex::scoped_lock lock(mMutexProcessed);
  return processed;

}

void Frame::setProcessed(bool flag)
{
  boost::mutex::scoped_lock lock(mMutexProcessed);
  processed = flag;

}


void Frame::SetInitialNavStateAndBias(const vill::NavState &ns) {
    boost::mutex::scoped_lock lock(mMutexProcessed);
    mNavState = ns;
    // Set bias as bias+delta_bias, and reset the delta_bias term
    mNavState.Set_BiasGyr(ns.Get_BiasGyr() + ns.Get_dBias_Gyr());
    mNavState.Set_BiasAcc(ns.Get_BiasAcc() + ns.Get_dBias_Acc());
    mNavState.Set_DeltaBiasGyr(Vector3d::Zero());
    mNavState.Set_DeltaBiasAcc(Vector3d::Zero());
}

void Frame::UpdateNavState(const vill::IMUPreintegrator& imupreint, const Vector3d& gw)
{
        boost::mutex::scoped_lock lock(mMutexNavState);

        Matrix3d dR = imupreint.getDeltaR();
        Vector3d dP = imupreint.getDeltaP();
        Vector3d dV = imupreint.getDeltaV();
        double dt = imupreint.getDeltaTime();

        Vector3d Pwbpre = mNavState.Get_P();
        Matrix3d Rwbpre = mNavState.Get_RotMatrix();
        Vector3d Vwbpre = mNavState.Get_V();

        Matrix3d Rwb = Rwbpre * dR;
        Vector3d Pwb = Pwbpre + Vwbpre * dt + 0.5 * gw * dt * dt + Rwbpre * dP;
        Vector3d Vwb = Vwbpre + gw * dt + Rwbpre * dV;
	cout<<"update navstate "<<(gw*dt).transpose()<<" "<<(Rwbpre * dV).transpose()<<std::endl;

        // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
        mNavState.Set_Pos(Pwb);
        mNavState.Set_Vel(Vwb);
        mNavState.Set_Rot(Rwb);
}

}//end of namespace