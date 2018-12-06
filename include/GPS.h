#ifndef GPS_H
#define GPS_H

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <iomanip>
#include <boost/graph/graph_concepts.hpp>
#include <math.h>
#include <ros/time.h>

namespace Velodyne_SLAM{
	
struct GPS{
	double  longitude;	//经度
	double latitude;	//纬度
	
	double northing;
	double easting;
	double altitude;
	
	double roll;
	double pitch;
	double yaw;
	
	int status;
	int satellites_used;
	ros::Time timestamp;
	
};

struct geoXYZ{
	double  x;	
	double  y;	
	double  z;
	double northing;
	double easting;
	double altitude;
};

	
class CoorConvert{

public:
	bool outOfChina(double lat, double lon);
	double transformLat(double x, double y);
	double transformLon(double x, double y);
	static void transformGPStoGeoXYZ(GPS gps, geoXYZ& geo);
	
	static double latToScale(double lat);
	static void latlonToMercator(GPS gps, geoXYZ& geo);
	
    /**
     * 地球坐标转换为火星坐标
     * World Geodetic System ==> Mars Geodetic System
     *
     * @param wgLat  地球坐标
     * @param wgLon
     *
     * mglat,mglon 火星坐标
     */
     void transform2Mars(double wgLat, double wgLon,double &mgLat,double &mgLon);
 	/**
     * 火星坐标 转换为 地球坐标
     * World Geodetic System ==> Mars Geodetic System
     *
     * @param wgLat  地球坐标
     * @param wgLon
     *
     * mglat,mglon 火星坐标
     */
	void mars2Real(double mgLat, double mgLon, double& wgLat, double& wgLon);
    /**
     * 火星坐标转换为百度坐标
     * @param gg_lat
     * @param gg_lon
     */
     void bd_encrypt(double gg_lat, double gg_lon,double &bd_lat,double & bd_lon);
     
    /**
     * 百度转火星
     * @param bd_lat
     * @param bd_lon
     */
     void bd_decrypt(double bd_lat, double bd_lon,double &gg_lat,double &gg_lon);

};


	
}


#endif
