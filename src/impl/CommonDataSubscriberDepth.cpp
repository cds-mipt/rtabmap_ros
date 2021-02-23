/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap_ros/CommonDataSubscriber.h>

namespace rtabmap_ros {

// RGB + Depth
void CommonDataSubscriber::depthCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScanDescCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan2dInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScan3dInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthScanDescInfoCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	nav_msgs::OdometryConstPtr odomMsg; // Null
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom
void CommonDataSubscriber::depthOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
/*
	static const char* file_name = "/home/cds-jetson-host/rtabmap/latency_error.txt";
	static const int buffer_size = 100;
	static int counter = 0;
	static float imageError_msec[buffer_size];
	static float depthError_msec[buffer_size];
	static float cameraInfoError_msec[buffer_size];
	static float scanError_msec[buffer_size];

	ros::Time odomTime = odomMsg->header.stamp;
	ros::Time imageTime = imageMsg->header.stamp;
	ros::Time depthTime = depthMsg->header.stamp;
	ros::Time cameraInfoTime = cameraInfoMsg->header.stamp;
	ros::Time scanTime = scanMsg->header.stamp;
	
	ros::Duration imageError = imageTime - odomTime;
	ros::Duration depthError = depthTime - odomTime;
	ros::Duration cameraInfoError = cameraInfoTime - odomTime;
	ros::Duration scanError = scanTime - odomTime;
	
	imageError_msec[counter] = imageError.toSec() * 1000;
	depthError_msec[counter] = depthError.toSec() * 1000;
	cameraInfoError_msec[counter] = cameraInfoError.toSec() * 1000;
	scanError_msec[counter] = scanError.toSec() * 1000;
	
	counter++;
	if (counter == buffer_size)
	{
		std::fstream fs;
		fs.open(file_name, std::fstream::app);
		for (int i = 0; i < buffer_size; i++)
		{
			fs << imageError_msec[i] << ' ' << depthError_msec[i] << ' ' << cameraInfoError_msec[i] << ' ' << scanError_msec[i] << std::endl;
		}
		fs.close();
		counter = 0;
	}
*/

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomScanDescInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

#ifdef RTABMAP_SYNC_USER_DATA
// RGB + Depth + User Data
void CommonDataSubscriber::depthDataCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScanDescCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthDataInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan2dInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScan3dInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthDataScanDescInfoCallback(
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	nav_msgs::OdometryConstPtr odomMsg; // null
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}

// RGB + Depth + Odom + User Data
void CommonDataSubscriber::depthOdomDataCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScanDescCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg)
{
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
void CommonDataSubscriber::depthOdomDataInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan2dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, *scanMsg, scan3dMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScan3dInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	sensor_msgs::LaserScan scan2dMsg; // Null
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scan2dMsg, *scanMsg, odomInfoMsg);
}
void CommonDataSubscriber::depthOdomDataScanDescInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::UserDataConstPtr & userDataMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const rtabmap_ros::ScanDescriptorConstPtr& scanMsg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptor;
	if(!scanMsg->global_descriptor.data.empty())
	{
		globalDescriptor.push_back(scanMsg->global_descriptor);
	}
	commonSingleDepthCallback(odomMsg, userDataMsg, cv_bridge::toCvShare(imageMsg), cv_bridge::toCvShare(depthMsg), *cameraInfoMsg, *cameraInfoMsg, scanMsg->scan, scanMsg->scan_cloud, odomInfoMsg, globalDescriptor);
}
#endif

void CommonDataSubscriber::setupDepthCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeUserData,
		bool subscribeScan2d,
		bool subscribeScan3d,
		bool subscribeScanDesc,
		bool subscribeOdomInfo,
		int queueSize,
		bool approxSync)
{
	ROS_INFO("Setup depth callback");

	std::string rgbPrefix = "rgb";
	std::string depthPrefix = "depth";
	ros::NodeHandle rgb_nh(nh, rgbPrefix);
	ros::NodeHandle depth_nh(nh, depthPrefix);
	ros::NodeHandle rgb_pnh(pnh, rgbPrefix);
	ros::NodeHandle depth_pnh(pnh, depthPrefix);
	image_transport::ImageTransport rgb_it(rgb_nh);
	image_transport::ImageTransport depth_it(depth_nh);
	image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
	image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

	imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), queueSize, hintsRgb);
	imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), queueSize, hintsDepth);
	cameraInfoSub_.subscribe(rgb_nh, "camera_info", queueSize);

#ifdef RTABMAP_SYNC_USER_DATA
	if(subscribeOdom && subscribeUserData)
	{
		odomSub_.subscribe(nh, "odom", queueSize);
		userDataSub_.subscribe(nh, "user_data", queueSize);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL7(depthOdomDataScanDescInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(depthOdomDataScanDesc, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL7(depthOdomDataScan2dInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(depthOdomDataScan2d, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL7(depthOdomDataScan3dInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL6(depthOdomDataScan3d, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", queueSize);
			SYNC_DECL6(depthOdomDataInfo, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL5(depthOdomData, approxSync, queueSize, odomSub_, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
	else 
#endif
	if(subscribeOdom)
	{
		odomSub_.subscribe(nh, "odom", queueSize);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL6(depthOdomScanDescInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthOdomScanDesc, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL6(depthOdomScan2dInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthOdomScan2d, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL6(depthOdomScan3dInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthOdomScan3d, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", queueSize);
			SYNC_DECL5(depthOdomInfo, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(depthOdom, approxSync, queueSize, odomSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
#ifdef RTABMAP_SYNC_USER_DATA
	else if(subscribeUserData)
	{
		userDataSub_.subscribe(nh, "user_data", queueSize);

		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", queueSize);

			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL6(depthDataScanDescInfo, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthDataScanDesc, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", queueSize);

			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL6(depthDataScan2dInfo, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthDataScan2d, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL6(depthDataScan3dInfo, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL5(depthDataScan3d, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", queueSize);
			SYNC_DECL5(depthDataInfo, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL4(depthData, approxSync, queueSize, userDataSub_, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
#endif
	else
	{
		if(subscribeScanDesc)
		{
			subscribedToScanDescriptor_ = true;
			scanDescSub_.subscribe(nh, "scan_descriptor", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL5(depthScanDescInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(depthScanDesc, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scanDescSub_);
			}
		}
		else if(subscribeScan2d)
		{
			subscribedToScan2d_ = true;
			scanSub_.subscribe(nh, "scan", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL5(depthScan2dInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(depthScan2d, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			}
		}
		else if(subscribeScan3d)
		{
			subscribedToScan3d_ = true;
			scan3dSub_.subscribe(nh, "scan_cloud", queueSize);
			if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", queueSize);
				SYNC_DECL5(depthScan3dInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_, odomInfoSub_);
			}
			else
			{
				SYNC_DECL4(depthScan3d, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, scan3dSub_);
			}
		}
		else if(subscribeOdomInfo)
		{
			subscribedToOdomInfo_ = true;
			odomInfoSub_.subscribe(nh, "odom_info", queueSize);
			SYNC_DECL4(depthInfo, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_, odomInfoSub_);
		}
		else
		{
			SYNC_DECL3(depth, approxSync, queueSize, imageSub_, imageDepthSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_ros */
