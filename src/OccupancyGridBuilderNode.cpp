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

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include "rtabmap_ros/MsgConversion.h"

#include <memory>

using namespace rtabmap;

class OccupancyGridBuilder {
public:
	OccupancyGridBuilder(int & argc, char** argv)
	{
		ros::NodeHandle pnh("~");

		std::string configPath;
		pnh.param("config_path", configPath, configPath);

		//parameters
		rtabmap::ParametersMap parameters;
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
		uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Mem"));

		if(!configPath.empty())
		{
			if(UFile::exists(configPath.c_str()))
			{
				ROS_INFO( "%s: Loading parameters from %s", ros::this_node::getName().c_str(), configPath.c_str());
				rtabmap::ParametersMap allParameters;
				Parameters::readINI(configPath.c_str(), allParameters);
				// only update odometry parameters
				for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
				{
					ParametersMap::iterator jter = allParameters.find(iter->first);
					if(jter!=allParameters.end())
					{
						iter->second = jter->second;
					}
				}
			}
			else
			{
				ROS_ERROR( "Config file \"%s\" not found!", configPath.c_str());
			}
		}

		for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			std::string vStr;
			bool vBool;
			int vInt;
			double vDouble;
			if(pnh.getParam(iter->first, vStr))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), vStr.c_str());
				iter->second = vStr;
			}
			else if(pnh.getParam(iter->first, vBool))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uBool2Str(vBool).c_str());
				iter->second = uBool2Str(vBool);
			}
			else if(pnh.getParam(iter->first, vDouble))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uNumber2Str(vDouble).c_str());
				iter->second = uNumber2Str(vDouble);
			}
			else if(pnh.getParam(iter->first, vInt))
			{
				ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uNumber2Str(vInt).c_str());
				iter->second = uNumber2Str(vInt);
			}

			if(iter->first.compare(Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
			{
				ROS_WARN( "Parameter min_inliers must be >= 8, setting to 8...");
				iter->second = uNumber2Str(8);
			}
		}

		rtabmap::ParametersMap argParameters = rtabmap::Parameters::parseArguments(argc, argv);
		for(rtabmap::ParametersMap::iterator iter=argParameters.begin(); iter!=argParameters.end(); ++iter)
		{
			rtabmap::ParametersMap::iterator jter = parameters.find(iter->first);
			if(jter!=parameters.end())
			{
				ROS_INFO( "Update %s parameter \"%s\"=\"%s\" from arguments", ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.c_str());
				jter->second = iter->second;
			}
		}

		// Backward compatibility
		for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
			iter!=Parameters::getRemovedParameters().end();
			++iter)
		{
			std::string vStr;
			if(pnh.getParam(iter->first, vStr))
			{
				if(iter->second.first && parameters.find(iter->second.second) != parameters.end())
				{
					// can be migrated
					parameters.at(iter->second.second) = vStr;
					ROS_WARN( "%s: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
							ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
				}
				else
				{
					if(iter->second.second.empty())
					{
						ROS_ERROR( "%s: Parameter \"%s\" doesn't exist anymore!",
								ros::this_node::getName().c_str(), iter->first.c_str());
					}
					else
					{
						ROS_ERROR( "%s: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
								ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str());
					}
				}
			}
		}

		occupancyGrid_.parseParameters(parameters);
		Parameters::parse(parameters, Parameters::kMemLaserScanDownsampleStepSize(), laserScanDownsampleStepSize_);
		Parameters::parse(parameters, Parameters::kMemLaserScanVoxelSize(), laserScanVoxelSize_);
		Parameters::parse(parameters, Parameters::kMemLaserScanNormalK(), laserScanNormalK_);
		Parameters::parse(parameters, Parameters::kMemLaserScanNormalRadius(), laserScanNormalRadius_);

		odomSub_ = std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, "odom", 10);
		pointCloudSub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, "scan_cloud", 10);
		sync_ = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), *odomSub_, *pointCloudSub_);
		sync_->registerCallback(boost::bind(&OccupancyGridBuilder::callback, this, _1, _2));
		occupancyGridPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
		
		/*
		pointCloudSub_ = nh_.subscribe("scan_cloud", 10, callback);
		*/
	}

	~OccupancyGridBuilder() {}
	
	/*
	void callback(const sensor_msgs::PointCloud2ConstPtr& pointCloud) {
		UDEBUG("\n\nReceived new data");
		rtabmap::Transform pose;
		try {
			tfListener_.waitForTransform("local_map_lidar", "base_link", pointCloud->header.stamp, ros::Duration(0.1));
			tf::StampedTransform tmp;
			tfListener_.lookupTransform("local_map_lidar", "base_link", pointCloud->header.stamp, tmp);
			pose = rtabmap_ros::transformFromTF(tmp);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
			exit(0);
		}
		const rtabmap::Signature& signature = createSignature(pose, pointCloud);
		addSignatureToOccupancyGrid(signature);
		nav_msgs::OccupancyGrid map = getOccupancyGridMap();
		map.header.stamp = pointCloud->header.stamp;
		occupancyGridPub_.publish(map);
		nodeId_++;
	}

	rtabmap::Signature createSignature(const rtabmap::Transform& pose, const sensor_msgs::PointCloud2ConstPtr& pointCloud) {
		rtabmap::SensorData data;
		rtabmap::LaserScan scan;
		bool convertion_ok = rtabmap_ros::convertScan3dMsg(
				*pointCloud,
				"base_link",
				"",
				ros::Time(0),
				scan,
				tfListener_,
				0,
				0);
		UASSERT(convertion_ok);
		filterLaserScan(scan);
		data.setLaserScan(scan);
		rtabmap::Signature signature(data);
		signature.setPose(rtabmap_ros::transformFromPoseMsg(pose));
		return signature;
	}
	*/

	void callback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& pointCloud) {
		UDEBUG("\n\nReceived new data");
		const rtabmap::Signature& signature = createSignature(odom, pointCloud);
		addSignatureToOccupancyGrid(signature);
		nav_msgs::OccupancyGrid map = getOccupancyGridMap();
		map.header.stamp = odom->header.stamp;
		occupancyGridPub_.publish(map);
		nodeId_++;
	}

	rtabmap::Signature createSignature(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& pointCloud) {
		rtabmap::SensorData data;
		rtabmap::LaserScan scan;
		bool convertion_ok = rtabmap_ros::convertScan3dMsg(
				*pointCloud,
				odom->child_frame_id,
				"",
				ros::Time(0),
				scan,
				tfListener_,
				0,
				0);
		UASSERT(convertion_ok);
		filterLaserScan(scan);
		data.setLaserScan(scan);
		rtabmap::Signature signature(data);
		signature.setPose(rtabmap_ros::transformFromPoseMsg(odom->pose.pose));
		return signature;
	}

	void filterLaserScan(rtabmap::LaserScan& scan) {
		UASSERT(scan.size() > 0);
		if(scan.rangeMax() == 0.0f)
		{
			bool is2d = scan.is2d();
			float squaredMaxRange = 0.0f;
			for(int i = 0; i < scan.size(); ++i)
			{
				const float* ptr = scan.data().ptr<float>(0, i);
				float sr;
				if(is2d)
				{
					sr = ptr[0]*ptr[0] + ptr[1]*ptr[1];
				}
				else
				{
					sr = ptr[0]*ptr[0] + ptr[1]*ptr[1] + ptr[2]*ptr[2];
				}
				if(sr > squaredMaxRange)
				{
					squaredMaxRange = sr;
				}
			}
			if(squaredMaxRange > 0.0f)
			{
				scan = LaserScan(scan.data(), scan.maxPoints(), sqrt(squaredMaxRange), scan.format(), scan.localTransform());
			}
		}

		scan = util3d::commonFiltering(scan,
				laserScanDownsampleStepSize_,
				0,
				0,
				laserScanVoxelSize_,
				laserScanNormalK_,
				laserScanNormalRadius_);
	}

	void addSignatureToOccupancyGrid(const rtabmap::Signature& signature) {
		cv::Mat groundCells;
		cv::Mat obstacleCells;
		cv::Mat emptyCells;
		cv::Point3f viewPoint;
		occupancyGrid_.createLocalMap(signature, groundCells, obstacleCells, emptyCells, viewPoint);
		occupancyGrid_.addToCache(nodeId_, groundCells, obstacleCells, emptyCells);
		poses_[nodeId_] = signature.getPose();
		occupancyGrid_.update(poses_);
	}

	nav_msgs::OccupancyGrid getOccupancyGridMap() {
		float gridCellSize = occupancyGrid_.getCellSize();
		float xMin, yMin;
		cv::Mat pixels = occupancyGrid_.getMap(xMin, yMin);
		UASSERT(!pixels.empty());

		nav_msgs::OccupancyGrid map;
		map.info.resolution = gridCellSize;
		map.info.origin.position.x = 0.0;
		map.info.origin.position.y = 0.0;
		map.info.origin.position.z = 0.0;
		map.info.origin.orientation.x = 0.0;
		map.info.origin.orientation.y = 0.0;
		map.info.origin.orientation.z = 0.0;
		map.info.origin.orientation.w = 1.0;

		map.info.width = pixels.cols;
		map.info.height = pixels.rows;
		map.info.origin.position.x = xMin;
		map.info.origin.position.y = yMin;
		map.data.resize(map.info.width * map.info.height);

		memcpy(map.data.data(), pixels.data, map.info.width * map.info.height);

		map.header.frame_id = "local_map_lidar";
		return map;
	}

private:
	ros::NodeHandle nh_;
	std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;
	std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pointCloudSub_;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
	std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync_;
	ros::Publisher occupancyGridPub_;
	/*
	ros::Subscriber pointCloudSub_;
	*/

	tf::TransformListener tfListener_;

	rtabmap::OccupancyGrid occupancyGrid_;
	int nodeId_ = 1;
	std::map<int, rtabmap::Transform> poses_;

	int laserScanDownsampleStepSize_;
	float laserScanVoxelSize_;
	int laserScanNormalK_;
	float laserScanNormalRadius_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancy_grid_builder");

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	// process "--params" argument
	for(int i = 1; i < argc; ++i)
	{
		if(strcmp(argv[i], "--params") == 0)
		{
			rtabmap::ParametersMap parameters;
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
			for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				std::string str = "Param: " + iter->first + " = \"" + iter->second + "\"";
				std::cout <<
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						rtabmap::Parameters::getDescription(iter->first).c_str() <<
						"]" <<
						std::endl;
			}
			ROS_WARN("Node will now exit after showing default parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}
		else if(strcmp(argv[i], "--udebug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
		}
		else if(strcmp(argv[i], "--uinfo") == 0)
		{
			ULogger::setLevel(ULogger::kInfo);
		}
	}

	OccupancyGridBuilder occupancy_grid_builder(argc, argv);
	ros::spin();
	return 0;
}
