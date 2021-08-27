#include "rtabmap_ros/OccupancyGridBuilderWrapper.h"
#include "time_measurer.h"

namespace rtabmap_ros {

rtabmap::ParametersMap OccupancyGridBuilder::readParameters(int argc, char** argv, const ros::NodeHandle& pnh) {
	std::string configPath;
	pnh.param("config_path", configPath, configPath);

	//parameters
	rtabmap::ParametersMap parameters;
	uInsert(parameters, rtabmap::Parameters::getDefaultParameters());

	if(!configPath.empty())
	{
		if(UFile::exists(configPath.c_str()))
		{
			ROS_INFO( "%s: Loading parameters from %s", ros::this_node::getName().c_str(), configPath.c_str());
			rtabmap::ParametersMap allParameters;
			rtabmap::Parameters::readINI(configPath.c_str(), allParameters);
			// only update odometry parameters
			for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				rtabmap::ParametersMap::iterator jter = allParameters.find(iter->first);
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

		if(iter->first.compare(rtabmap::Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
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
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=rtabmap::Parameters::getRemovedParameters().begin();
		iter!=rtabmap::Parameters::getRemovedParameters().end();
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

	return parameters;
}

OccupancyGridBuilder::OccupancyGridBuilder(int argc, char** argv) :
			CommonDataSubscriber(false),
			nodeId_(1) {
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	rtabmap::ParametersMap parameters = readParameters(argc, argv, pnh);
	occupancyGrid_.parseParameters(parameters);
	occupancyGridPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
	setupCallbacks(nh, pnh, "DataSubscriber");
}

void OccupancyGridBuilder::commonDepthCallback(
				const nav_msgs::OdometryConstPtr& odomMsg,
				const rtabmap_ros::UserDataConstPtr& userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
				const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor>& globalDescriptorMsgs /* std::vector<rtabmap_ros::GlobalDescriptor>() */,
				const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPoints /* std::vector<std::vector<rtabmap_ros::KeyPoint>>() */,
				const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3d /* std::vector<std::vector<rtabmap_ros::Point3f>>() */,
				const std::vector<cv::Mat>& localDescriptors /* std::vector<cv::Mat>() */) {
	MEASURE_BLOCK_TIME(DepthCallback);
	UDEBUG("\n\nReceived new data");
	const rtabmap::Signature& signature = createSignature(odomMsg, imageMsgs, depthMsgs, cameraInfoMsgs,
														  localKeyPoints, localPoints3d, localDescriptors);
	addSignatureToOccupancyGrid(signature);
	nav_msgs::OccupancyGrid map = getOccupancyGridMap();
	map.header.stamp = odomMsg->header.stamp;
	occupancyGridPub_.publish(map);
	nodeId_++;
}

rtabmap::Signature OccupancyGridBuilder::createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
														 const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
														 const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
														 const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
														 const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPointsMsgs,
														 const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3dMsgs,
														 const std::vector<cv::Mat>& localDescriptorsMsgs) {
	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point3f> points;
	cv::Mat descriptors;
	bool convertion_ok = rtabmap_ros::convertRGBDMsgs(imageMsgs, depthMsgs, cameraInfoMsgs, odomMsg->child_frame_id, "", ros::Time(0),
													  rgb, depth, cameraModels, tfListener_, 0,
													  localKeyPointsMsgs, localPoints3dMsgs, localDescriptorsMsgs,
													  &keypoints, &points, &descriptors);
	UASSERT(convertion_ok);
	rtabmap::SensorData data;
	data.setRGBDImage(rgb, depth, cameraModels);
	rtabmap::Signature signature(data);
	signature.setPose(rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose));
	return signature;
}

void OccupancyGridBuilder::commonLaserScanCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const sensor_msgs::LaserScan& scanMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
			const rtabmap_ros::GlobalDescriptor& globalDescriptor /* rtabmap_ros::GlobalDescriptor() */) {
	MEASURE_BLOCK_TIME(LaserScanCallback);
	UDEBUG("\n\nReceived new data");
	const rtabmap::Signature& signature = createSignature(odomMsg, scan3dMsg);
	addSignatureToOccupancyGrid(signature);
	nav_msgs::OccupancyGrid map = getOccupancyGridMap();
	map.header.stamp = odomMsg->header.stamp;
	occupancyGridPub_.publish(map);
	nodeId_++;
}

rtabmap::Signature OccupancyGridBuilder::createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
														 const sensor_msgs::PointCloud2& scan3dMsg) {
	rtabmap::LaserScan scan;
	bool convertion_ok = rtabmap_ros::convertScan3dMsg(scan3dMsg, odomMsg->child_frame_id, "", ros::Time(0), scan, tfListener_, 0);
	UASSERT(convertion_ok);
	rtabmap::SensorData data;
	data.setLaserScan(scan);
	rtabmap::Signature signature(data);
	signature.setPose(rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose));
	return signature;
}

void OccupancyGridBuilder::addSignatureToOccupancyGrid(const rtabmap::Signature& signature) {
	cv::Mat groundCells;
	cv::Mat obstacleCells;
	cv::Mat emptyCells;
	cv::Point3f viewPoint;
	occupancyGrid_.createLocalMap(signature, groundCells, obstacleCells, emptyCells, viewPoint);
	occupancyGrid_.addToCache(nodeId_, groundCells, obstacleCells, emptyCells);
	poses_[nodeId_] = signature.getPose();
	occupancyGrid_.update(poses_);
}

nav_msgs::OccupancyGrid OccupancyGridBuilder::getOccupancyGridMap() {
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

}
