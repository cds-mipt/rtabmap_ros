#include "rtabmap_ros/OccupancyGridBuilderWrapper.h"

bool writeMatBinary(std::fstream& ofs, const cv::Mat& out_mat)
{
	if(!ofs.is_open()) {
		return false;
	}
	if(out_mat.empty()) {
		int s = 0;
		ofs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = out_mat.type();
	ofs.write((const char*)(&out_mat.rows), sizeof(int));
	ofs.write((const char*)(&out_mat.cols), sizeof(int));
	ofs.write((const char*)(&type), sizeof(int));
	ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

	return true;
}

bool readMatBinary(std::fstream& ifs, cv::Mat& in_mat)
{
	if(!ifs.is_open()) {
		return false;
	}
	
	int rows, cols, type;
	ifs.read((char*)(&rows), sizeof(int));
	if(rows == 0) {
		return true;
	}
	ifs.read((char*)(&cols), sizeof(int));
	ifs.read((char*)(&type), sizeof(int));

	in_mat.release();
	in_mat.create(rows, cols, type);
	ifs.read((char*)(in_mat.data), in_mat.elemSize() * in_mat.total());

	return true;
}

namespace rtabmap_ros {

rtabmap::ParametersMap OccupancyGridBuilder::readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh) {
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

void OccupancyGridBuilder::readParameters(const ros::NodeHandle& pnh) {
	pnh.param("db_path", dbPath_, std::string(""));
	pnh.param("load_db", loadDb_, false);
	pnh.param("save_db", saveDb_, false);
	pnh.param("sync_save", syncSave_, false);
}

OccupancyGridBuilder::OccupancyGridBuilder(int argc, char** argv) :
			CommonDataSubscriber(false),
			UThread(),
			nodeId_(1),
			dbDriver_(0) {
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	rtabmap::ParametersMap parameters = readRtabmapParameters(argc, argv, pnh);
	readParameters(pnh);

	occupancyGrid_.parseParameters(parameters);
	occupancyGridPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);

	if (dbPath_.empty()) {
		loadDb_ = false;
		saveDb_ = false;
	}
	if (syncSave_) {
		if (loadDb_) {
			if (saveDb_) {
				dbFile_.open(dbPath_, std::fstream::in | std::fstream::out | std::fstream::binary | std::fstream::app);
			} else {
				dbFile_.open(dbPath_, std::fstream::in | std::fstream::binary | std::fstream::app);
			}
			UASSERT(dbFile_.is_open());
			loadOccupancyGrid();
			if (!saveDb_) {
				dbFile_.close();
			} else {
				dbFile_.clear();
				dbFile_.seekp(0, std::fstream::end);
			}
		}
		else if (saveDb_) {
			dbFile_.open(dbPath_, std::fstream::out | std::fstream::binary | std::fstream::trunc);
		}
	} else {
		if (loadDb_ || saveDb_) {
			dbDriver_ = rtabmap::DBDriver::create();
			if (loadDb_) {
				bool openOk = dbDriver_->openConnection(dbPath_, false);
				UASSERT(openOk);
				loadOccupancyGrid();
				if (!saveDb_) {
					dbDriver_->closeConnection();
					delete dbDriver_;
				}
			} else {
				bool openOk = dbDriver_->openConnection(dbPath_, true);
				UASSERT(openOk);
			}
		}
	}

	setupCallbacks(nh, pnh, "DataSubscriber");
}

OccupancyGridBuilder::~OccupancyGridBuilder() {
	if (saveDb_) {
		if (syncSave_) {
			dbFile_.close();
		} else {
			join();
			start();
			join();
			dbDriver_->closeConnection();
			delete dbDriver_;
		}
	}
}

void OccupancyGridBuilder::loadOccupancyGridForSync() {
	int maxNodeId = 0;
	dbFile_.seekg(0, std::fstream::beg);
	while (dbFile_.peek() != EOF) {
		int nodeId;
		dbFile_.read((char*)(&nodeId), sizeof(int));
		if (nodeId > maxNodeId) maxNodeId = nodeId;
		cv::Mat poseMat;
		readMatBinary(dbFile_, poseMat);
		rtabmap::Transform pose(poseMat);
		poses_[nodeId] = pose;
		cv::Mat groundCells, obstacleCells, emptyCells;
		readMatBinary(dbFile_, groundCells);
		readMatBinary(dbFile_, obstacleCells);
		readMatBinary(dbFile_, emptyCells);
		occupancyGrid_.addToCache(nodeId, groundCells, obstacleCells, emptyCells);
	}
	occupancyGrid_.update(poses_);
	nodeId_ = maxNodeId + 1;
}

void OccupancyGridBuilder::loadOccupancyGridForAsync() {
	std::set<int> nodeIds;
	dbDriver_->getAllNodeIds(nodeIds);
	int maxNodeId = 0;
	for (auto nodeId : nodeIds) {
		if (nodeId > maxNodeId) maxNodeId = nodeId;
		rtabmap::SensorData data;
		dbDriver_->getNodeData(nodeId, data, false, false, false, true);
		UASSERT(data.gridCellSize() == occupancyGrid_.getCellSize());
		cv::Mat groundCells, obstacleCells, emptyCells;
		data.uncompressDataConst(0, 0, 0, 0, &groundCells, &obstacleCells, &emptyCells);
		occupancyGrid_.addToCache(nodeId, groundCells, obstacleCells, emptyCells);
		rtabmap::Transform pose;
		int mapId, weight;
		std::string label;
		double stamp;
		rtabmap::Transform groundTruthPose;
		std::vector<float> velocity;
		rtabmap::GPS gps;
		rtabmap::EnvSensors sensors;
		dbDriver_->getNodeInfo(nodeId, pose, mapId, weight, label, stamp, groundTruthPose, velocity, gps, sensors);
		poses_[nodeId] = pose;
	}
	occupancyGrid_.update(poses_);
	nodeId_ = maxNodeId + 1;
}

void OccupancyGridBuilder::loadOccupancyGrid() {
	MEASURE_BLOCK_TIME(loadOccupancyGrid);
	if (syncSave_) {
		loadOccupancyGridForSync();
	} else {
		loadOccupancyGridForAsync();
	}
}

void OccupancyGridBuilder::mainLoop() {
	MEASURE_BLOCK_TIME(mainLoop);
	dataToSaveMutex_.lock();
	auto nodeIdsToSave = std::move(nodeIdsToSave_);
	auto posesToSave = std::move(posesToSave_);
	auto groundCellsToSave = std::move(groundCellsToSave_);
	auto obstacleCellsToSave = std::move(obstacleCellsToSave_);
	auto emptyCellsToSave = std::move(emptyCellsToSave_);
	dataToSaveMutex_.unlock();
	UASSERT(nodeIdsToSave.size() == posesToSave.size());
	UASSERT(posesToSave.size() == groundCellsToSave.size());
	UASSERT(groundCellsToSave.size() == obstacleCellsToSave.size());
	UASSERT(obstacleCellsToSave.size() == emptyCellsToSave.size());
	while (!nodeIdsToSave.empty()) {
		int nodeId = nodeIdsToSave.back();
		const rtabmap::Transform& pose = posesToSave.back();
		const cv::Mat& groundCells = groundCellsToSave.back();
		const cv::Mat& obstacleCells = obstacleCellsToSave.back();
		const cv::Mat& emptyCells = emptyCellsToSave.back();
		rtabmap::Signature* signature = new rtabmap::Signature(nodeId);
		signature->setPose(pose);
		signature->sensorData().setOccupancyGrid(groundCells, obstacleCells, emptyCells, occupancyGrid_.getCellSize(), cv::Point3f());
		dbDriver_->asyncSave(signature);
		nodeIdsToSave.pop_back();
		posesToSave.pop_back();
		groundCellsToSave.pop_back();
		obstacleCellsToSave.pop_back();
		emptyCellsToSave.pop_back();
	}
	dbDriver_->emptyTrashes();
	kill();
}

void OccupancyGridBuilder::saveCellsForSync(const rtabmap::Signature& signature, const cv::Mat& groundCells,
											const cv::Mat& obstacleCells, const cv::Mat& emptyCells) {
	int nodeId = signature.id();
	dbFile_.write((const char*)(&nodeId), sizeof(int));
	const cv::Mat& poseMat = signature.getPose().dataMatrix();
	writeMatBinary(dbFile_, poseMat);
	writeMatBinary(dbFile_, groundCells);
	writeMatBinary(dbFile_, obstacleCells);
	writeMatBinary(dbFile_, emptyCells);
}

void OccupancyGridBuilder::saveCellsForAsync(const rtabmap::Signature& signature, const cv::Mat& groundCells,
											 const cv::Mat& obstacleCells, const cv::Mat& emptyCells) {
	dataToSaveMutex_.lock();
	nodeIdsToSave_.push_back(signature.id());
	posesToSave_.push_back(signature.getPose());
	groundCellsToSave_.push_back(groundCells);
	obstacleCellsToSave_.push_back(obstacleCells);
	emptyCellsToSave_.push_back(emptyCells);
	dataToSaveMutex_.unlock();
	if (nodeIdsToSave_.size() == 1000) {
		start();
	}
}

void OccupancyGridBuilder::saveCells(const rtabmap::Signature& signature, const cv::Mat& groundCells,
									 const cv::Mat& obstacleCells, const cv::Mat& emptyCells) {
	MEASURE_BLOCK_TIME(saveCells);
	if (syncSave_) {
		saveCellsForSync(signature, groundCells, obstacleCells, emptyCells);
	} else {
		saveCellsForAsync(signature, groundCells, obstacleCells, emptyCells);
	}
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
	MEASURE_BLOCK_TIME(commonDepthCallback);
	UDEBUG("\n\nReceived new data");
	const rtabmap::Signature& signature = createSignature(odomMsg, imageMsgs, depthMsgs, cameraInfoMsgs,
														  localKeyPoints, localPoints3d, localDescriptors);
	manageNewSignature(signature, odomMsg->header.stamp, odomMsg->header.frame_id);
}

void OccupancyGridBuilder::commonLaserScanCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const sensor_msgs::LaserScan& scanMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
			const rtabmap_ros::GlobalDescriptor& globalDescriptor /* rtabmap_ros::GlobalDescriptor() */) {
	MEASURE_BLOCK_TIME(commonLaserScanCallback);
	UDEBUG("\n\nReceived new data");
	const rtabmap::Signature& signature = createSignature(odomMsg, scan3dMsg);
	manageNewSignature(signature, odomMsg->header.stamp, odomMsg->header.frame_id);
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
	bool convertionOk = rtabmap_ros::convertRGBDMsgs(imageMsgs, depthMsgs, cameraInfoMsgs, odomMsg->child_frame_id, "", ros::Time(0),
													 rgb, depth, cameraModels, tfListener_, 0,
													 localKeyPointsMsgs, localPoints3dMsgs, localDescriptorsMsgs,
													 &keypoints, &points, &descriptors);
	UASSERT(convertionOk);
	rtabmap::SensorData data;
	data.setRGBDImage(rgb, depth, cameraModels);
	rtabmap::Signature signature(data);
	signature.setPose(rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose));
	return signature;
}

rtabmap::Signature OccupancyGridBuilder::createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
														  const sensor_msgs::PointCloud2& scan3dMsg) {
	rtabmap::LaserScan scan;
	bool convertionOk = rtabmap_ros::convertScan3dMsg(scan3dMsg, odomMsg->child_frame_id, "", ros::Time(0), scan, tfListener_, 0);
	UASSERT(convertionOk);
	rtabmap::SensorData data;
	data.setId(nodeId_);
	data.setLaserScan(scan);
	rtabmap::Signature signature(data);
	signature.setPose(rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose));
	return signature;
}

void OccupancyGridBuilder::manageNewSignature(const rtabmap::Signature& signature, ros::Time stamp, std::string frame_id) {
	cv::Mat groundCells, obstacleCells, emptyCells;
	cv::Point3f viewPoint;
	addSignatureToOccupancyGrid(signature, groundCells, obstacleCells, emptyCells, viewPoint);
	if (saveDb_) {
		saveCells(signature, groundCells, obstacleCells, emptyCells);
	}
	nav_msgs::OccupancyGrid map = getOccupancyGridMap();
	map.header.stamp = stamp;
	map.header.frame_id = frame_id;
	occupancyGridPub_.publish(map);
	nodeId_++;
}

void OccupancyGridBuilder::addSignatureToOccupancyGrid(const rtabmap::Signature& signature, cv::Mat& groundCells, cv::Mat& obstacleCells,
													   cv::Mat& emptyCells, cv::Point3f& viewPoint) {
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
	return map;
}

}
