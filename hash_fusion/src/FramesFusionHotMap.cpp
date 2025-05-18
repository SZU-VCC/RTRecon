#include "FramesFusion.h"
/**
 * 问题记录：
 *  被树挡住的区域被错误更新
 *  height 计算错误，由于sdf的射线法计算
*/
void FramesFusion::GenerateHotMap(const Eigen::Vector3f& vCenter, const int iFrameId, const ros::Time oTimeStamp) {

	TimeDebuggerProxy timer(iFrameId, &reconstruct_timer);

	grid_map::GridMap oHotMap({"sdf", "start", "occ", "free"});
	oHotMap.setFrameId(odomFrameId);
	// 此处或许需要动态的局部窗口
	oHotMap.setGeometry(grid_map::Length(100.0, 100.0), 1.0, grid_map::Position(vCenter.x(), vCenter.y()));
	
	DistanceIoVolume* pDistanceIoVolume = dynamic_cast<DistanceIoVolume*>(distanceIoVolume.get());
	if(pDistanceIoVolume == nullptr) {
		ROS_INFO_CYAN("%s", "[FramesFusion] Volume type is not DistanceIoVolume!");
		return;
	}

	constexpr int iHeightSampleCount = 30;
	constexpr float fFloorOffset = 5.0f;
	float fHeightStart = vCenter.z() - fFloorOffset;
	float fSampleStep = pDistanceIoVolume->GetVoxelLength().z() * 0.5;

	for(grid_map::GridMapIterator it(oHotMap); !it.isPastEnd(); ++it) {

		// sdf
		grid_map::Position position;
		oHotMap.getPosition(*it, position);
		Eigen::Vector3f vQueryPoint(position.x(), position.y(), vCenter.z());
		float fSdf = pDistanceIoVolume->SearchSdf(vQueryPoint);
		oHotMap.at("sdf", *it) = fSdf;

		// calculate map
		int iStatus = 0, iIndexRecord = 0;
		float fHeight = fHeightStart;
		for(int i = 0; i < iHeightSampleCount && iStatus < 3; ++i, fHeight+=fSampleStep) {
			Eigen::Vector3f vQueryPoint(position.x(), position.y(), fHeight);
			float fSdf = pDistanceIoVolume->SearchSdf(vQueryPoint);
			switch(iStatus) {
				case 0:
					if(abs(fSdf) <= fSampleStep) {
						iStatus = 1;
						iIndexRecord = i;
						oHotMap.at("start", *it) = fHeight;
					}
					break;
				case 1:
					if(abs(fSdf) >= fSampleStep) {
						iStatus = 2;
						oHotMap.at("occ", *it) = i - iIndexRecord;
						iIndexRecord = i;
					}
					break;
				case 2:
					if(fSdf <= 0) {
						iStatus = 3;
						oHotMap.at("free", *it) = i - iIndexRecord;
					}
					break;
			}
		}
		switch(iStatus) {
			case 0: 
				oHotMap.at("start", *it) = -INFINITY; 
				break;
			case 1: 
				oHotMap.at("occ", *it) = iHeightSampleCount - iIndexRecord; 
				break;
			case 2: 
				oHotMap.at("free", *it) = iHeightSampleCount - iIndexRecord; 
				break;
		}
	}

	timer.DebugTime("1_generate_hotmap");

	oHotMap.setTimestamp(oTimeStamp.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(oHotMap, message);
    HotMapPublisher.publish(message);

	timer.DebugTime("2_publish_hotmap");

	timer.GetCurrentLineTime();
}

/**
 * 未经过预处理的地图输出，处理步骤交由导航算法执行
 * @param vCenter - 雷达中心位置
 * @param iFrameId - 接收到的帧的编号
 * @param oTimeStamp - 帧的时间戳
*/
void FramesFusion::ExtractSdfMaps(const Eigen::Vector3f& vCenter, const int iFrameId, const ros::Time oTimeStamp) 
{
	TimeDebuggerProxy timer(iFrameId, &reconstruct_timer);

	grid_map::GridMap oSdfMap({"height"});
	oSdfMap.setFrameId(odomFrameId);
	oSdfMap.setGeometry(grid_map::Length(100.0, 100.0), 1.0, grid_map::Position(vCenter.x(), vCenter.y()));
	
	DistanceIoVolume* pDistanceIoVolume = dynamic_cast<DistanceIoVolume*>(distanceIoVolume.get());
	if(pDistanceIoVolume == nullptr) {
		ROS_INFO_CYAN("%s", "[FramesFusion] Volume type is not DistanceIoVolume!");
		return;
	}

	constexpr int iLayerNumber = 7; // 应该设置为奇数(如果中心是0的话)
	constexpr double dLayerCenterOffset = 0;
	constexpr double dLayerStep = 0.5;

	std::vector<double> vHeightList(iLayerNumber);
	std::vector<std::string> vLayerNameList(iLayerNumber);

	for(int i = 0; i < iLayerNumber; ++i) {

		double dLayerHeight = dLayerCenterOffset - (i - iLayerNumber / 2) * dLayerStep;

		std::stringstream sLayerNameMaker;
		sLayerNameMaker << "sdf_" << dLayerHeight;
		oSdfMap.add(sLayerNameMaker.str());

		vHeightList[i] = dLayerHeight;
		vLayerNameList[i] = sLayerNameMaker.str();
	}

	for(grid_map::GridMapIterator it(oSdfMap); !it.isPastEnd(); ++it) {

		// sdf
		grid_map::Position position;
		oSdfMap.getPosition(*it, position);
		oSdfMap.at("height", *it) = -2.0;

		for(int i = 0; i < iLayerNumber; ++i) {

			Eigen::Vector3f vQueryPoint(position.x(), position.y(), vCenter.z() + vHeightList[i]);
			float fSdf = pDistanceIoVolume->SearchSdf(vQueryPoint);
			if(fSdf == -std::numeric_limits<float>().infinity()) fSdf = -1.0f;
			oSdfMap.at(vLayerNameList[i], *it) = fSdf;
		}
	}

	timer.DebugTime("1_generate_sdfmap");

	oSdfMap.setTimestamp(oTimeStamp.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(oSdfMap, message);
    HotMapPublisher.publish(message);

	timer.DebugTime("2_publish_sdfmap");

	timer.GetCurrentLineTime();
	// timer.CoutCurrentLine();

	if(outputFiles) {
		OutputSdfMapImage(vCenter, oSdfMap, iFrameId);
	}
}
