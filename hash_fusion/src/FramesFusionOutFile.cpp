#include "FramesFusion.h"

std::string FramesFusion::CheckOutputPath(std::string & outFileHead, const std::string & fileName){
    std::string sfOutFilePath;
    if (outFileHead.back() != '/')
        outFileHead += "/";
    sfOutFilePath = outFileHead + fileName;
    if (sfOutFilePath.back() != '/')
       sfOutFilePath += "/";
    if (std::filesystem::exists(sfOutFilePath)) {
        if (std::filesystem::is_directory(sfOutFilePath)) { // 路径存在，且是文件夹
        } else { // 路径存在，但不是文件夹
            LOG(ERROR) << "Output path exists, but it is not a directory: " << sfOutFilePath;
        }
    } else { // 路径不存在
        std::stringstream sOutputCommand;
        sOutputCommand << "mkdir -p " << sfOutFilePath;
        if (system(sOutputCommand.str().c_str()) != -1) {
        } else {
            LOG(ERROR) << "Output path does not exist, creating failed: " << sfOutFilePath;
        }
    }
    return sfOutFilePath;
}


/*************************************************
Function: SaveFinalMeshAndPointCloud
Description: save mesh and point cloud of whole scene to files
Called By: ~FramesFusion
*************************************************/
void FramesFusion::SaveFinalMeshAndPointCloud() {

	//define ouput ply file name
	std::stringstream sOutPCNormalFileName;
	sOutPCNormalFileName << fileHead << "Map_PCNormal.ply";

	if(useAdditionalPoints) {
	
		mapPCN += mapPCNAdded;
		mapPCN += mapPCNTrueAdded;
	}

	// TODO： output fix

	if(useUnionSetConnection) {
		distanceIoVolume->RebuildUnionSetAll(strictDotRef, softDotRef, confidenceLevelLength);
		distanceIoVolume->UpdateUnionConflict(removeSizeRef, removeTimeRef);
	}

	HashVoxeler::HashVolume vVolumeCopy;
	SignedDistance oSDF(-1, convDim, convAddPointNumRef, convFusionDistanceRef1);
	std::unordered_map<HashPos, float, HashFunc> vSignedDis;
	if(useUnionSetConnection) vSignedDis = oSDF.ConvedGlanceAllUnion(*distanceIoVolume, removeSizeRef);
	else vSignedDis = oSDF.ConvedGlance(*distanceIoVolume);
	vVolumeCopy = oSDF.m_vVolumeCopy;

	// output mesh static
	{
		pcl::PolygonMesh oResultMesh;
		CIsoSurface<float> oMarchingCuber;
		Eigen::Vector3f vVoxelLength = distanceIoVolume->GetVoxelLength();
		oMarchingCuber.GenerateSurface(vSignedDis, oSDF.m_vVolumeCopy, 0, vVoxelLength.x(), vVoxelLength.y(), vVoxelLength.z());
		pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ oOffset(0, 0, 0);
		oMarchingCuber.OutputMesh(oOffset, oResultMesh, pMCResultCloud);
		for(auto & polygon : oResultMesh.polygons) {
			polygon.vertices.pop_back();
		}
		std::stringstream sOutputPath;
		sOutputPath << fileHead << "final_mesh.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oResultMesh);
		std::cout << output::format_cyan << "The output file is " << sOutputPath.str() << output::format_white << std::endl;
	}
	// output mesh dynamic
	{
		SignedDistance oSDF(-1, convDim, 0, convFusionDistanceRef1);
		auto vSignedDis = oSDF.ConvedGlanceAll(*distanceIoVolume);
		
		pcl::PolygonMesh oResultMesh;
		CIsoSurface<float> oMarchingCuber;
		Eigen::Vector3f vVoxelLength = distanceIoVolume->GetVoxelLength();
		oMarchingCuber.GenerateSurface(vSignedDis, oSDF.m_vVolumeCopy, 0, vVoxelLength.x(), vVoxelLength.y(), vVoxelLength.z());
		pcl::PointCloud<pcl::PointXYZ>::Ptr pMCResultCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ oOffset(0, 0, 0);
		oMarchingCuber.OutputMesh(oOffset, oResultMesh, pMCResultCloud);
		for(auto & polygon : oResultMesh.polygons) {
			polygon.vertices.pop_back();
		}
		std::stringstream sOutputPath;
		sOutputPath << fileHead << "final_mesh_worm.ply";
		pcl::io::savePLYFileBinary(sOutputPath.str(), oResultMesh);
		std::cout << output::format_cyan << "The output file is " << sOutputPath.str() << output::format_white << std::endl;
	}

	// output pc
	auto vAllPoints = AllCloud(mapPCN);
	pcl::PointCloud<pcl::PointNormal> pc; 
	pcl::PointCloud<pcl::PointXYZINormal> static_pc;
	pcl::PointCloud<pcl::PointNormal> dynamic_pc;
	for(int i = 0; i < vAllPoints->size(); ++i) {

		// if the point cloud is too large, open this to random sample
		// if(rand() % 8 > 3) continue;

		std::cout << "All ready process points: " << i+1 << "/" << vAllPoints->size() << "\r";

		pcl::PointNormal& point = (*vAllPoints)[i];
		pc.push_back(point);

		HashPos pos;
		distanceIoVolume->PointBelongVoxelPos(point, pos);
		if(!vVolumeCopy.count(pos)) {
			dynamic_pc.push_back(point);
			continue;
		}

		// filter inner points
		float distance_ref, normal_ref;
		Eigen::Vector3f oCorner = vVolumeCopy[pos].getVector3fMap();
		Eigen::Vector3f oNormal = vVolumeCopy[pos].getNormalVector3fMap();
		Eigen::Vector3f vPoint = point.getVector3fMap();
		Eigen::Vector3f vNormal = point.getNormalVector3fMap();
		float & token = vVolumeCopy[pos].data_c[1];
		if(vVolumeCopy[pos].data_c[3] > convFusionDistanceRef1 || token >= __INT_MAX__) {
			distance_ref = distanceIoVolume->GetVoxelLength().norm() * 0.1;
			normal_ref = 0.8;
		}
		else {
			distance_ref = distanceIoVolume->GetVoxelLength().norm() * 0.3;
			normal_ref = 0;
		}

		if(abs(oNormal.dot(oCorner - vPoint)) > distance_ref || vNormal.dot(oNormal) < normal_ref) {
			// if(vNormal.dot(oNormal) < normal_ref)
			// 	dynamic_pc.push_back(point);
			continue;
		}

		// put point in final result
		pcl::PointXYZINormal new_point;
		new_point.x = point.x;
		new_point.y = point.y;
		new_point.z = point.z;
		new_point.normal_x = point.normal_x;
		new_point.normal_y = point.normal_y;
		new_point.normal_z = point.normal_z;
		
		// point confidence (impacted by depth & support & conflict)
		new_point.intensity = min(vVolumeCopy[pos].data_n[3], 100.0f); 
		
		// maybe use to save gauss distance, but now don't make scence
		new_point.curvature = vVolumeCopy[pos].data_c[2];

		static_pc.push_back(new_point);
	}
	std::cout << "\nSaving file ..." << std::endl;
	pcl::io::savePLYFileBinary(sOutPCNormalFileName.str(), pc);
	pcl::io::savePLYFileBinary(sOutPCNormalFileName.str()+".static.ply", static_pc);
	pcl::io::savePLYFileBinary(sOutPCNormalFileName.str()+".dynamic.ply", dynamic_pc);
}

void FramesFusion::OutputSdfMapImage(const Eigen::Vector3f& vCenter, grid_map::GridMap& oSdfMap, int iFrameIndex) {

	constexpr int iLayerNumber = 5; // 应该设置为奇数(如果中心是0的话)
	constexpr float scaler = 2.56f;
	constexpr double dLayerCenterOffset = 0;
	constexpr double dLayerStep = 0.5;

	int width = oSdfMap.getSize().x(), height = oSdfMap.getSize().y();
	std::vector<std::string> vLayerNameList(iLayerNumber);
	std::vector<cv::Mat> vImageList(vLayerNameList.size());
	for(auto& oImage : vImageList) {
		oImage.create(width * scaler, height * scaler, CV_8UC3);
	}

	///* 
	DistanceIoVolume* pDistanceIoVolume = dynamic_cast<DistanceIoVolume*>(distanceIoVolume.get());
	if(pDistanceIoVolume == nullptr) {
		ROS_INFO_CYAN("%s", "[FramesFusion] Volume type is not DistanceIoVolume!");
		return;
	}
	
	std::vector<double> vHeightList(vLayerNameList.size());
	for(int i = 0; i < vLayerNameList.size(); ++i) {

		double dLayerHeight = dLayerCenterOffset - (i - iLayerNumber / 2) * dLayerStep;

		std::stringstream sLayerNameMaker;
		sLayerNameMaker << "sdf_" << dLayerHeight;
		oSdfMap.add(sLayerNameMaker.str());

		vHeightList[i] = dLayerHeight;
		vLayerNameList[i] = sLayerNameMaker.str();
	}

	for(int x = 0; x < width * scaler; ++x) {
		float fx = x / scaler + vCenter.x() - width / 2;
	for(int y = 0; y < height * scaler; ++y) {
		float fy = y / scaler + vCenter.y() - height / 2;
		for(int i = 0; i < vLayerNameList.size(); ++i) {
			Eigen::Vector3f vQueryPoint(fx, fy, vCenter.z() + vHeightList[i]);
			float fSdf = pDistanceIoVolume->SearchSdf(vQueryPoint);
			if(fSdf == -std::numeric_limits<float>().infinity()) fSdf = -1.0f;
			fSdf = 0.5f - fSdf / 2.0f;
			// fSdf = 0.5f - (fSdf - 2.0f) / 6.0f;
			fSdf = std::clamp(fSdf, 0.0f, 0.8f) * 360.0f;
			cv::Vec3b& pixel = vImageList[i].at<cv::Vec3b>(x, y);
			rosPubManager.HSVToRGB(fSdf, 1, 1, pixel[2], pixel[1], pixel[0]);
		}
	}}
	//*/

	/*
	for(grid_map::GridMapIterator it(oSdfMap); !it.isPastEnd(); ++it) {

		// sdf
		grid_map::Position position;
		oSdfMap.getPosition(*it, position);
		position += oSdfMap.getSize().cast<double>().matrix() * 0.5;
		position -= vCenter.topRows(2).cast<double>();

		for(int i = 0; i < vLayerNameList.size(); ++i) {
			float fSdf = oSdfMap.at(vLayerNameList[i], *it);
			fSdf = 0.5f - (fSdf - 2.0f) / 6.0f;
			fSdf = std::clamp(fSdf, 0.0f, 0.8f) * 360.0f;
			cv::Vec3b& pixel = vImageList[i].at<cv::Vec3b>(position.x(), position.y());
			rosPubManager.HSVToRGB(fSdf, 1, 1, pixel[2], pixel[1], pixel[0]);
		}
	}
	//*/
	if(outputFiles)
		for(int i = 0; i < vLayerNameList.size(); ++i) {
			std::stringstream sFileName;
			sFileName << outputPathList["hotmap"] << "dy_" << iFrameIndex << "_" << vLayerNameList[i] << ".png";
			cv::resize(vImageList[i], vImageList[i], cv::Size(), 2, 2);
			cv::imwrite(sFileName.str(), vImageList[i]);
		}
}

void FramesFusion::OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, bool bAllRecord){
  
    //generate a output file if possible
	if( outPCFileFlag || bAllRecord){

		outPCFileName.clear();
	    //set the current time stamp as a file name
		//outPCFileName << fileHead << "PC_" << ros::Time::now() << ".txt"; 

		//set the count as a file name
		outPCFileName << fileHead << "PC_" << pcFrameCount << ".txt"; 

		outPCFileFlag = false;

        //print output file generation message
		std::cout << "[*] Attention, a point cloud recording file is created in " << outPCFileName.str() << std::endl;
	}

    //output
	outPCFile.open(outPCFileName.str(), std::ios::out | std::ios::app);

	//output in a txt file
	//the storage type of output file is x y z time frames 
    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        outPCFile << vCloud.points[i].x << " "
                  << vCloud.points[i].y << " "
                  << vCloud.points[i].z << " " 
                  << pcFrameCount << " " 
                  << std::endl;
    }//end for         

    outPCFile.close();

    //count new point cloud input (plus frame) 

}

void FramesFusion::OutputPCFile(const pcl::PointCloud<pcl::PointXYZ> & vCloud, const std::vector<float> & vFeatures, bool bAllRecord){
  
    //generate a output file if possible
	if( outPCFileFlag || bAllRecord){

		outPCFileName.clear();
	    //set the current time stamp as a file name
		//outPCFileName << fileHead << "PC_" << ros::Time::now() << ".txt"; 

		//set the count as a file name
		outPCFileName << fileHead << "PC_" << pcFrameCount << ".txt"; 

		outPCFileFlag = false;

        //print output file generation message
		std::cout << "[*] Attention, a point cloud recording file is created in " << outPCFileName.str() << std::endl;
	}

    //output
	outPCFile.open(outPCFileName.str(), std::ios::out | std::ios::app);

	//output in a txt file
	//the storage type of output file is x y z time frames 
    //record the point clouds
    for(int i = 0; i != vCloud.size(); ++i ){

        //output in a txt file
        //the storage type of output file is x y z time frames right/left_sensor
        outPCFile << vCloud.points[i].x << " "
                  << vCloud.points[i].y << " "
                  << vCloud.points[i].z << " " 
                  << vFeatures[i] << " "
                  << pcFrameCount << " " 
                  << std::endl;
    }//end for         

    outPCFile.close();

    //count new point cloud input (plus frame) 

}

pcl::PointCloud<pcl::PointNormal>::Ptr AllCloud(pcl::PointCloud<pcl::PointNormal>& cloud_vector) {

    return cloud_vector.makeShared();
}

void FramesFusion::SaveVoxels() {

	//define ouput ply file name
	std::stringstream sOutVoxelFileName;
	sOutVoxelFileName << fileHead << "Voxel.ply";

	DistanceIoVolume* pDistanceIoVolume = dynamic_cast<DistanceIoVolume*>(distanceIoVolume.get());
	if(pDistanceIoVolume == nullptr) {
		ROS_INFO_CYAN("%s", "[FramesFusion] Volume type is not DistanceIoVolume!");
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB> vVoxels;
	for(auto&& [oPos,_] : pDistanceIoVolume->m_vVolume) {
		int iLevel = pDistanceIoVolume->SearchLevel(oPos);
		int iIo = pDistanceIoVolume->SearchIo(oPos);
		if(iLevel >= 0) {
			float fLength = voxelResolution.x * (1 << iLevel);
			vVoxels.push_back(pcl::PointXYZRGB(255 - iLevel * 50, iIo ? 0 : 255, iLevel));
			vVoxels.back().getVector3fMap() = Eigen::Vector3f(oPos.x, oPos.y, oPos.z);
		}
	}

	pcl::io::savePLYFileBinary(sOutVoxelFileName.str(), vVoxels);
	std::cout << output::format_cyan << "The output file is " << sOutVoxelFileName.str() << output::format_white << std::endl;
}