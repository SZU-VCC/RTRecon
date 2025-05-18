#include "FrameRecon.h"


std::string FrameRecon::CheckOutputPath(std::string & outFileHead, const std::string & fileName){
    std::string OutFilePath;
    if (outFileHead.back() != '/')
        outFileHead += "/";
    OutFilePath = outFileHead + fileName;
    if (OutFilePath.back() != '/')
       OutFilePath += "/";
    if (std::filesystem::exists(OutFilePath)) {
        if (std::filesystem::is_directory(OutFilePath)) { // 路径存在，且是文件夹
        } else { // 路径存在，但不是文件夹
            LOG(ERROR) << "Output path exists, but it is not a directory: " << OutFilePath;
        }
    } else { // 路径不存在
        std::stringstream sOutputCommand;
        sOutputCommand << "mkdir -p " << OutFilePath;
        if (system(sOutputCommand.str().c_str()) != -1) {
        } else {
            LOG(ERROR) << "Output path does not exist, creating failed: " << OutFilePath;
        }
    }
    return OutFilePath;
}

void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> &vCloud, bool bAllRecord) {
    // generate a output file if possible
    if (outPCFileFlag || bAllRecord) {
        outPCFileName.clear();
        // set the current time stamp as a file name
        // outPCFileName << sfOutFileHead << "PC_" << ros::Time::now() <<
        // ".txt";

        // set the count as a file name
        outPCFileName << sfOutFileHead << "PC_" << pcFrameCount << ".txt";

        outPCFileFlag = false;

        // print output file generation message
        std::cout << "[*] Attention, a point cloud recording file is created in " << outPCFileName.str() << std::endl;
    }

    // output
    outPCFile.open(outPCFileName.str(), std::ios::out | std::ios::app);

    // output in a txt file
    // the storage type of output file is x y z time frames
    // record the point clouds
    for (int i = 0; i != vCloud.size(); ++i) {
        // output in a txt file
        // the storage type of output file is x y z time frames
        // right/left_sensor
        outPCFile << vCloud.points[i].x << " " << vCloud.points[i].y << " " << vCloud.points[i].z << " " << pcFrameCount
                  << " " << std::endl;
    } // end for

    outPCFile.close();

    // count new point cloud input (plus frame)
}

void FrameRecon::OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> &vCloud, const std::vector<float> &vFeatures,
                              bool bAllRecord) {
    // generate a output file if possible
    if (outPCFileFlag || bAllRecord) {
        outPCFileName.clear();
        // set the current time stamp as a file name
        // outPCFileName << sfOutFileHead << "PC_" << ros::Time::now() <<
        // ".txt";

        // set the count as a file name
        outPCFileName << sfOutFileHead << "PC_" << pcFrameCount << ".txt";

        outPCFileFlag = false;

        // print output file generation message
        std::cout << "[*] Attention, a point cloud recording file is created in " << outPCFileName.str() << std::endl;
    }

    // output
    outPCFile.open(outPCFileName.str(), std::ios::out | std::ios::app);

    // output in a txt file
    // the storage type of output file is x y z time frames
    // record the point clouds
    for (int i = 0; i != vCloud.size(); ++i) {
        // output in a txt file
        // the storage type of output file is x y z time frames
        // right/left_sensor
        outPCFile << vCloud.points[i].x << " " << vCloud.points[i].y << " " << vCloud.points[i].z << " " << vFeatures[i]
                  << " " << pcFrameCount << " " << std::endl;
    } // end for

    outPCFile.close();

    // count new point cloud input (plus frame)
}
