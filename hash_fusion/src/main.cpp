#include "FramesFusion.h"
#include <glog/logging.h>
#include <memory>

int main(int argc, char** argv){

    // init glog
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_INFO); // 设置glog的输出级别，这里的含义是输出INFO级别以上的信息
    FLAGS_colorlogtostderr = true;               // 开启终端颜色区分

    ros::init(argc, argv, "frames_fusion");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    int iFusionMode = 0;
    // privateNode.param("fusion_mode", iFusionMode, 0);
    std::unique_ptr<FramesFusion> pFramesFusion;
    switch(iFusionMode) {
    // case 1: pFramesFusion.reset(new MultiResolutionFusion(node,privateNode)); break;
    // case 2: pFramesFusion.reset(new PreConvFusion(node,privateNode));         break;
    default: pFramesFusion.reset(new FramesFusion(node,privateNode));
    }

    pFramesFusion->LazyLoading();
    google::ShutdownGoogleLogging();
    ros::spin();

    return 0;

}

