#include "SimpleRecon.h"
#include "FrameRecon.h"
#include <iostream>
#include <ros/ros.h>

// glog
#include <glog/logging.h>


int main(int argc, char **argv) {

    // init glog
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_INFO); // 设置glog的输出级别，这里的含义是输出INFO级别以上的信息
    FLAGS_colorlogtostderr = true;               // 开启终端颜色区分

	// init ros
    ros::init(argc, argv, "simple_frame");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    bool UseGHPR;
    privateNode.param("use_ghpr", UseGHPR, true);

    std::unique_ptr<FrameRecon> FrameReBuilder(UseGHPR ? new FrameRecon(node, privateNode)
                                                       : new SimpleRecon(node, privateNode));

    FrameReBuilder->LazyLoading();

    ros::spin();
    google::ShutdownGoogleLogging();
    return 0;
}
