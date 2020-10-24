//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

using namespace myslam;

DEFINE_string(config_file, "./config/default.yaml", "config file path");


void vo_thread(myslam::VisualOdometry::Ptr vo){
    assert(vo->Init()==true);
    vo->Run();

}
int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
 
    vo->viewer_ = Viewer::Ptr(new Viewer);

    std::thread vo_t(vo_thread, vo);
    // assert(vo->Init() == true);
    vo->viewer_->ThreadLoop(); // 可视化在主线程

    // vo->Run();
    vo_t.join();

    return 0;
}
