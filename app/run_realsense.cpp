//
// Created by gaoxiang on 19-5-4.
// modified by jianwei zhang
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char **argv) {
    //google::ParseCommandLineFlags(&argc, &argv, true);

    simpleslam::VisualOdometry::Ptr vo(
        new simpleslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->SavePose(false);
    vo->SavePointCloud(true);

    vo->Run();

    return 0;
}
