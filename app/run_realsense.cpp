//
// Created by gaoxiang on 19-5-4.
// modified by jianwei zhang
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

//DEFINE_string(config_file, "../config/default.yaml", "config file path");

std::string str("../config/default.yaml");
int main(int argc, char **argv) {
    //google::ParseCommandLineFlags(&argc, &argv, true);
    
    simpleslam::VisualOdometry::Ptr vo(
        new simpleslam::VisualOdometry(str));
    vo->SavePose(false);
    vo->SavePointCloud(false);
    vo->SetRealtime(false);
    vo->SetBuildMap(true);
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
