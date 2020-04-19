//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace simpleslam {

inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); } //simple helper function

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);
    mapping_ = Mapping::Ptr(new Mapping());

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCamera(dataset_->GetCamera(0));

    backend_->SetMap(map_);
    backend_->SetCamera(dataset_->GetCamera(0));

    viewer_->SetMap(map_);


    return true;
}

void VisualOdometry::Run() {
    while (1) {
        //LOG(INFO) << "VO is running";
        if (Step() == false) {
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();



    if(save_pose_)
        dataset_->SavePose(new_frame);


    if(save_point_cloud)
    {
        auto t3 = std::chrono::steady_clock::now();
        auto pcd = mapping_->get_pcd(new_frame,dataset_->GetCamera(0));
        auto t4 = std::chrono::steady_clock::now();
        //mapping_->pcd_viewer->showCloud(mapping_->pcd);
        dataset_->SavePointCloud(pcd);
        timer2  +=  std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count();
    }

    auto t5 = std::chrono::steady_clock::now();

    timer1  +=  std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    timer3  +=  std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t1).count();
    if(dataset_->GetIndex()%30==0)  //every 30 frames
    {
        LOG(INFO) << "VO cost time: " << timer1/30 << " seconds.";
        if(save_point_cloud)
            LOG(INFO) << "Add Point Cloud cost time: " << timer2/30 << " seconds.";
        LOG(INFO) << "Time per frame: " << timer3/30 << " seconds.";
        LOG(INFO) << "Frame rate: " << 30/timer3 ;
        timer1=timer2=timer3=0;
    }
    return success;
}

}  // namespace myslam
