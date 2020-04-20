#include "myslam/IO.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace simpleslam {

IO::IO(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

bool IO::Init() {
    // read camera intrinsics and extrinsics
    ifstream fin(dataset_path_ + "/calib.txt");
    if (!fin) {
        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
        return false;
    }
    int num_camera;
    fin>>num_camera;
    for (int i = 0; i < num_camera; ++i) {

        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        //K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) <<"camera intrisics:"<<new_camera->K();
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }
    fin.close();
    current_image_index_ = 0;

    if(realtime_)
        SetupRealsenseCamera();
    return true;
}

Frame::Ptr IO::NextFrame() {
    auto new_frame = Frame::CreateFrame();
    if(realtime_)
    {
        rs2::frameset frameset = pipe_->wait_for_frames();
        frameset = align_to_color_->process(frameset);
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        cv::Mat cv_color(color.get_height(),color.get_width(),CV_8UC3,(void*)color.get_data());
        cv::Mat cv_gray;
        cv::cvtColor(cv_color,cv_gray,cv::COLOR_BGR2GRAY);
        cv::Mat cv_depth(depth.get_height(),depth.get_width(),CV_16U,(void*)depth.get_data());


        new_frame->img_ = cv_gray;
        new_frame->color_ = cv_color;
        new_frame->depth_ = cv_depth;
    }
    else{
        boost::format img_fmt("%s/color/%05d.png");
        boost::format depth_fmt("%s/depth/%05d.png");
        cv::Mat img, depth,color;
        // read images
        color =
            cv::imread((img_fmt % dataset_path_  % current_image_index_).str(),
                       cv::IMREAD_UNCHANGED);
        cv::cvtColor(color,img,cv::COLOR_BGR2GRAY);
        depth =
            cv::imread((depth_fmt % dataset_path_  % current_image_index_).str(),
                       cv::IMREAD_UNCHANGED);

        if (img.data == nullptr || depth.data == nullptr) {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }
        new_frame->img_ = img;
        new_frame->depth_ = depth;
        new_frame->color_ = color;
    }
    current_image_index_++;
    return new_frame;
}

bool IO::SavePose(Frame::Ptr current_frame)
{

    boost::format pose_fmt("%s/pose/%05d.txt");
    SE3 pose = current_frame->Pose();
    std::ofstream myfile;
    myfile.open ((pose_fmt % dataset_path_  % current_image_index_).str());
    myfile << pose.rotationMatrix()<<'\n'<<pose.translation();
    myfile.close();
}

bool IO::SavePointCloud(Mapping::PointCloud::Ptr pcd)
{
    boost::format pcd_fmt("%s/pointcloud/%05d.pcd");
    pcl::io::savePCDFileASCII ((pcd_fmt % dataset_path_  % current_image_index_).str(), *pcd);
}

void IO::SetupRealsenseCamera() {
    pipe_ = std::make_shared<rs2::pipeline>();
    align_to_color_ = std::make_shared<rs2::align>(RS2_STREAM_COLOR);

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    pipe_->start(cfg);
}


}  // namespace myslam