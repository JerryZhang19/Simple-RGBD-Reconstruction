#include "myslam/dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace simpleslam {

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

bool Dataset::Init() {
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
    return true;
}

Frame::Ptr Dataset::NextFrame() {
    boost::format img_fmt("%s/color/%05d.png");
    boost::format depth_fmt("%s/depth/%05d.png");
    cv::Mat img, depth;
    // read images
    img =
        cv::imread((img_fmt % dataset_path_  % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    depth =
        cv::imread((depth_fmt % dataset_path_  % current_image_index_).str(),
                   cv::IMREAD_UNCHANGED);

    //std::cout<<"while reading, depth="<<depth.at<unsigned short>(cv::Point(100,100));
    //cv::imshow("imput",img);
    //cv::waitKey(0);

    if (img.data == nullptr || depth.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    //cv::Mat image_left_resized, image_right_resized;
    //cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
    //           cv::INTER_NEAREST);
    //cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
    //           cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->img_ = img;
    new_frame->depth_ = depth;
    current_image_index_++;
    return new_frame;
}

bool Dataset::SavePose(Frame::Ptr current_frame)
{
    boost::format pose_fmt("%s/pose/%05d.txt");
    SE3 pose = current_frame->Pose();
    std::ofstream myfile;
    myfile.open ((pose_fmt % dataset_path_  % current_image_index_).str());
    myfile << pose.rotationMatrix()<<'\n'<<pose.translation();
    myfile.close();
}

bool Dataset::SavePointCloud(Mapping::PointCloud::Ptr pcd)
{
    boost::format pcd_fmt("%s/pointcloud/%05d.pcd");
    pcl::io::savePCDFileASCII ((pcd_fmt % dataset_path_  % current_image_index_).str(), *pcd);
}

}  // namespace myslam