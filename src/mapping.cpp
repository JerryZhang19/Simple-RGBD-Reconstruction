//
// Created by jerry on 4/18/20.
//

#include "myslam/mapping.h"

namespace simpleslam{
    Mapping::Mapping()
    {
        dense_map = Map_Ptr(new TSDF(0.04,0.3,open3d::integration::TSDFVolumeColorType::RGB8));
    }

    bool Mapping::merge_with(Frame::Ptr frame,Camera::Ptr camera) {
        auto depth_ptr = std::make_shared<cv::Mat>(frame->depth_);
        auto color_ptr = std::make_shared<cv::Mat>(frame->color_);
        int height = depth_ptr->rows;
        int width = depth_ptr->cols;
        dense_map->Integrate(*to_o3d_RGBDImage(frame),*to_o3d_intrinsic(camera),frame->Pose().matrix());

        //LOG(INFO)<<"point cloud size:"<<dense_map->
    }



}