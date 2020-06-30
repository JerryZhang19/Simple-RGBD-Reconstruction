//
// Created by jerry on 4/18/20.
//

#include "myslam/mapping.h"

namespace simpleslam{
    Mapping::Mapping()
    {
        float voxel_length = 0.0007;
        dense_map = Map_Ptr(new TSDF(voxel_length,0.004,open3d::integration::TSDFVolumeColorType::RGB8));
    }

    bool Mapping::merge_with(Frame::Ptr frame,Camera::Ptr camera) {
        auto depth_ptr = std::make_shared<cv::Mat>(frame->depth_);
        auto color_ptr = std::make_shared<cv::Mat>(frame->color_);
        dense_map->Integrate(*to_o3d_RGBDImage(frame),*to_o3d_intrinsic(camera),frame->Pose().matrix());
    }



}