//
// Created by jerry on 4/18/20.
//

#include "myslam/mapping.h"


namespace simpleslam{
    Mapping::Mapping()
    {
        pcd = PointCloud::Ptr(new PointCloud);
    }

    bool Mapping::merge_with(Frame::Ptr frame,Camera::Ptr camera)
    {
        auto R = frame->Pose().rotationMatrix();
        auto t = frame->Pose().translation();
        auto depth_ptr = std::make_shared<cv::Mat>(frame->depth_);
        auto color_ptr = std::make_shared<cv::Mat>(frame->img_);
        int height = depth_ptr->rows;
        int width = depth_ptr->cols;
        for (int i=0;i<height;i++)
            for (int j=0;j<width;j++) {
                short depth_val = depth_ptr->at<short>(i, j);
                if(depth_val<=camera->max_depth&&depth_val>=camera->min_depth){
                    Eigen::Vector3d point;

                    PointT p;


                }
            }
    }




}