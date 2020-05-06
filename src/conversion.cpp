//
// Created by jerry on 4/29/20.
//

#include "myslam/conversion.h"

namespace simpleslam {
    std::shared_ptr<open3d::geometry::RGBDImage> to_o3d_RGBDImage(std::shared_ptr<Frame> frame)
    {
        int height = frame->color_.rows;
        int width = frame->color_.cols;

        auto color_im = std::make_shared<open3d::geometry::Image>();
        color_im->Prepare(width, height, 3, sizeof(uint8_t));
        uint8_t *pi = (uint8_t *)(color_im->data_.data());
        for (int i = 0; i < height; i++) {
            for (int k = 0; k < width; k++) {
                cv::Vec3b pixel = frame->color_.at<cv::Vec3b>(i, k);
                *pi++ = pixel[2];
                *pi++ = pixel[1];
                *pi++ = pixel[0];
            }
        }

        auto depth_im = std::make_shared<open3d::geometry::Image>();
        depth_im->Prepare(width, height, 1, sizeof(uint16_t));

        uint16_t * p = (uint16_t *)depth_im->data_.data();

        for (int i = 0; i < height; i++) {
            for (int k = 0; k < width; k++) {
                *p++ = frame->depth_.at<uint16_t>(i, k);
            }
        }

        auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(*color_im, *depth_im, 1000.0, 4.0, false);
        return rgbd_image;
    }


    std::shared_ptr<open3d::camera::PinholeCameraIntrinsic> to_o3d_intrinsic(std::shared_ptr<Camera> camera)
    {
        return std::make_shared<open3d::camera::PinholeCameraIntrinsic>(camera->width_, camera->height_, camera->fx_,camera->fy_,camera->cx_,camera->cy_);
    }

}