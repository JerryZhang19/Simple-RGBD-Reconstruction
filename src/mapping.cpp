//
// Created by jerry on 4/18/20.
//

#include "myslam/mapping.h"


namespace simpleslam{
    Mapping::Mapping()
    {
        pcd_viewer = std::make_shared<pcl::visualization::CloudViewer>("Point Cloud Viewer");
        dense_map = PointCloud::Ptr(new PointCloud);
    }


    bool Mapping::merge_with(Frame::Ptr frame,Camera::Ptr camera) {

        auto depth_ptr = std::make_shared<cv::Mat>(frame->depth_);
        auto color_ptr = std::make_shared<cv::Mat>(frame->color_);
        int height = depth_ptr->rows;
        int width = depth_ptr->cols;

        PointCloud::Ptr current(new PointCloud);

        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                unsigned short depth_val = depth_ptr->at<short>(y, x);
                if (depth_val <= camera->max_depth && depth_val >= camera->min_depth) {
                    Vec3 pointWorld = camera->pixel2world(Vec2(x, y), frame->Pose(), double(depth_val) / 1000);
                    PointT p;
                    p.x = pointWorld[0];
                    p.y = pointWorld[1];
                    p.z = pointWorld[2];
                    p.b = color_ptr->data[y * color_ptr->step + x * color_ptr->channels()];
                    p.g = color_ptr->data[y * color_ptr->step + x * color_ptr->channels() + 1];
                    p.r = color_ptr->data[y * color_ptr->step + x * color_ptr->channels() + 2];
                    current->push_back(p);
                }
            }
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<Mapping::PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);
        (*dense_map) +=*tmp;

        pcl::VoxelGrid<PointT> voxel_filter;
        double resolution = 0.03;
        voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
        Mapping::PointCloud::Ptr tmp2(new PointCloud);
        voxel_filter.setInputCloud(dense_map);
        voxel_filter.filter(*tmp2);
        tmp2->swap(*dense_map);
        LOG(INFO)<<"point cloud size:"<<dense_map->size();
    }

    Mapping::PointCloud::Ptr Mapping::get_pcd(Frame::Ptr frame,Camera::Ptr camera) {

        auto result_pcd =  PointCloud::Ptr(new PointCloud);

        auto depth_ptr = std::make_shared<cv::Mat>(frame->depth_);
        auto color_ptr = std::make_shared<cv::Mat>(frame->color_);
        int height = depth_ptr->rows;
        int width = depth_ptr->cols;
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                unsigned short depth_val = depth_ptr->at<short>(y, x);
                if (depth_val <= camera->max_depth && depth_val >= camera->min_depth) {
                    Vec3 pointWorld = camera->pixel2world(Vec2(x, y), frame->Pose(), double(depth_val) / 1000);
                    PointT p;
                    p.x = pointWorld[0];
                    p.y = pointWorld[1];
                    p.z = pointWorld[2];
                    p.b = color_ptr->data[y * color_ptr->step + x * color_ptr->channels()];
                    p.g = color_ptr->data[y * color_ptr->step + x * color_ptr->channels() + 1];
                    p.r = color_ptr->data[y * color_ptr->step + x * color_ptr->channels() + 2];
                    result_pcd->push_back(p);
                }
            }
        return result_pcd;
    }

}