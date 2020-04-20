//
// Created by jerry on 4/18/20.
//

#ifndef SIMPLESLAM_MAPPING_H
#define SIMPLESLAM_MAPPING_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/camera.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

namespace simpleslam{


class Mapping{
public:
    typedef std::shared_ptr<Mapping> Ptr;

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud::Ptr dense_map;
    std::shared_ptr<pcl::visualization::CloudViewer> pcd_viewer;


    Mapping();
    bool merge_with(Frame::Ptr frame, Camera::Ptr camera);
    PointCloud::Ptr get_pcd(Frame::Ptr frame,Camera::Ptr camera);

};





};

#endif //SIMPLESLAM_MAPPING_H
