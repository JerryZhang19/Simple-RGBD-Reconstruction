//
// Created by jerry on 4/18/20.
//

#ifndef SIMPLESLAM_MAPPING_H
#define SIMPLESLAM_MAPPING_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/dataset.h"
#include "myslam/camera.h"

namespace simpleslam{


class Mapping{
public:
    typedef std::shared_ptr<Mapping> Ptr;

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud::Ptr pcd;


    Mapping();
    bool merge_with(Frame::Ptr frame, Camera::Ptr camera);

};





};

#endif //SIMPLESLAM_MAPPING_H
