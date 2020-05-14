//
// Created by jerry on 4/29/20.
//

#ifndef SIMPLESLAM_CONVERSION_H
#define SIMPLESLAM_CONVERSION_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/camera.h"
#include "Open3D/Integration/ScalableTSDFVolume.h"
#include "Open3D/Visualization/Utility/DrawGeometry.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"


namespace simpleslam {
    std::shared_ptr<open3d::geometry::RGBDImage> to_o3d_RGBDImage(std::shared_ptr<Frame> frame);
    std::shared_ptr<open3d::camera::PinholeCameraIntrinsic> to_o3d_intrinsic(std::shared_ptr<Camera> camera);

}
#endif //SIMPLESLAM_CONVERSION_H
