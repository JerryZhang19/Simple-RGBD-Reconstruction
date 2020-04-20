#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <librealsense2/rs.hpp>

#include "mapping.h"

namespace simpleslam {

/**
 * IO class, handles camera input or dataset
 * Init之后可获得相机和下一帧图像
 */
class IO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<IO> Ptr;
    IO(const std::string& dataset_path);

    /// 初始化，返回是否成功
    bool Init();
    bool Init_Camera();
    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

    bool SavePose(Frame::Ptr current_frame);
    bool SavePointCloud(Mapping::PointCloud::Ptr pcd);
    int GetIndex(){return current_image_index_;}
    void SetRealtime(bool flag){realtime_=flag;}
    void SetupRealsenseCamera();
   private:
    std::string dataset_path_;
    int current_image_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
    bool realtime_=false;

    std::shared_ptr<rs2::pipeline> pipe_=nullptr;
    std::shared_ptr<rs2::align> align_to_color_=nullptr;
};
}  // namespace myslam

#endif