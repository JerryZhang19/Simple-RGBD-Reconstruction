//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace simpleslam {

inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); } //simple helper function

Frontend::Frontend() {
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(simpleslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            RgbdInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    std::cout<< "Set frame " << current_frame_->id_ << " as keyframe "<<std::endl;
    std::cout<<"total key frame:"<<map_->GetAllKeyFrames().size()<<std::endl;
    //          << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    InitializeNewPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}


int Frontend::InitializeNewPoints()  {
    //std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_.size(); ++i) {
        if (current_frame_->features_[i]->map_point_.expired()) {
            // 左图的特征点未关联地图点，读出深度转为坐标
            Vec3 pworld = camera_->pixel2world(current_frame_->features_[i]->get_vec2(),
                                               camera_->pose_,// Identity SE3
                                               current_frame_->features_[i]->init_depth_); // use depth sensor only to init

            auto new_map_point = MapPoint::CreateNewMappoint();
            pworld = current_pose_Twc * pworld;
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_[i]);
            //new_map_point->AddObservation(current_frame_->features_[i]);
            current_frame_->features_[i]->map_point_ = new_map_point;
            map_->InsertMapPoint(new_map_point);
            cnt_triangulated_pts++;
        }
    }
    //LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}


int Frontend::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_.size(); ++i) {
        auto mp = current_frame_->features_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->features_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 25; //5 pixels
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    //LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
    //          << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    //LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}





int Frontend::TrackLastFrame() {
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px =
                camera_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // rejected by optimization, will be reused when keyframe inserted
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->img_, current_frame_->img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            auto position = cv::Point2d(kp.pt);
            position.x = position.x < 0. ? 0. : position.x;
            position.y = position.y < 0. ? 0. : position.y;
            position.x = position.x > current_frame_->depth_.cols-1. ? double(current_frame_->depth_.cols-1.) : position.x;
            position.y = position.y > current_frame_->depth_.rows-1. ? double(current_frame_->depth_.rows-1.) : position.y;

            int depth = current_frame_->depth_.at<unsigned short>(position);
            Feature::Ptr feature(new Feature(current_frame_, kp,double(depth)/1000.0));
            feature->map_point_ = last_frame_->features_[i]->map_point_;
            current_frame_->features_.push_back(feature);
            num_good_pts++;
        }
    }
    //LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}




bool Frontend::RgbdInit() {
    int num_features_left = DetectFeatures();
    //int num_coor_features = FindFeaturesInRight();
    if (num_features_left < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

int Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        auto position = cv::Point2d(kp.pt);
        int depth = current_frame_->depth_.at<unsigned short>(position);
        
        if(depth<=Frame::max_depth&&depth>=Frame::min_depth)
        {
            //std::cout<<"depth:"<<depth<<std::endl;
            current_frame_->features_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp,double(depth)/1000.0)));
            cnt_detected++;
        }
    }

    return cnt_detected;
}


bool Frontend::BuildInitMap() {
    //SE3 pose= camera_->pose();
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->features_.size(); ++i) {
        //if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        /*
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
        */
        Vec3 pworld = camera_->pixel2world(Vec2(current_frame_->features_[i]->position_.pt.x,
                current_frame_->features_[i]->position_.pt.y),
                camera_->pose_,// Identity SE3
                current_frame_->features_[i]->init_depth_); // use depth sensor only to init
        //std::cout<<"pworld:"<<pworld<<std::endl;

        auto new_map_point = MapPoint::CreateNewMappoint();
        new_map_point->SetPos(pworld);
        new_map_point->AddObservation(current_frame_->features_[i]);

        current_frame_->features_[i]->map_point_ = new_map_point;
        cnt_init_landmarks++;
        map_->InsertMapPoint(new_map_point);
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    std::cout << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

bool Frontend::Reset() {
    //LOG(INFO) << "Reset is not implemented. ";
    std::exit(0);
    return true;
}

}  // namespace myslam
