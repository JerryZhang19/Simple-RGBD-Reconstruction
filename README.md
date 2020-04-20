# A Simple Sparse Odometry Based RGBD Reconstruction

The project use Intel Realsense D435i RGB-D sensor. Pointcloud Reconstruction is working now. 

Odometry part uses optical flow tracking and bundle adjustment optimization using g2o.

## Ongoing work
Other reconstruction method like TSDF and Mesh

GPU Reconstruction and realtime paralell reconstruction and localization.

## Future work

Loop Closure

Sparse map saving in a way suitable for relocalization.

## Related work

A realtime Visual Odometry using D435i: https://github.com/JerryZhang19/Realsense_RGBD_Odometry

## Reference

slambook2 https://github.com/gaoxiang12/slambook2 gives a great introduction to Visual SLAM.

Code framework of RGB-D Visual Odometry part is inherited from slambook2 ch13 (streo odometry).

## DEPENDENCY

* CMake,

        sudo apt-get install cmake

* google-glog + gflags,

        sudo apt-get install libgoogle-glog-dev
        
* Eigen3,

        sudo apt-get install libeigen3-dev

* SuiteSparse and CXSparse,

        sudo apt-get install libsuitesparse-dev

* Boost,

        sudo apt-get install libboost-dev libboost-filesystem-dev

* Pangolin, 

        git clone https://github.com/stevenlovegrove/Pangolin.git
        cd Pangolin
        mkdir build
        cd build
        cmake ..
        cmake --build .
        
* g2o, 

        https://github.com/RainerKuemmerle/g2o
* pcl,

        sudo apt-get install libpcl-dev pcl-tools
* librealsense,

        https://github.com/IntelRealSense/librealsense
       
      
