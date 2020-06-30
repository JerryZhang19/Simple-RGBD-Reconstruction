cmake_minimum_required(VERSION 2.8)
project(simpleslam)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -pthread")
if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp")
endif()
if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -framework OpenGL -framework GLUT")
    set(CMAKE_MACOSX_RPATH 1)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################



# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message("OPENCV INCLUDE DIRECTORY: ${OpenCV_INCLUDE_DIRS}")
MESSAGE("Opencv Library Dir: ${OpenCV_LIBRARY_DIRS}")
MESSAGE("OpenCV_DIR: ${OpenCV_DIR}")
MESSAGE("OpenCV_LIBS: ${OpenCV_LIBS}")

# pangolin
find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

#-------------------Open3D-------------------
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})

message("CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
message("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0 -fPIC -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

message("AFTER")
message("CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
message("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")


message("Open3D_C_FLAGS: ${Open3D_C_FLAGS}")
message("Open3D_CXX_FLAGS: ${Open3D_CXX_FLAGS}")
message("Open3D_EXE_LINKER_FLAGS: ${Open3D_EXE_LINKER_FLAGS}")
#-------------------Open3D-------------------

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

#PCL
#find_package(PCL  REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})



#MESSAGE(${Open3D_CXX_FLAGS})
#MESSAGE(${CMAKE_CXX_FLAGS})
#MESSAGE(${GTEST_BOTH_LIBRARIES})
#MESSAGE(${GLOG_LIBRARIES})
#MESSAGE(${GFLAGS_LIBRARIES})
#intel realsense
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        )
    list(APPEND THIRD_PARTY_LIBS 
        GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        realsense2
        ${Open3D_LIBRARIES}
        )
enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(app)
