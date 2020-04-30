
add_library(simpleslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        IO.cpp
        mapping.cpp
        conversion.cpp)

#target_include_directories(simpleslam PUBLIC ${Open3D_INCLUDE_DIRS} )

target_link_libraries(simpleslam
        ${THIRD_PARTY_LIBS})
