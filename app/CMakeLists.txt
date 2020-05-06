
add_executable(realsense_simpleslam run_realsense.cpp)

#target_include_directories(realsense_simpleslam PUBLIC ${Open3D_INCLUDE_DIRS})
#link_directories(${Open3D_LIBRARY_DIRS})
target_link_libraries(realsense_simpleslam simpleslam ${THIRD_PARTY_LIBS})
