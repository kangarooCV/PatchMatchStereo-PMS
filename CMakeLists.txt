cmake_minimum_required(VERSION 3.20)
project(PatchMatchStereo)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
set(OpenCV_DIR E:/software/opencv454/build)
find_package(OpenCV REQUIRED)

add_executable(PatchMatchStereo
        include/cost_computor.hpp
        include/PatchMatchStereo.h
        include/pms_propagation.h
        include/pms_types.h
        include/pms_util.h
        include/stdafx.h
        include/targetver.h
        src/PatchMatchStereo.cpp
        src/pms_propagation.cpp
        src/pms_util.cpp
        src/stdafx.cpp
        main.cpp include/utils.h src/utils.cpp)


target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})