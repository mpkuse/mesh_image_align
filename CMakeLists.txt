cmake_minimum_required (VERSION 2.8)
project (proto_app)
set( CMAKE_CXX_STANDARD 11 )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system program_options filesystem REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

find_package(Theia REQUIRED)
include_directories(${THEIA_INCLUDE_DIRS})

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  utils
)



FILE(GLOB KuseUtilSources
            utils/MiscUtils.cpp
            utils/RawFileIO.cpp
            utils/PoseManipUtils.cpp
            utils/CameraGeometry.cpp
            #PinholeCamera.cpp
    )


include_directories(utils/camodocal/include)
FILE(GLOB CamodocalCameraModelSources
        utils/camodocal/src/chessboard/Chessboard.cc
        utils/camodocal/src/calib/CameraCalibration.cc
        utils/camodocal/src/calib/StereoCameraCalibration.cc
        utils/camodocal/src/camera_models/Camera.cc
        utils/camodocal/src/camera_models/CameraFactory.cc
        utils/camodocal/src/camera_models/CostFunctionFactory.cc
        utils/camodocal/src/camera_models/PinholeCamera.cc
        utils/camodocal/src/camera_models/CataCamera.cc
        utils/camodocal/src/camera_models/EquidistantCamera.cc
        utils/camodocal/src/camera_models/ScaramuzzaCamera.cc
        utils/camodocal/src/sparse_graph/Transform.cc
        utils/camodocal/src/gpl/gpl.cc
        utils/camodocal/src/gpl/EigenQuaternionParameterization.cc
    )


add_executable( main main.cpp ${KuseUtilSources} ${CamodocalCameraModelSources} )

target_link_libraries( main
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
${Boost_PROGRAM_OPTIONS_LIBRARY}
${Boost_SYSTEM_LIBRARY}
${THEIA_LIBRARIES}
        )
