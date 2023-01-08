CMAKE_MINIMUM_REQUIRED( VERSION 3.16.3 )

PROJECT( desktop_robot )

SET( EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin" )

ADD_DEFINITIONS( -std=c++17 )

#find required eigen
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

#find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

#directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/cluster" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/data_transport" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/epoll" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/hardware_devices" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/localization" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/loop_closure" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/path_planning" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/path_tracking" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/pgo" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/time_synchronize" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/utils" )
INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include/visualization" )

AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/src/epoll src_file )
AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR} src_file )

#directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_EXECUTABLE( desktop_robot ${src_file} )
    
TARGET_LINK_LIBRARIES( desktop_robot ${OpenCV_LIBS} )
TARGET_LINK_LIBRARIES( desktop_robot pthread )
