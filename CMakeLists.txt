cmake_minimum_required(VERSION 3.5)
SET(CMAKE_CXX_STANDARD 14)

message("Current Cmake version is : " ${CMAKE_VERSION})
project(testOptICP)

########################
# set compile options  #
########################
#delte the cache to rebuild after changing the options

OPTION(BUILD_WITH_OPENCV "Build with OpenCV2 for Image related Processing" ON)
OPTION(DEBUG_ON "Turn on DEBUG Mode" OFF)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++14 -w -fopenmp -pthread")
SET(CMAKE_C_FLAGS "-O3 -std=c++14 -fopenmp -pthread")

IF(DEBUG_ON)
MESSAGE("[Debug Mode]")
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
ELSE()
MESSAGE("[Release Mode]")
SET(CMAKE_BUILD_TYPE "Release") 
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
ENDIF()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

########################
# find libs & include  #
########################


find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(3RDPARTY_DIR /home/cc/workspace/3rdparty)

#eigen
# set(EIGEN3_INCLUDE_DIR ${3RDPARTY_DIR}/Eigen/include/eigen3)
set(EIGEN3_FOUND true)
find_package(Eigen3 REQUIRED)
IF(EIGEN3_FOUND) 
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
MESSAGE("EIGEN3 [OK]:" ${EIGEN3_INCLUDE_DIR})
ENDIF(EIGEN3_FOUND)


find_package(Boost REQUIRED)

# set(PCL_DIR ${3RDPARTY_DIR}/pcl1.9/share/pcl-1.9)
# find_package(PCL 1.9 REQUIRED COMPONENTS io common features kdtree visualization segmentation filters sample_consensus)
find_package(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

#OpenCV2
IF(BUILD_WITH_OPENCV)
FIND_PACKAGE(OpenCV 3.0 REQUIRED QUIET)
IF(OPENCV_FOUND) 
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
MESSAGE("OPENCV [OK]: " ${OpenCV_INCLUDE_DIRS}) 
ADD_DEFINITIONS(-DOPENCV_ON)
ENDIF(OPENCV_FOUND)
ENDIF(BUILD_WITH_OPENCV)

find_package(Ceres REQUIRED PATHS ${3RDPARTY_DIR}/ceres-1.14 )

message(STATUS "ceres: " ${GERES_LIBRARIES})

# g2o
# find_package(G2O REQUIRED PATHS ${3RDPARTY_DIR}/g2o/g2o2017_CSparse)
set(G2O_INCLUDE_DIRS ${3RDPARTY_DIR}/g2o/g2o2017_CSparse/include)
set(G2O_LIB_DIR ${3RDPARTY_DIR}/g2o/g2o2017_CSparse/lib) 
SET(G2O_LIBRARIES g2o_cli g2o_core g2o_csparse_extension g2o_ext_freeglut_minimal 
             g2o_incremental g2o_interactive g2o_interface g2o_opengl_helper 
             g2o_parser g2o_simulator g2o_solver_cholmod g2o_solver_csparse 
             g2o_solver_dense g2o_solver_eigen g2o_solver_pcg g2o_solver_slam2d_linear 
             g2o_solver_structure_only g2o_stuff g2o_types_data g2o_types_icp 
             g2o_types_sba g2o_types_sclam2d g2o_types_sim3 g2o_types_slam2d
             g2o_types_slam2d_addons g2o_types_slam3d g2o_types_slam3d_addons g2o_viewer)
IF(G2O_FOUND)
    message("G2O lib is found:" ${G2O_INCLUDE_DIRS}) 
    message("G2O lib:" ${G2O_LIBRARIES})
ENDIF(G2O_FOUND)

# include folder
INCLUDE_DIRECTORIES(
        ${3RDPARTY_DIR}/yaml/include
        ${BOOST_HEADER_DIR}
        ${G2O_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/app)

link_directories( ${CERES_LIBRARY_DIRS} ${G2O_LIB_DIR})

# source folder
# AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src CPP_LIST)
# file(GLOB_RECURSE CPP_LIST "*.cpp")
file(GLOB_RECURSE CPP_LIST "${PROJECT_SOURCE_DIR}/src/*.cpp")
# file(GLOB_RECURSE NODE_CPP "app/*_node.cpp")

# MESSAGE("---- " ${NODE_CPP})

# list(REMOVE_ITEM CPP_LIST ${NODE_CPP})
set(SRC_LIST 
        ${CPP_LIST} )
MESSAGE("SRC_LIST is " ${SRC_LIST})

########################
#      link libs       #
########################
#link pcl lib (neccessary)
SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES} ${BOOST_LIBRARIES} ${3RDPARTY_DIR}/yaml/lib/libyaml-cpp.a)



IF(BUILD_WITH_OPENCV)
#link opencv (optional)
SET(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS})
ENDIF(BUILD_WITH_OPENCV)

########################
#   add executables    #
########################

# Change this line to include your own executable file
add_executable(test_optimize_icp app/test_optimize_icp_node.cpp ${SRC_LIST} )
# Link to teaserpp & Eigen3
target_link_libraries(test_optimize_icp Eigen3::Eigen  ${DEP_LIBS} ${Boost_LIBRARIES}  ${CERES_LIBRARIES} ${G2O_LIBRARIES})
