cmake_minimum_required(VERSION 3.1)

project(testicp)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread system) # Specify the Boost components you need
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src)

set(THIRD_PARTY 
${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES} pthread g2o_core ${CERES_LIBRARIES}
)

add_library(UTILS SHARED 
${PROJECT_SOURCE_DIR}/src/icp_registration.cpp
${PROJECT_SOURCE_DIR}/src/icp_svd_registration.cpp
${PROJECT_SOURCE_DIR}/src/ndt_registration.cpp
${PROJECT_SOURCE_DIR}/src/opt_ICP_CERES.cpp
${PROJECT_SOURCE_DIR}/src/opt_ICP_G2O.cpp
${PROJECT_SOURCE_DIR}/src/opt_ICP_GN.cpp

${PROJECT_SOURCE_DIR}/src/lidarOptimization/lidarCeres.cpp

${PROJECT_SOURCE_DIR}/src/ndt_cpu/ndt_cpu_registration.cpp
${PROJECT_SOURCE_DIR}/src/ndt_cpu/NormalDistributionsTransform.cpp
${PROJECT_SOURCE_DIR}/src/ndt_cpu/Octree.cpp
${PROJECT_SOURCE_DIR}/src/ndt_cpu/Registration.cpp
${PROJECT_SOURCE_DIR}/src/ndt_cpu/VoxelGrid.cpp

${PROJECT_SOURCE_DIR}/src/sicp/sicp_registration.cpp
)
target_link_libraries(UTILS ${THIRD_PARTY})

add_executable(icp ${PROJECT_SOURCE_DIR}/app/test_optimize_icp_node.cpp)
target_link_libraries(icp UTILS)

