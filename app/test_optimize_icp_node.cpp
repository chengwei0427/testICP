#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "opt_ICP_GN.h"
#include "icp_svd_registration.hpp"
#include "icp_registration.hpp"
#include "ndt_registration.hpp"
#include "sicp/scip_registration.hpp"
#include "ndt_cpu/ndt_cpu_registration.hpp"
#include "opt_ICP_CERES.h"
#include "opt_ICP_G2O.h"

bool InitRegistration(std::shared_ptr<TESTICP::RegistrationInterface> &reg_ptr, const YAML::Node &config);
void display_2_pc(const typename pcl::PointCloud<PointType>::Ptr &Cloud1, const typename pcl::PointCloud<PointType>::Ptr &Cloud2,
                  std::string displayname, int display_downsample_ratio);

int main()
{
    std::string config_file = "../config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file);
    std::string t_str = config_node["target_file"].as<std::string>();
    std::string s_str = config_node["source_file"].as<std::string>();
    std::cout << "load file: " << t_str << ", and " << s_str << std::endl;
    double ds_size = config_node["ds_size"].as<double>();

    CLOUD_PTR cloud_source(new CLOUD());
    CLOUD_PTR cloud_target(new CLOUD());
    if (pcl::io::loadPCDFile<PointType>(s_str, *cloud_source) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<PointType>(t_str, *cloud_target) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }

    std::shared_ptr<TESTICP::RegistrationInterface> reg_ptr;
    InitRegistration(reg_ptr, config_node);

    pcl::VoxelGrid<PointType> ds_sample;
    ds_sample.setLeafSize(ds_size, ds_size, ds_size);
    ds_sample.setInputCloud(cloud_source);
    ds_sample.filter(*cloud_source);
    ds_sample.setInputCloud(cloud_target);
    ds_sample.filter(*cloud_target);

    Eigen::AngleAxisd r_z(M_PI / 30, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd r_x(M_PI / 130, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
    Tr.block<3, 3>(0, 0) = (r_x * r_z).matrix();
    Tr.block<3, 1>(0, 3) = Eigen::Vector3d(0.1, -0.3, 0.2);
    pcl::transformPointCloud(*cloud_source, *cloud_target, Tr);
    std::cout << "origi T:\n"
              << Tr << std::endl;
    {
        Sophus::SE3d SE3_rt((Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(1, 0, 0)) * Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d(0, 0, 1))).matrix(), Eigen::Vector3d(1, -3, 2));
        std::cout << "SE3 rt:\n"
                  << SE3_rt.matrix() << std::endl;
        Sophus::SE3d SE3_update((r_x * r_z).matrix(), Eigen::Vector3d(0.1, -0.3, 0.2));
        std::cout << "SE3 delta:\n"
                  << SE3_update.matrix() << std::endl;
        std::cout << (SE3_rt * SE3_update).matrix() << "\n"
                  << (SE3_update * SE3_rt).matrix() << std::endl;

        Eigen::Matrix<double, 6, 1> se3;
        se3.setZero();
        se3(0, 0) = 0.1;
        se3(3, 0) = 0.2;
        std::cout << (Sophus::SE3d::exp(se3) * SE3_rt).matrix() << "\n"
                  << (SE3_rt * Sophus::SE3d::exp(se3)).matrix() << std::endl;
    }

    display_2_pc(cloud_source, cloud_target, "before", 1);
    CLOUD_PTR transformed_source(new CLOUD());
    Eigen::Matrix4f T;

    reg_ptr->setTargetCloud(cloud_target);
    reg_ptr->scanMatch(cloud_source, Eigen::Matrix4f::Identity(), transformed_source, T);
    std::cout << T << std::endl;
    display_2_pc(transformed_source, cloud_target, "after", 1);

    system("pause");
}

bool InitRegistration(std::shared_ptr<TESTICP::RegistrationInterface> &reg_ptr, const YAML::Node &config)
{
    std::string reg_method = config["registration_method"].as<std::string>();
    std::cout << "Registration method: " << reg_method << std::endl;
    if (reg_method == "ICP_GN")
        reg_ptr = std::make_shared<TESTICP::opt_ICP_GN>(config[reg_method]);
    else if (reg_method == "ICP_SVD")
        reg_ptr = std::make_shared<TESTICP::ICPSVDRegistration>(config[reg_method]);
    else if (reg_method == "NDT")
        reg_ptr = std::make_shared<TESTICP::NDTRegistration>(config[reg_method]);
    else if (reg_method == "NDT_CPU")
        reg_ptr = std::make_shared<TESTICP::NDTCPURegistration>(config[reg_method]);
    else if (reg_method == "ICP")
        reg_ptr = std::make_shared<TESTICP::ICPRegistration>(config[reg_method]);
    else if (reg_method == "SICP")
        reg_ptr = std::make_shared<TESTICP::SICPRegistration>(config[reg_method]);
    else if (reg_method == "ICP_CERES")
        reg_ptr = std::make_shared<TESTICP::opt_ICP_CERES>(config[reg_method]);
    else if (reg_method == "ICP_G2O")
        reg_ptr = std::make_shared<TESTICP::opt_ICP_G2O>(config[reg_method]);
    else
    {
        std::cout << "Not have this type" << std::endl;
        return false;
    }
    return true;
}

void display_2_pc(const typename pcl::PointCloud<PointType>::Ptr &Cloud1, const typename pcl::PointCloud<PointType>::Ptr &Cloud2,
                  std::string displayname, int display_downsample_ratio)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));

    viewer->setBackgroundColor(0, 0, 0);
    char t[256];
    std::string s;
    int n = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < Cloud1->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud1->points[i].x;
            pt.y = Cloud1->points[i].y;
            pt.z = Cloud1->points[i].z;
            pt.r = 0;
            pt.g = 125;
            pt.b = 255;
            pointcloud1->points.push_back(pt);
        }
    } // Golden

    viewer->addPointCloud(pointcloud1, "pointcloudT");

    for (size_t i = 0; i < Cloud2->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud2->points[i].x;
            pt.y = Cloud2->points[i].y;
            pt.z = Cloud2->points[i].z;
            pt.r = 255;
            pt.g = 123;
            pt.b = 0;
            pointcloud2->points.push_back(pt);
        }
    } // Silver

    viewer->addPointCloud(pointcloud2, "pointcloudS");

    cout << "Click X(close) to continue..." << endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}