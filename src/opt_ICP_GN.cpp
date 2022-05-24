#include "opt_ICP_GN.h"
#include "../3rdparty/sophus/se3.hpp"

namespace TESTICP
{
    opt_ICP_GN::opt_ICP_GN(const YAML::Node &node)
        : kdtree_flann(new pcl::KdTreeFLANN<PointType>)
    {

        max_iterations = node["max_iter"].as<int>();
        max_coresspoind_dis = node["max_corr_dist"].as<float>();
        trans_eps = node["trans_eps"].as<float>();
        euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    }

    opt_ICP_GN::~opt_ICP_GN()
    {
    }

    bool opt_ICP_GN::setTargetCloud(const CLOUD_PTR &target)
    {
        target_ptr = target;
        kdtree_flann->setInputCloud(target);
    }

    bool opt_ICP_GN::scanMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                               CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {
        source_ptr = source;
        CLOUD_PTR transform_cloud(new CLOUD());
        Eigen::Matrix4f T = predict_pose;
        for (int i = 0; i < max_iterations; ++i)
        {
            pcl::transformPointCloud(*source_ptr, *transform_cloud, T);
            Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();
            Eigen::Matrix<float, 6, 1> B = Eigen::Matrix<float, 6, 1>::Zero();

            // std::cout << "------------ " << i << "------------" << std::endl;
            for (int j = 0; j < transform_cloud->size(); ++j)
            {
                const PointType &origin_pt = source_ptr->points[j];
                if (!pcl::isFinite(origin_pt))
                    continue;

                const PointType &transform_pt = transform_cloud->at(j);
                std::vector<float> res_dis;
                std::vector<int> indices;
                kdtree_flann->nearestKSearch(transform_pt, 1, indices, res_dis);
                if (res_dis.front() > max_coresspoind_dis)
                    continue;

                Eigen::Vector3f nearest_pt = Eigen::Vector3f(target_ptr->at(indices.front()).x,
                                                             target_ptr->at(indices.front()).y,
                                                             target_ptr->at(indices.front()).z);
                Eigen::Vector3f point_eigen(transform_pt.x, transform_pt.y, transform_pt.z);
                Eigen::Vector3f origin_eigen(origin_pt.x, origin_pt.y, origin_pt.z);
                Eigen::Vector3f error = point_eigen - nearest_pt;

                Eigen::Matrix<float, 3, 6> J = Eigen::Matrix<float, 3, 6>::Zero();

                J.leftCols(3) = Eigen::Matrix3f::Identity();
                J.rightCols(3) = -T.block<3, 3>(0, 0) * Sophus::SO3f::hat(origin_eigen); //  右乘扰动
                // J.rightCols(3) = -Sophus::SO3f::hat(T.block<3, 3>(0, 0) * origin_eigen); //  左乘扰动

                H += J.transpose() * J;
                B += -J.transpose() * error;
            }
            if (H.determinant() == 0)
                continue;

            //  FIXME: 这里得到的位姿为李代数形式,(平移在前,旋转在后)
            // Eigen::Matrix<float, 6, 1> delta_x = H.inverse() * B;
            Eigen::Matrix<float, 6, 1> delta_x = H.ldlt().solve(B);

            Sophus::SE3f SE3_Rt(T);
            Sophus::SE3f u_T = SE3_Rt * Sophus::SE3f::exp(delta_x);
            //  FIXME: 如下表达,是否严谨
            T.block<3, 1>(0, 3) = T.block<3, 1>(0, 3) + delta_x.head(3);
            T.block<3, 3>(0, 0) *= Sophus::SO3f::exp(delta_x.tail(3)).matrix();

            // std::cout << "T\n"
            //           << T << std::endl;
            // std::cout << "T-se3: \n"
            //           << u_T.matrix() << std::endl;
        }

        final_pose = T;
        result_pose = T;
        pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
        return true;
    }

    float opt_ICP_GN::getFitnessScore()
    {
        float max_range = std::numeric_limits<float>::max();
        float score = 0.f;

        CLOUD_PTR transform_cloud(new CLOUD());
        pcl::transformPointCloud(*source_ptr, *transform_cloud, final_pose);
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);

        int nr = 0;

        for (size_t i = 0; i < transform_cloud->size(); ++i)
        {
            kdtree_flann->nearestKSearch(transform_cloud->points[i], 1, nn_indices, nn_dists);
            if (nn_dists.front() <= max_range)
            {
                score += nn_dists.front();
                nr++;
            }
        }
        if (nr > 0)
            return score / static_cast<float>(nr);
        else
            return (std::numeric_limits<float>::max());
    }
}