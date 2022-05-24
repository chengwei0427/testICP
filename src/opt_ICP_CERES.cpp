#include "opt_ICP_CERES.h"
#include "lidarOptimization/lidarCeres.h"

namespace TESTICP
{
    opt_ICP_CERES::opt_ICP_CERES(const YAML::Node &node)
        : kdtree_flann(new pcl::KdTreeFLANN<PointType>)
    {

        max_iterations = node["max_iter"].as<int>();
        max_coresspoind_dis = node["max_corr_dist"].as<float>();
        trans_eps = node["trans_eps"].as<float>();
        euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    }

    opt_ICP_CERES::~opt_ICP_CERES()
    {
    }

    bool opt_ICP_CERES::setTargetCloud(const CLOUD_PTR &target)
    {
        target_ptr = target;
        kdtree_flann->setInputCloud(target);
    }

    bool opt_ICP_CERES::scanMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                                  CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {
        source_ptr = source;
        CLOUD_PTR transform_cloud(new CLOUD());
        Eigen::Matrix4d T = predict_pose.cast<double>();
        q_w_curr = Eigen::Quaterniond(T.block<3, 3>(0, 0));
        t_w_curr = T.block<3, 1>(0, 3);

        for (int i = 0; i < max_iterations; ++i)
        {
            pcl::transformPointCloud(*source_ptr, *transform_cloud, T);
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new test_ceres::PoseSE3Parameterization());

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

                Eigen::Vector3d nearest_pt = Eigen::Vector3d(target_ptr->at(indices.front()).x,
                                                             target_ptr->at(indices.front()).y,
                                                             target_ptr->at(indices.front()).z);

                Eigen::Vector3d origin_eigen(origin_pt.x, origin_pt.y, origin_pt.z);

                ceres::CostFunction *cost_function = new test_ceres::EdgeAnalyticCostFuntion(origin_eigen, nearest_pt);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 10;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            T.setIdentity();
            T.block<3, 1>(0, 3) = t_w_curr;
            T.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();

            // std::cout << "T\n"
            //           << T << std::endl;
        }

        final_pose = T.cast<float>();
        result_pose = T.cast<float>();
        pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
        return true;
    }

    float opt_ICP_CERES::getFitnessScore()
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