#include "opt_ICP_G2O.h"

namespace TESTICP
{
    opt_ICP_G2O::opt_ICP_G2O(const YAML::Node &node)
        : kdtree_flann(new pcl::KdTreeFLANN<PointType>)
    {

        max_iterations = node["max_iter"].as<int>();
        max_coresspoind_dis = node["max_corr_dist"].as<float>();
        trans_eps = node["trans_eps"].as<float>();
        euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    }

    opt_ICP_G2O::~opt_ICP_G2O()
    {
    }

    bool opt_ICP_G2O::setTargetCloud(const CLOUD_PTR &target)
    {
        target_ptr = target;
        kdtree_flann->setInputCloud(target);
    }

    bool opt_ICP_G2O::scanMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                                CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {
        source_ptr = source;
        CLOUD_PTR transform_cloud(new CLOUD());
        Eigen::Matrix4f T = predict_pose;
        for (int i = 0; i < max_iterations; ++i)
        {
            pcl::transformPointCloud(*source_ptr, *transform_cloud, T);

            typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
            typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

            // auto solver = new g2o::OptimizationAlgorithmGaussNewton(
            //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

            // typedef LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

            LinearSolverType *linearSolver = new LinearSolverType();
            // linearSolver->setBlockOrdering(false);
            BlockSolverType *blockSolver = new BlockSolverType(linearSolver);
            auto solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
            // auto solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

            g2o::SparseOptimizer opt;
            opt.setAlgorithm(solver);
            opt.setVerbose(false); //    打印
            //  增加顶点
            ICPVertex *v = new ICPVertex();
            v->setEstimate(Sophus::SE3d());
            v->setId(0);
            opt.addVertex(v);

            //  增加边
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

                // Eigen::Vector3d origin_eigen(origin_pt.x, origin_pt.y, origin_pt.z);
                Eigen::Vector3d point_eigen(transform_pt.x, transform_pt.y, transform_pt.z);

                //    加入边
                ICPEdge *edge = new ICPEdge(point_eigen);
                edge->setId(j);
                edge->setVertex(0, v);
                edge->setMeasurement(nearest_pt);
                edge->setInformation(Eigen::Matrix<double, 3, 3>::Identity());
                opt.addEdge(edge);
            }

            opt.initializeOptimization();
            opt.optimize(10);
            Sophus::SE3f T_se3 = v->estimate().cast<float>();
            std::cout << T_se3.matrix() << std::endl;

            T = T_se3.matrix() * T;
        }

        final_pose = T;
        result_pose = T;
        pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
        return true;
    }

    float opt_ICP_G2O::getFitnessScore()
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