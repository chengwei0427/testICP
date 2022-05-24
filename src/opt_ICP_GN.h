
#pragma once
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "registration_interface.hpp"

namespace TESTICP
{
    class opt_ICP_GN : public RegistrationInterface
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        opt_ICP_GN(const YAML::Node &node);
        ~opt_ICP_GN();
        bool setTargetCloud(const CLOUD_PTR &target) override;
        bool scanMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                       CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose) override;

        float getFitnessScore();

    private:
        CLOUD_PTR target_ptr, source_ptr;
        Eigen::Matrix4f final_pose;
        int max_iterations;
        float max_coresspoind_dis;
        float trans_eps;
        float euc_fitness_eps;

        pcl::KdTreeFLANN<PointType>::Ptr kdtree_flann;
    };
}