
#pragma once
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "../3rdparty/sophus/se3.hpp"

#include "registration_interface.hpp"

namespace TESTICP
{
    class opt_ICP_G2O : public RegistrationInterface
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        opt_ICP_G2O(const YAML::Node &node);
        ~opt_ICP_G2O();
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

    class ICPVertex : public g2o::BaseVertex<6, Sophus::SE3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void setToOriginImpl() override
        {
            _estimate = Sophus::SE3d();
        }
        virtual void oplusImpl(const double *update) override
        {
            Eigen::Matrix<double, 6, 1> delta_r;
            delta_r << update[0], update[1], update[2], update[3], update[4], update[5];
            // _estimate = Sophus::SE3d::exp(delta_r) * _estimate;  //  左乘
            _estimate = _estimate * Sophus::SE3d::exp(delta_r);
        }
        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}
    };

    class ICPEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, ICPVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ICPEdge(Eigen::Vector3d x) : BaseUnaryEdge(), _p(x) {}

        virtual void computeError() override
        {
            const ICPVertex *v = static_cast<const ICPVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            _error = T * _p - _measurement;
        }

        virtual void linearizeOplus() override
        {
            const ICPVertex *v = static_cast<const ICPVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            //  e =Tp-p  ,de/T的李代数
            _jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            _jacobianOplusXi.block<3, 3>(0, 3) = -(T.matrix()).block<3, 3>(0, 0) * Sophus::SO3d::hat(_p); //  右乘扰动
            // _jacobianOplusXi.block<3, 3>(0, 3) = -Sophus::SO3d::hat(T * _p); //  左乘扰动
        }

        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}

    private:
        Eigen::Vector3d _p;
    };
}