#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace test_ceres
{

    Eigen::Matrix3d skew(const Eigen::Vector3d &mat);

    void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t);

    class EdgeAnalyticCostFuntion : public ceres::SizedCostFunction<3, 7>
    {
    public:
        EdgeAnalyticCostFuntion(Eigen::Vector3d cur_pt_, Eigen::Vector3d near_pt_);
        virtual ~EdgeAnalyticCostFuntion() {}
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        Eigen::Vector3d cur_pt, near_pt;
    };

    class PoseSE3Parameterization : public ceres::LocalParameterization
    {
    public:
        PoseSE3Parameterization(){};
        virtual ~PoseSE3Parameterization(){};
        virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
        virtual bool ComputeJacobian(const double *x, double *jacobian) const;
        virtual int GlobalSize() const { return 7; }
        virtual int LocalSize() const { return 6; }
    };
}