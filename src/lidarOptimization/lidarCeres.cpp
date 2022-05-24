#include <lidarOptimization/lidarCeres.h>

namespace test_ceres
{
    EdgeAnalyticCostFuntion::EdgeAnalyticCostFuntion(Eigen::Vector3d cur_pt_, Eigen::Vector3d near_pt_)
        : cur_pt(cur_pt_), near_pt(near_pt_) {}

    bool EdgeAnalyticCostFuntion::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
        Eigen::Vector3d lp = q_last_curr * cur_pt + t_last_curr;
        residuals[0] = lp.x() - near_pt.x();
        residuals[1] = lp.y() - near_pt.y();
        residuals[2] = lp.z() - near_pt.z();

        if (jacobians != NULL)
        {
            Eigen::Matrix3d skew_lp = skew(cur_pt);
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            // dp_by_se3.block<3, 3>(0, 0) = -q_last_curr.matrix() * skew_lp;  //  右乘扰动
            dp_by_se3.block<3, 3>(0, 0) = -skew(q_last_curr * cur_pt);
            (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<3, 6>(0, 0) = dp_by_se3;
        }

        return true;
    }

    bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        Eigen::Map<const Eigen::Vector3d> trans(x + 4);

        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_t;
        getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(delta), delta_q, delta_t);
        Eigen::Map<const Eigen::Quaterniond> quater(x);
        Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
        Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

        quater_plus = delta_q * quater;
        trans_plus = delta_q * trans + delta_t;
        return true;
    }

    bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        (j.topRows(6)).setIdentity();
        (j.bottomRows(1)).setZero();
        return true;
    }

    Eigen::Matrix<double, 3, 3> skew(const Eigen::Vector3d &mat)
    {
        Eigen::Matrix<double, 3, 3> skew_mat;
        skew_mat.setZero();
        skew_mat(0, 1) = -mat(2);
        skew_mat(0, 2) = mat(1);
        skew_mat(1, 2) = -mat(0);
        skew_mat(1, 0) = mat(2);
        skew_mat(2, 0) = -mat(1);
        skew_mat(2, 1) = mat(0);
        return skew_mat;
    }

    void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t)
    {
        Eigen::Vector3d omega(se3.data());
        Eigen::Vector3d upsilon(se3.data() + 3);
        Eigen::Matrix3d Omega = skew(omega);

        double theta = omega.norm();
        double half_theta = 0.5 * theta;

        double imag_factor;
        double real_factor = std::cos(half_theta);
        if (theta < 1e-10)
        {
            double theta_sq = theta * theta;
            double theta_po4 = theta_sq * theta_sq;
            imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
        }
        else
        {
            double sin_half_theta = sin(half_theta);
            imag_factor = sin_half_theta / theta;
        }
        q = Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());

        Eigen::Matrix3d J;
        if (theta < 1e-10)
            J = q.matrix();
        else
        {
            Eigen::Matrix3d Omega2 = Omega * Omega;
            J = (Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * Omega + (theta - std::sin(theta)) / (std::pow(theta, 3)) * Omega2);
        }
        t = J * upsilon;
    }

}