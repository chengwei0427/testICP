#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "registration_interface.hpp"

namespace TESTICP
{
  class ICPRegistration : public RegistrationInterface
  {
  public:
    ICPRegistration(const YAML::Node &node);
    ICPRegistration(
        float max_corr_dist,
        float trans_eps,
        float euc_fitness_eps,
        int max_iter);

    bool setTargetCloud(const CLOUD_PTR &input_target) override;
    bool scanMatch(const CLOUD_PTR &input_source,
                   const Eigen::Matrix4f &predict_pose,
                   CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4f &result_pose) override;

  private:
    bool SetRegistrationParam(
        float max_corr_dist,
        float trans_eps,
        float euc_fitness_eps,
        int max_iter);

  private:
    pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp_ptr_;
  };
}
#endif