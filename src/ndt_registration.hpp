
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "registration_interface.hpp"
namespace TESTICP
{
  class NDTRegistration : public RegistrationInterface
  {
  public:
    NDTRegistration(const YAML::Node &node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool setTargetCloud(const CLOUD_PTR &input_target) override;
    bool scanMatch(const CLOUD_PTR &input_source,
                   const Eigen::Matrix4f &predict_pose,
                   CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4f &result_pose) override;

  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    pcl::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_ptr_;
  };
}
#endif