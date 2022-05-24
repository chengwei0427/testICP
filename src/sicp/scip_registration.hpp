
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_

#include <pcl/common/transforms.h>

#include "sicp/ICP.h"

#include "registration_interface.hpp"

namespace TESTICP
{
  class SICPRegistration : public RegistrationInterface
  {
  public:
    SICPRegistration(const YAML::Node &node);

    bool setTargetCloud(const CLOUD_PTR &input_target) override;
    bool scanMatch(
        const CLOUD_PTR &input_source,
        const Eigen::Matrix4f &predict_pose,
        CLOUD_PTR &result_cloud_ptr,
        Eigen::Matrix4f &result_pose) override;

  private:
    CLOUD_PTR input_target_;
    CLOUD_PTR input_source_;

    Eigen::Matrix4f transformation_;

    SICP::Parameters params_;
  };
}
#endif // LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_