
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "cloud_data.h"
namespace TESTICP
{
  class RegistrationInterface
  {
  public:
    virtual ~RegistrationInterface() = default;

    virtual bool setTargetCloud(const CLOUD_PTR &input_target) = 0;
    virtual bool scanMatch(const CLOUD_PTR &input_source,
                           const Eigen::Matrix4f &predict_pose,
                           CLOUD_PTR &result_cloud_ptr,
                           Eigen::Matrix4f &result_pose) = 0;
  };
}

#endif