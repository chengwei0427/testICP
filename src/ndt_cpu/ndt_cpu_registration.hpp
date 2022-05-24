#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_

#include "registration_interface.hpp"
#include "ndt_cpu/NormalDistributionsTransform.h"

namespace TESTICP
{

  class NDTCPURegistration : public RegistrationInterface
  { // 继承点云配准的基类
  public:
    NDTCPURegistration(const YAML::Node &node);

    bool setTargetCloud(const CLOUD_PTR &input_target) override;
    bool scanMatch(
        const CLOUD_PTR &input_source,
        const Eigen::Matrix4f &predict_pose,
        CLOUD_PTR &result_cloud_ptr,
        Eigen::Matrix4f &result_pose) override;

  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    cpu::NormalDistributionsTransform<PointType, PointType> ndt_cpu_; // 实例化cpu_ndt 对象
  };

} // namespace lidar_localization

#endif