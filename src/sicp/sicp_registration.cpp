#include "sicp/scip_registration.hpp"

namespace TESTICP
{
    SICPRegistration::SICPRegistration(
        const YAML::Node &node)
    {
        // parse params:
        params_.p = node["p"].as<double>();
        params_.mu = node["mu"].as<double>();
        params_.alpha = node["alpha"].as<double>();
        params_.max_mu = node["max_mu"].as<double>();
        params_.max_icp = node["max_icp"].as<int>();
        params_.max_outer = node["max_outer"].as<int>();
        params_.max_inner = node["max_inner"].as<int>();
        params_.stop = node["stop"].as<double>();
    }

    bool SICPRegistration::setTargetCloud(const CLOUD_PTR &input_target)
    {
        input_target_ = input_target;

        return true;
    }

    // 输入为两帧点云，预测的下一帧的位姿，结果位姿
    bool SICPRegistration::scanMatch(
        const CLOUD_PTR &input_source,
        const Eigen::Matrix4f &predict_pose,
        CLOUD_PTR &result_cloud_ptr,
        Eigen::Matrix4f &result_pose)
    {
        input_source_ = input_source;

        // pre-process input source:
        CLOUD_PTR transformed_input_source(new CLOUD());
        pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

        //
        // TODO: second option -- adapt existing implementation
        //
        // TODO: format inputs for SICP:

        Eigen::Matrix3Xd source(3, transformed_input_source->size());
        Eigen::Matrix3Xd target(3, input_target_->size());

        for (int i = 0; i < transformed_input_source->size(); i++)
        {
            source(0, i) = transformed_input_source->points[i].x;
            source(1, i) = transformed_input_source->points[i].y;
            source(2, i) = transformed_input_source->points[i].z;
        }

        for (int i = 0; i < input_target_->size(); i++)
        {
            target(0, i) = input_target_->points[i].x;
            target(1, i) = input_target_->points[i].y;
            target(2, i) = input_target_->points[i].z;
        }
        // TODO: SICP registration:
        transformation_.setZero();

        Eigen::Affine3d result = SICP::point_to_point(source, target, params_);

        transformation_ = result.matrix().cast<float>();

        // set output:
        result_pose = transformation_ * predict_pose;
        pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

        // 这里需要更新上一帧的点云
        // SetInputTarget(result_cloud_ptr);

        return true;
    }
}