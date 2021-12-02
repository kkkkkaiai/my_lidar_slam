//
// Created by znfs on 2021/11/28.
//
#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using std::string;
using std::vector;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Quaterniond;
using Eigen::Matrix3d;
using Eigen::Isometry3d; // 4d matrix

namespace helti {
    struct Extrinsics {
        Extrinsics(vector<double> &q, vector<double> &t)
//            : quaternion_(quaternion), translation_(translation)
        {
            Quaterniond quat(q[3], q[0], q[1], q[2]);
            quaternion_ << q[3], q[0], q[1], q[2];
            transform_ = Eigen::Isometry3d::Identity();
            transform_.rotate(quat.toRotationMatrix());
            translation_ << t[0], t[1], t[2];
            transform_.pretranslate(translation_);
        }

        Isometry3d Trans(Quaterniond &q) const {
            auto temp_T = Eigen::Isometry3d::Identity();
            temp_T.rotate(q.toRotationMatrix());
            temp_T = transform_ * temp_T;
            temp_T.pretranslate(translation_);
            return temp_T;
        }

        Isometry3d Trans(Isometry3d &Iso) const {
            Iso = transform_ * Iso;
            Iso.pretranslate(translation_);
            return Iso;
        }

        Isometry3d transform_;  // to base_link
        Vector4d quaternion_;  // to base_link
        Vector3d translation_; // to base_link
    };

    struct IMUIntrinsics {
        IMUIntrinsics(vector<double> & bias_a, vector<double> &bias_g, vector<double> & gravity)
        :bias_a_ (bias_a), bias_g_(bias_g), gravity_(gravity) {}

        vector<double> bias_a_;
        vector<double> bias_g_;
        vector<double> gravity_;
    };

    struct IMU : public Extrinsics, public IMUIntrinsics {
        IMU(vector<double>& q, vector<double> &t, vector<double> a = vector<double>(),
                vector<double> g = vector<double>(), vector<double> gravity = vector<double>())
                : Extrinsics(q, t ), IMUIntrinsics(a, g, gravity)
        {
            is_imu_ins_valid = false;
            if (!bias_a_.empty() && !bias_g_.empty() && !gravity_.empty()) {
                is_imu_ins_valid = true;
            }
        }

        bool is_imu_ins_valid;
    };

    struct OsLidar : public Extrinsics {
        OsLidar(vector<double> &q, vector<double> &t)
                : Extrinsics(q, t) {}
    };

    class LoadParam {
    public:
        LoadParam(const string& filename){
            YAML::Node config = YAML::LoadFile(filename);

            auto imu_quat = config["sensors"]["imu"]["extrinsics"]["quaternion"].as<vector<double>>();
            auto imu_trans = config["sensors"]["imu"]["extrinsics"]["translation"].as<vector<double>>();
            auto imu_bias_a = config["sensors"]["imu"]["intrinsics"]["parameters"]["bias_a"].as<vector<double>>();
            auto imu_bias_g = config["sensors"]["imu"]["intrinsics"]["parameters"]["bias_g"].as<vector<double>>();
            auto gravity = config["sensors"]["imu"]["intrinsics"]["parameters"]["gravity"].as<vector<double>>();

            auto os_lidar_quat = config["sensors"]["os_lidar"]["extrinsics"]["quaternion"].as<vector<double>>();
            auto os_lidar_trans = config["sensors"]["os_lidar"]["extrinsics"]["translation"].as<vector<double>>();

            imu_ = new IMU(imu_quat, imu_trans, imu_bias_a, imu_bias_g, gravity);
            os_lidar_ = new OsLidar(os_lidar_quat, os_lidar_trans);
        }


        inline static LoadParam HeltiParam() {
            auto temp_param = LoadParam("/home/znfs/2021/SLAM/src/calibration_02.yaml");
//            temp_param.os_lidar_->transform_ = temp_param.os_lidar_->Trans(temp_param.imu_->transform_);
            return temp_param;
        }

        IMU* imu_;
        OsLidar* os_lidar_;
    };

}