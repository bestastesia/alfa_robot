#include "alfa_robot_analytic_ik/v3_redundant_analytic_ik.hpp"

#include "alfa_robot_analytic_ik/analytic_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace alfa_robot::analytic_ik
{
namespace
{

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kUrdfPi = 3.1415927;
constexpr double kUrdfHalfPi = 1.5707963;
constexpr double kGeometryTolerance = 1e-8;
constexpr double kSingularityTolerance = 1e-10;

using JointVector = std::array<double, 7>;
using TransformArray = std::array<Eigen::Isometry3d, 7>;

const JointVector kLegacyLowerLimits = {
  -kPi,
  -1.83259571,
  -kPi,
  -2.61799388,
  -kPi,
  -2.09439510,
  -kPi,
};

const JointVector kLegacyUpperLimits = {
  kPi,
  1.83259571,
  kPi,
  2.61799388,
  kPi,
  2.09439510,
  kPi,
};

const JointVector kV305LowerLimits = {
  -kPi,
  -1.83259571,
  -kPi,
  -2.53072742,
  -kPi,
  -2.09439510,
  -kPi,
};

const JointVector kV305UpperLimits = {
  kPi,
  1.83259571,
  kPi,
  2.53072742,
  kPi,
  2.09439510,
  kPi,
};

const JointVector kV307LowerLimits = {
  -kPi,
  -1.83259571,
  -kPi,
  -2.53072742,
  -kPi,
  -1.91986218,
  -kPi,
};

const JointVector kV307UpperLimits = {
  kPi,
  1.83259571,
  kPi,
  2.53072742,
  kPi,
  1.91986218,
  kPi,
};

Eigen::Matrix3d rotX(double value)
{
  return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d rotY(double value)
{
  return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

Eigen::Matrix3d rotZ(double value)
{
  return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Isometry3d transformFromOrigin(
  const Eigen::Vector3d& translation,
  const Eigen::Vector3d& rpy)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  transform.linear() = rotZ(rpy.z()) * rotY(rpy.y()) * rotX(rpy.x());
  return transform;
}

Eigen::Isometry3d rotationTransform(const Eigen::Matrix3d& rotation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation;
  return transform;
}

Eigen::Matrix3d rotationAroundAxis(const Eigen::Vector3d& axis, double angle)
{
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

Eigen::Isometry3d rotationAroundLine(
  const Eigen::Vector3d& axis,
  const Eigen::Vector3d& point,
  double angle)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotationAroundAxis(axis, angle);
  transform.translation() = point - transform.linear() * point;
  return transform;
}

TransformArray fixedJointTransforms(V3RedundantArmModel model)
{
  if (model == V3RedundantArmModel::V311Left) {
    const Eigen::Isometry3d arm_mount = transformFromOrigin(
      {0.0, 0.0, -0.0142}, {0.0, 0.0, kPi / 2.0});
    return {
      arm_mount * transformFromOrigin(
        {0.32201385, -0.181, 1.3500054},
        {0.0, -1.3089969, kUrdfPi}),
      transformFromOrigin(
        {0.039999999, 0.0, 0.1512},
        {0.0, -kUrdfHalfPi, -kUrdfPi}),
      transformFromOrigin(
        {0.1795, 0.0, -0.040000001},
        {0.0, -kUrdfHalfPi, kUrdfPi}),
      transformFromOrigin(
        {-1.2261866e-09, -0.068, 0.3265},
        {kUrdfHalfPi, -1.3089969, -kUrdfPi}),
      transformFromOrigin(
        {0.22457775, 0.060175428, 0.068},
        {kUrdfHalfPi, 0.0, 1.8325957}),
      transformFromOrigin(
        {0.061499999, 0.0, 0.2415},
        {0.0, -kUrdfHalfPi, -kUrdfPi}),
      transformFromOrigin(
        {0.0993, 0.0, -0.0615},
        {0.0, -kUrdfHalfPi, kUrdfPi}),
    };
  }
  if (model == V3RedundantArmModel::V311Right) {
    const Eigen::Isometry3d arm_mount = transformFromOrigin(
      {0.0, 0.0, -0.0142}, {0.0, 0.0, kPi / 2.0});
    return {
      arm_mount * transformFromOrigin(
        {-0.33054221, -0.181, 1.3500054},
        {0.0, -1.3089969, 0.0}),
      transformFromOrigin(
        {0.039999999, 0.0, 0.1512},
        {0.0, kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {-0.1795, 0.0, -0.040000001},
        {0.0, -kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {-1.2287747e-09, 0.068, 0.3265},
        {kUrdfHalfPi, 1.3089969, 0.0}),
      transformFromOrigin(
        {-0.22457775, 0.060175428, 0.068},
        {-kUrdfHalfPi, 0.0, 1.3089969}),
      transformFromOrigin(
        {0.061499999, 0.0, 0.2415},
        {0.0, kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {-0.0993, 0.0, -0.0615},
        {0.0, -kUrdfHalfPi, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V309Left) {
    return {
      rotationTransform(rotZ(kPi / 2.0)) * transformFromOrigin(
        {0.35372443, -0.181, 1.309511},
        {0.0, -1.3089969, kUrdfPi}),
      transformFromOrigin(
        {0.039999999, 0.0, 0.1272},
        {0.0, -kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {0.1795, 0.0, 0.040000001},
        {0.0, kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {-1.2261866e-09, 0.07, 0.3265},
        {kUrdfHalfPi, -1.3089969, -kUrdfPi}),
      transformFromOrigin(
        {0.22457775, 0.060175428, -0.07},
        {-kUrdfHalfPi, 0.0, -1.30899695}),
      transformFromOrigin(
        {0.076499999, 0.0, 0.2405},
        {0.0, -kUrdfHalfPi, -kUrdfPi}),
      transformFromOrigin(
        {0.1025, 0.0, -0.0765},
        {0.0, kUrdfHalfPi, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V309Right) {
    return {
      rotationTransform(rotZ(kPi / 2.0)) * transformFromOrigin(
        {-0.33054221, -0.181, 1.3032993},
        {0.0, -1.3089969, 0.0}),
      transformFromOrigin(
        {0.039999999, 0.0, 0.1512},
        {0.0, kUrdfHalfPi, -kUrdfPi}),
      transformFromOrigin(
        {-0.1795, 0.0, 0.040000001},
        {0.0, kUrdfHalfPi, kUrdfPi}),
      transformFromOrigin(
        {-1.2287747e-09, -0.068, 0.3265},
        {kUrdfHalfPi, 1.3089969, 0.0}),
      transformFromOrigin(
        {-0.22457775, 0.060175428, -0.068},
        {-kUrdfHalfPi, 0.0, 1.3089969}),
      transformFromOrigin(
        {0.061499999, 0.0, 0.2405},
        {0.0, kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {-0.1025, 0.0, -0.0615},
        {0.0, -kUrdfHalfPi, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V308Left) {
    return {
      rotationTransform(rotZ(kPi / 2.0)) * transformFromOrigin(
        {-0.33054221, -0.181, 1.3032993},
        {0.0, -1.3089969, 0.0}),
      transformFromOrigin(
        {0.039999999, 0.0, 0.1512},
        {0.0, kUrdfHalfPi, -kUrdfPi}),
      transformFromOrigin(
        {-0.1795, 0.0, 0.040000001},
        {0.0, kUrdfHalfPi, kUrdfPi}),
      transformFromOrigin(
        {-1.2287747e-09, -0.068, 0.3265},
        {kUrdfHalfPi, 1.3089969, 0.0}),
      transformFromOrigin(
        {-0.22457775, 0.060175428, -0.068},
        {-kUrdfHalfPi, 0.0, 1.3089969}),
      transformFromOrigin(
        {0.061499999, 0.0, 0.2405},
        {0.0, kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {-0.1025, 0.0, -0.0615},
        {0.0, -kUrdfHalfPi, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V308Right) {
    return {
      rotationTransform(rotZ(kPi / 2.0)) * transformFromOrigin(
        {0.35372443, -0.181, 1.309511},
        {0.0, -1.3089969, kUrdfPi}),
      transformFromOrigin(
        {0.039999999, 0.0, 0.1272},
        {0.0, -kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {0.1795, 0.0, 0.040000001},
        {0.0, kUrdfHalfPi, 0.0}),
      transformFromOrigin(
        {-1.2261866e-09, 0.07, 0.3265},
        {kUrdfHalfPi, -1.3089969, -kUrdfPi}),
      transformFromOrigin(
        {0.22457775, 0.060175428, -0.07},
        {kUrdfHalfPi, 0.0, 1.8325957}),
      transformFromOrigin(
        {0.076499999, 0.0, 0.2405},
        {0.0, -kUrdfHalfPi, -kUrdfPi}),
      transformFromOrigin(
        {0.1025, 0.0, -0.0765},
        {0.0, -kUrdfHalfPi, kUrdfPi}),
    };
  }
  if (model == V3RedundantArmModel::V307Left) {
    return {
      transformFromOrigin(
        {0.18099999, -0.2995, 1.3228},
        {kUrdfHalfPi, 0.22548136, 0.0}),
      transformFromOrigin(
        {-0.021239679, 0.092595227, 0.112},
        {-kUrdfHalfPi, 0.0, 0.22548136}),
      transformFromOrigin(
        {3.328254e-10, -0.1795, -0.095},
        {kUrdfHalfPi, 0.22548136, 0.0}),
      transformFromOrigin(
        {0.066278689, 0.015203138, 0.3265},
        {kUrdfHalfPi, 0.0, 1.7962777}),
      transformFromOrigin(
        {-2.5261506e-10, 0.234, -0.068},
        {kUrdfHalfPi, -1.345315, -kUrdfPi}),
      transformFromOrigin(
        {-0.013749897, 0.059943226, 0.242},
        {-kUrdfHalfPi, 0.0, 0.22548136}),
      transformFromOrigin(
        {3.1333161e-10, -0.041, -0.0615},
        {kUrdfHalfPi, 0.22548136, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V307Right) {
    return {
      transformFromOrigin(
        {0.18099999, 0.2995, 1.3228},
        {-kUrdfHalfPi, 0.22548136, 0.0}),
      transformFromOrigin(
        {-0.021239679, -0.092595227, 0.112},
        {kUrdfHalfPi, 0.0, -0.22548136}),
      transformFromOrigin(
        {3.328254e-10, 0.246, -0.095},
        {-kUrdfHalfPi, 0.22548136, 0.0}),
      transformFromOrigin(
        {0.038987464, -0.0089430226, 0.26},
        {-kUrdfHalfPi, 0.0, -1.7962777}),
      transformFromOrigin(
        {-1.6265724e-10, -0.234, -0.04},
        {-kUrdfHalfPi, -1.345315, -kUrdfPi}),
      transformFromOrigin(
        {-0.013749897, -0.059943226, 0.242},
        {kUrdfHalfPi, 0.0, -0.22548136}),
      transformFromOrigin(
        {3.1333161e-10, 0.0995, -0.0615},
        {-kUrdfHalfPi, 0.22548136, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V306Left ||
      model == V3RedundantArmModel::V306Right) {
    const Eigen::Isometry3d arm_mount = model == V3RedundantArmModel::V306Left ?
      transformFromOrigin(
        {-0.055000007, -0.286, -1.417},
        {kUrdfHalfPi, -0.22548136, 0.0}) :
      transformFromOrigin(
        {-0.055000007, 0.296, -1.417},
        {-kUrdfHalfPi, -0.22548136, 0.0});
    return {
      arm_mount * transformFromOrigin({0.0, 0.0, 0.0045}, {0.0, 0.0, 0.0}),
      transformFromOrigin(
        {-0.02406893, -0.031948186, 0.112},
        {-kUrdfHalfPi, 0.0, -0.64565691}),
      transformFromOrigin(
        {0.0, -0.1795, 0.04},
        {kUrdfHalfPi, -0.64565691, 0.0}),
      transformFromOrigin(
        {0.061100907, -0.046031828, 0.3265},
        {-kUrdfHalfPi, 0.0, 0.92513942}),
      transformFromOrigin(
        {1.2528395e-10, -0.234, 0.0765},
        {kUrdfHalfPi, 0.92513942, 0.0}),
      transformFromOrigin(
        {-0.033696502, -0.044727461, 0.242},
        {-kUrdfHalfPi, 0.0, -0.64565691}),
      transformFromOrigin(
        {1.0640907e-10, -0.041, 0.056},
        {kUrdfHalfPi, -0.64565691, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V305Left) {
    return {
      transformFromOrigin(
        {-0.055000007, -0.2905, -1.417},
        {kUrdfHalfPi, -0.22548136, 0.0}),
      transformFromOrigin(
        {-0.049472837, 0.0072414368, 0.122},
        {kUrdfHalfPi, 0.0, -1.7161362}),
      transformFromOrigin(
        {2.5750223e-10, 0.1405, -0.05},
        {-kUrdfHalfPi, 1.4254565, -kUrdfPi}),
      transformFromOrigin(
        {-0.0091167376, -0.049161826, -0.3775},
        {kUrdfHalfPi, 0.0, -0.18336049}),
      transformFromOrigin(
        {1.2925199e-10, -0.254, -0.05},
        {kUrdfHalfPi, 0.18336049, 0.0}),
      transformFromOrigin(
        {0.016070053, 0.025332852, 0.223},
        {-kUrdfHalfPi, 0.0, -0.56529915}),
      transformFromOrigin(
        {0.0, -0.039, -0.03},
        {kUrdfHalfPi, -0.56529915, 0.0}),
    };
  }
  if (model == V3RedundantArmModel::V305Right) {
    return {
      transformFromOrigin(
        {-0.055000007, 0.3005, -1.417},
        {-kUrdfHalfPi, -0.22548136, 0.0}),
      transformFromOrigin(
        {0.047051538, -0.016916049, 0.112},
        {kUrdfHalfPi, 0.0, 1.2256642}),
      transformFromOrigin(
        {0.0, 0.246, -0.05},
        {-kUrdfHalfPi, -1.2256642, 0.0}),
      transformFromOrigin(
        {0.00082154954, 0.04999325, 0.272},
        {-kUrdfHalfPi, 0.0, -0.016431729}),
      transformFromOrigin(
        {0.0, -0.244, -0.05},
        {kUrdfHalfPi, -0.016431729, 0.0}),
      transformFromOrigin(
        {0.013852058, 0.014426382, 0.233},
        {-kUrdfHalfPi, 0.0, -0.76509136}),
      transformFromOrigin(
        {0.0, -0.036, -0.02},
        {kUrdfHalfPi, -0.76509136, 0.0}),
    };
  }
  return {
    transformFromOrigin({0.0, 0.0, 0.0}, {kUrdfPi, 0.0, 0.0}),
    transformFromOrigin(
      {0.047051538, 0.016916049, -0.117},
      {-kUrdfHalfPi, 0.0, -1.2256642}),
    transformFromOrigin(
      {0.0, 0.1405, -0.05},
      {-kUrdfHalfPi, -1.2256642, 0.0}),
    transformFromOrigin(
      {-0.00082154955, -0.04999325, 0.3775},
      {kUrdfHalfPi, 0.0, -0.016431729}),
    transformFromOrigin(
      {0.0, 0.241, -0.05},
      {-kUrdfHalfPi, 0.016431729, 0.0}),
    transformFromOrigin(
      {0.013852058, 0.014426382, 0.236},
      {-kUrdfHalfPi, 0.0, -0.76509136}),
    transformFromOrigin(
      {0.0, -0.039, -0.02},
      {kUrdfHalfPi, -0.76509136, 0.0}),
  };
}

Eigen::Isometry3d toolTransform(V3RedundantArmModel model)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (model == V3RedundantArmModel::V305Left) {
    transform.translation().z() = 0.1865;
  } else if (model == V3RedundantArmModel::V305Right) {
    transform.translation().z() = 0.1895;
  } else if (model == V3RedundantArmModel::V306Left ||
             model == V3RedundantArmModel::V306Right ||
             model == V3RedundantArmModel::V307Left ||
             model == V3RedundantArmModel::V307Right) {
    transform.translation().z() = 0.19435;
  } else if (model == V3RedundantArmModel::V308Left ||
             model == V3RedundantArmModel::V308Right ||
             model == V3RedundantArmModel::V309Left ||
             model == V3RedundantArmModel::V309Right ||
             model == V3RedundantArmModel::V311Left ||
             model == V3RedundantArmModel::V311Right) {
    transform.translation().z() = 0.13585;
  }
  return transform;
}

Eigen::Vector3d commonAxisCenter(
  const std::array<Eigen::Vector3d, 7>& axis_points,
  const std::array<Eigen::Vector3d, 7>& axes,
  const std::array<size_t, 3>& indices)
{
  Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (const size_t index : indices) {
    const Eigen::Vector3d axis = axes[index].normalized();
    const Eigen::Matrix3d projector =
      Eigen::Matrix3d::Identity() - axis * axis.transpose();
    lhs += projector;
    rhs += projector * axis_points[index];
  }
  return lhs.ldlt().solve(rhs);
}

struct Geometry
{
  TransformArray fixed_transforms{};
  std::array<Eigen::Vector3d, 7> axis_points{};
  std::array<Eigen::Vector3d, 7> axes{};
  Eigen::Isometry3d zero_tool = Eigen::Isometry3d::Identity();
  Eigen::Vector3d shoulder = Eigen::Vector3d::Zero();
  Eigen::Vector3d elbow = Eigen::Vector3d::Zero();
  Eigen::Vector3d wrist = Eigen::Vector3d::Zero();
  Eigen::Vector3d upper_arm_direction = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d forearm_direction = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d wrist_to_tool_in_tool = Eigen::Vector3d::Zero();
  double upper_arm_length = 0.0;
  double forearm_length = 0.0;
};

Geometry makeGeometry(V3RedundantArmModel model)
{
  Geometry geometry;
  geometry.fixed_transforms = fixedJointTransforms(model);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (size_t index = 0; index < geometry.fixed_transforms.size(); ++index) {
    transform = transform * geometry.fixed_transforms[index];
    geometry.axis_points[index] = transform.translation();
    geometry.axes[index] =
      (transform.linear() * Eigen::Vector3d::UnitZ()).normalized();
  }
  geometry.zero_tool = transform * toolTransform(model);
  geometry.shoulder = commonAxisCenter(
    geometry.axis_points, geometry.axes, {0, 1, 2});
  geometry.elbow = commonAxisCenter(
    geometry.axis_points, geometry.axes, {2, 3, 4});
  geometry.wrist = commonAxisCenter(
    geometry.axis_points, geometry.axes, {4, 5, 6});
  const Eigen::Vector3d upper_arm = geometry.elbow - geometry.shoulder;
  const Eigen::Vector3d forearm = geometry.wrist - geometry.elbow;
  geometry.upper_arm_length = upper_arm.norm();
  geometry.forearm_length = forearm.norm();
  geometry.upper_arm_direction = upper_arm.normalized();
  geometry.forearm_direction = forearm.normalized();
  geometry.wrist_to_tool_in_tool =
    geometry.zero_tool.linear().transpose() *
    (geometry.zero_tool.translation() - geometry.wrist);
  return geometry;
}

const Geometry& geometry(V3RedundantArmModel model)
{
  static const Geometry legacy = makeGeometry(V3RedundantArmModel::LegacyV304);
  static const Geometry v305_left = makeGeometry(V3RedundantArmModel::V305Left);
  static const Geometry v305_right = makeGeometry(V3RedundantArmModel::V305Right);
  static const Geometry v306_left = makeGeometry(V3RedundantArmModel::V306Left);
  static const Geometry v306_right = makeGeometry(V3RedundantArmModel::V306Right);
  static const Geometry v307_left = makeGeometry(V3RedundantArmModel::V307Left);
  static const Geometry v307_right = makeGeometry(V3RedundantArmModel::V307Right);
  static const Geometry v308_left = makeGeometry(V3RedundantArmModel::V308Left);
  static const Geometry v308_right = makeGeometry(V3RedundantArmModel::V308Right);
  static const Geometry v309_left = makeGeometry(V3RedundantArmModel::V309Left);
  static const Geometry v309_right = makeGeometry(V3RedundantArmModel::V309Right);
  static const Geometry v311_left = makeGeometry(V3RedundantArmModel::V311Left);
  static const Geometry v311_right = makeGeometry(V3RedundantArmModel::V311Right);
  if (model == V3RedundantArmModel::V311Left) {
    return v311_left;
  }
  if (model == V3RedundantArmModel::V311Right) {
    return v311_right;
  }
  if (model == V3RedundantArmModel::V309Left) {
    return v309_left;
  }
  if (model == V3RedundantArmModel::V309Right) {
    return v309_right;
  }
  if (model == V3RedundantArmModel::V308Left) {
    return v308_left;
  }
  if (model == V3RedundantArmModel::V308Right) {
    return v308_right;
  }
  if (model == V3RedundantArmModel::V307Left) {
    return v307_left;
  }
  if (model == V3RedundantArmModel::V307Right) {
    return v307_right;
  }
  if (model == V3RedundantArmModel::V306Left) {
    return v306_left;
  }
  if (model == V3RedundantArmModel::V306Right) {
    return v306_right;
  }
  if (model == V3RedundantArmModel::V305Left) {
    return v305_left;
  }
  if (model == V3RedundantArmModel::V305Right) {
    return v305_right;
  }
  return legacy;
}

const JointVector& lowerLimits(V3RedundantArmModel model)
{
  if (model == V3RedundantArmModel::V307Left ||
      model == V3RedundantArmModel::V307Right ||
      model == V3RedundantArmModel::V308Left ||
      model == V3RedundantArmModel::V308Right ||
      model == V3RedundantArmModel::V309Left ||
      model == V3RedundantArmModel::V309Right ||
      model == V3RedundantArmModel::V311Left ||
      model == V3RedundantArmModel::V311Right) {
    return kV307LowerLimits;
  }
  return model == V3RedundantArmModel::LegacyV304 ?
         kLegacyLowerLimits : kV305LowerLimits;
}

const JointVector& upperLimits(V3RedundantArmModel model)
{
  if (model == V3RedundantArmModel::V307Left ||
      model == V3RedundantArmModel::V307Right ||
      model == V3RedundantArmModel::V308Left ||
      model == V3RedundantArmModel::V308Right ||
      model == V3RedundantArmModel::V309Left ||
      model == V3RedundantArmModel::V309Right ||
      model == V3RedundantArmModel::V311Left ||
      model == V3RedundantArmModel::V311Right) {
    return kV307UpperLimits;
  }
  return model == V3RedundantArmModel::LegacyV304 ?
         kLegacyUpperLimits : kV305UpperLimits;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> swivelBasis(
  const Eigen::Vector3d& shoulder_to_wrist_direction)
{
  Eigen::Vector3d reference = Eigen::Vector3d::UnitX();
  Eigen::Vector3d first =
    reference - shoulder_to_wrist_direction *
    shoulder_to_wrist_direction.dot(reference);
  if (first.norm() < 1e-8) {
    reference = Eigen::Vector3d::UnitY();
    first = reference - shoulder_to_wrist_direction *
      shoulder_to_wrist_direction.dot(reference);
  }
  first.normalize();
  Eigen::Vector3d second = shoulder_to_wrist_direction.cross(first).normalized();
  return {first, second};
}

std::optional<double> orientedAngleAroundAxis(
  const Eigen::Vector3d& axis,
  const Eigen::Vector3d& from,
  const Eigen::Vector3d& to)
{
  const Eigen::Vector3d unit_axis = axis.normalized();
  Eigen::Vector3d projected_from = from - unit_axis * unit_axis.dot(from);
  Eigen::Vector3d projected_to = to - unit_axis * unit_axis.dot(to);
  if (projected_from.norm() < kSingularityTolerance ||
      projected_to.norm() < kSingularityTolerance) {
    return std::nullopt;
  }
  projected_from.normalize();
  projected_to.normalize();
  return std::atan2(
    unit_axis.dot(projected_from.cross(projected_to)),
    projected_from.dot(projected_to));
}

std::vector<double> signedBranches(double magnitude)
{
  if (std::abs(magnitude) < kSingularityTolerance) {
    return {0.0};
  }
  return {magnitude, -magnitude};
}

bool insideLimits(const JointVector& joints, V3RedundantArmModel model)
{
  const JointVector& lower = lowerLimits(model);
  const JointVector& upper = upperLimits(model);
  for (size_t index = 0; index < joints.size(); ++index) {
    if (joints[index] < lower[index] - 1e-9 ||
        joints[index] > upper[index] + 1e-9) {
      return false;
    }
  }
  return true;
}

double minimumLimitMargin(const JointVector& joints, V3RedundantArmModel model)
{
  const JointVector& lower = lowerLimits(model);
  const JointVector& upper = upperLimits(model);
  double margin = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < joints.size(); ++index) {
    margin = std::min(
      margin,
      std::min(
        joints[index] - lower[index],
        upper[index] - joints[index]));
  }
  return margin;
}

double seedDistance(const JointVector& joints, const JointVector& seed)
{
  double squared_distance = 0.0;
  for (size_t index = 0; index < joints.size(); ++index) {
    const double delta = normalizeAngle(joints[index] - seed[index]);
    squared_distance += delta * delta;
  }
  return std::sqrt(squared_distance);
}

bool sameJointSolution(const JointVector& lhs, const JointVector& rhs)
{
  double squared_distance = 0.0;
  for (size_t index = 0; index < lhs.size(); ++index) {
    const double delta = normalizeAngle(lhs[index] - rhs[index]);
    squared_distance += delta * delta;
  }
  return squared_distance < 1e-16;
}

double orientationError(
  const Eigen::Matrix3d& target,
  const Eigen::Matrix3d& actual)
{
  return std::abs(Eigen::AngleAxisd(target.transpose() * actual).angle());
}

Eigen::Vector3d pointAfterJointMotions(
  const Eigen::Vector3d& home_point,
  const JointVector& joints,
  size_t joint_count,
  V3RedundantArmModel model_kind)
{
  const Geometry& model = geometry(model_kind);
  Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
  for (size_t index = 0; index < joint_count; ++index) {
    motion = motion * rotationAroundLine(
      model.axes[index], model.axis_points[index], joints[index]);
  }
  return motion * home_point;
}

}  // namespace

V3RedundantArmAnalyticIk::V3RedundantArmAnalyticIk(V3RedundantArmModel model)
: model_(model)
{
}

std::vector<V3RedundantIkSolution> V3RedundantArmAnalyticIk::solveInArmBase(
  const V3RedundantIkRequest& request) const
{
  std::vector<V3RedundantIkSolution> solutions;
  if (!request.target_in_arm_base.matrix().allFinite()) {
    return solutions;
  }

  const Geometry& model = geometry(model_);
  const Eigen::Vector3d wrist =
    request.target_in_arm_base.translation() -
    request.target_in_arm_base.linear() * model.wrist_to_tool_in_tool;
  const Eigen::Vector3d shoulder_to_wrist = wrist - model.shoulder;
  const double shoulder_to_wrist_distance = shoulder_to_wrist.norm();
  if (shoulder_to_wrist_distance < kGeometryTolerance ||
      shoulder_to_wrist_distance >
        model.upper_arm_length + model.forearm_length + kGeometryTolerance ||
      shoulder_to_wrist_distance <
        std::abs(model.upper_arm_length - model.forearm_length) -
        kGeometryTolerance) {
    return solutions;
  }

  const Eigen::Vector3d shoulder_to_wrist_direction =
    shoulder_to_wrist / shoulder_to_wrist_distance;
  const double bounded_shoulder_to_wrist_distance = std::clamp(
    shoulder_to_wrist_distance,
    std::abs(model.upper_arm_length - model.forearm_length),
    model.upper_arm_length + model.forearm_length);
  const double circle_offset =
    (model.upper_arm_length * model.upper_arm_length -
     model.forearm_length * model.forearm_length +
     bounded_shoulder_to_wrist_distance * bounded_shoulder_to_wrist_distance) /
    (2.0 * bounded_shoulder_to_wrist_distance);
  const double circle_radius_squared =
    model.upper_arm_length * model.upper_arm_length -
    circle_offset * circle_offset;
  if (circle_radius_squared < -kGeometryTolerance) {
    return solutions;
  }
  const double circle_radius = std::sqrt(std::max(0.0, circle_radius_squared));
  const auto [swivel_first, swivel_second] =
    swivelBasis(shoulder_to_wrist_direction);
  const double swivel = normalizeAngle(request.swivel_angle);
  const Eigen::Vector3d elbow_circle_center =
    model.shoulder + circle_offset * shoulder_to_wrist_direction;
  const Eigen::Vector3d elbow =
    elbow_circle_center + circle_radius *
    (std::cos(swivel) * swivel_first +
     std::sin(swivel) * swivel_second);
  const Eigen::Vector3d upper_arm_direction =
    (elbow - model.shoulder).normalized();
  const Eigen::Vector3d forearm_direction = (wrist - elbow).normalized();

  const double q2_magnitude = std::acos(std::clamp(
    model.upper_arm_direction.dot(upper_arm_direction), -1.0, 1.0));
  for (const double q2 : signedBranches(q2_magnitude)) {
    const Eigen::Vector3d upper_before_q1 =
      rotationAroundAxis(model.axes[1], q2) * model.upper_arm_direction;
    const double q1 = normalizeAngle(
      orientedAngleAroundAxis(
        model.axes[0], upper_before_q1, upper_arm_direction)
        .value_or(request.seed[0]));
    const Eigen::Matrix3d rotation_12 =
      rotationAroundAxis(model.axes[0], q1) *
      rotationAroundAxis(model.axes[1], q2);
    const Eigen::Vector3d forearm_before_rotation_12 =
      rotation_12.transpose() * forearm_direction;
    const double q4_magnitude = std::acos(std::clamp(
      model.forearm_direction.dot(forearm_before_rotation_12),
      -1.0, 1.0));

    for (const double q4 : signedBranches(q4_magnitude)) {
      const Eigen::Vector3d forearm_before_q3 =
        rotationAroundAxis(model.axes[3], q4) * model.forearm_direction;
      const double q3 = normalizeAngle(
        orientedAngleAroundAxis(
          model.axes[2], forearm_before_q3, forearm_before_rotation_12)
          .value_or(request.seed[2]));
      const Eigen::Matrix3d rotation_1234 =
        rotation_12 *
        rotationAroundAxis(model.axes[2], q3) *
        rotationAroundAxis(model.axes[3], q4);
      const Eigen::Matrix3d wrist_rotation =
        rotation_1234.transpose() *
        request.target_in_arm_base.linear() *
        model.zero_tool.linear().transpose();
      const Eigen::Vector3d wrist_roll_axis = model.axes[4].normalized();
      const double q6_magnitude = std::acos(std::clamp(
        wrist_roll_axis.dot(wrist_rotation * wrist_roll_axis),
        -1.0, 1.0));

      for (const double q6 : signedBranches(q6_magnitude)) {
        const Eigen::Vector3d wrist_before_q5 =
          rotationAroundAxis(model.axes[5], q6) * wrist_roll_axis;
        const double q5 = normalizeAngle(
          orientedAngleAroundAxis(
            wrist_roll_axis,
            wrist_before_q5,
            wrist_rotation * wrist_roll_axis)
            .value_or(request.seed[4]));
        const Eigen::Matrix3d joint7_rotation =
          rotationAroundAxis(model.axes[5], -q6) *
          rotationAroundAxis(wrist_roll_axis, -q5) *
          wrist_rotation;
        Eigen::Vector3d wrist_bend_axis =
          model.axes[5] -
          wrist_roll_axis * wrist_roll_axis.dot(model.axes[5]);
        wrist_bend_axis.normalize();
        const double q7 = normalizeAngle(
          orientedAngleAroundAxis(
            wrist_roll_axis,
            wrist_bend_axis,
            joint7_rotation * wrist_bend_axis)
            .value_or(request.seed[6]));

        JointVector joints = {
          q1,
          normalizeAngle(q2),
          q3,
          normalizeAngle(q4),
          q5,
          normalizeAngle(q6),
          q7,
        };
        if (request.enforce_joint_limits && !insideLimits(joints, model_)) {
          continue;
        }
        const Eigen::Isometry3d actual = forwardInArmBase(joints);
        const double position_error =
          (actual.translation() - request.target_in_arm_base.translation()).norm();
        const double rotation_error = orientationError(
          request.target_in_arm_base.linear(), actual.linear());
        if (position_error > request.position_tolerance ||
            rotation_error > request.orientation_tolerance) {
          continue;
        }
        const bool duplicate = std::any_of(
          solutions.begin(), solutions.end(),
          [&](const V3RedundantIkSolution& kept) {
            return sameJointSolution(kept.joints, joints);
          });
        if (duplicate) {
          continue;
        }
        V3RedundantIkSolution solution;
        solution.joints = joints;
        solution.swivel_angle = swivel;
        solution.shoulder_branch = q2 > 0.0 ? 1 : (q2 < 0.0 ? -1 : 0);
        solution.elbow_branch = q4 > 0.0 ? 1 : (q4 < 0.0 ? -1 : 0);
        solution.wrist_branch = q6 > 0.0 ? 1 : (q6 < 0.0 ? -1 : 0);
        solution.position_error = position_error;
        solution.orientation_error = rotation_error;
        solution.seed_distance = seedDistance(joints, request.seed);
        solution.minimum_joint_limit_margin = minimumLimitMargin(joints, model_);
        solutions.push_back(solution);
      }
    }
  }

  std::sort(
    solutions.begin(), solutions.end(),
    [](const V3RedundantIkSolution& lhs, const V3RedundantIkSolution& rhs) {
      if (lhs.seed_distance != rhs.seed_distance) {
        return lhs.seed_distance < rhs.seed_distance;
      }
      if (lhs.minimum_joint_limit_margin != rhs.minimum_joint_limit_margin) {
        return lhs.minimum_joint_limit_margin > rhs.minimum_joint_limit_margin;
      }
      if (lhs.position_error != rhs.position_error) {
        return lhs.position_error < rhs.position_error;
      }
      return lhs.orientation_error < rhs.orientation_error;
    });
  return solutions;
}

Eigen::Isometry3d V3RedundantArmAnalyticIk::forwardInArmBase(
  const JointVector& joints) const
{
  const Geometry& model = geometry(model_);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (size_t index = 0; index < joints.size(); ++index) {
    transform = transform * model.fixed_transforms[index] *
      rotationTransform(rotZ(joints[index]));
  }
  return transform * toolTransform(model_);
}

Eigen::Vector3d V3RedundantArmAnalyticIk::elbowPositionInArmBase(
  const JointVector& joints) const
{
  return pointAfterJointMotions(geometry(model_).elbow, joints, 2, model_);
}

Eigen::Vector3d V3RedundantArmAnalyticIk::wristCenterInArmBase(
  const JointVector& joints) const
{
  return pointAfterJointMotions(geometry(model_).wrist, joints, 4, model_);
}

double V3RedundantArmAnalyticIk::swivelAngle(
  const JointVector& joints) const
{
  const Geometry& model = geometry(model_);
  const Eigen::Vector3d wrist = wristCenterInArmBase(joints);
  const Eigen::Vector3d shoulder_to_wrist = wrist - model.shoulder;
  const double shoulder_to_wrist_distance = shoulder_to_wrist.norm();
  if (shoulder_to_wrist_distance < kGeometryTolerance) {
    return 0.0;
  }
  const Eigen::Vector3d shoulder_to_wrist_direction =
    shoulder_to_wrist / shoulder_to_wrist_distance;
  const double circle_offset =
    (model.upper_arm_length * model.upper_arm_length -
     model.forearm_length * model.forearm_length +
     shoulder_to_wrist_distance * shoulder_to_wrist_distance) /
    (2.0 * shoulder_to_wrist_distance);
  const double circle_radius_squared =
    model.upper_arm_length * model.upper_arm_length -
    circle_offset * circle_offset;
  if (circle_radius_squared <=
      kSingularityTolerance * kSingularityTolerance) {
    return 0.0;
  }
  const Eigen::Vector3d circle_center =
    model.shoulder + circle_offset * shoulder_to_wrist_direction;
  const Eigen::Vector3d elbow_offset =
    (elbowPositionInArmBase(joints) - circle_center).normalized();
  const auto [swivel_first, swivel_second] =
    swivelBasis(shoulder_to_wrist_direction);
  return std::atan2(
    elbow_offset.dot(swivel_second),
    elbow_offset.dot(swivel_first));
}

Eigen::Vector3d V3RedundantArmAnalyticIk::modelShoulderCenterInArmBase() const
{
  return geometry(model_).shoulder;
}

double V3RedundantArmAnalyticIk::modelArmLength() const
{
  const auto& g = geometry(model_);
  return g.upper_arm_length + g.forearm_length;
}

Eigen::Vector3d V3RedundantArmAnalyticIk::shoulderCenterInArmBase()
{
  return geometry(V3RedundantArmModel::LegacyV304).shoulder;
}

double V3RedundantArmAnalyticIk::upperArmLength()
{
  return geometry(V3RedundantArmModel::LegacyV304).upper_arm_length;
}

double V3RedundantArmAnalyticIk::forearmLength()
{
  return geometry(V3RedundantArmModel::LegacyV304).forearm_length;
}

JointVector V3RedundantArmAnalyticIk::jointLowerLimits() const
{
  return lowerLimits(model_);
}

JointVector V3RedundantArmAnalyticIk::jointUpperLimits() const
{
  return upperLimits(model_);
}

}  // namespace alfa_robot::analytic_ik
