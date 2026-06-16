/*
 * ROS wrapper for CUDA-PointPillars TensorRT inference.
 */

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/check.hpp"
#include "pointpillar.hpp"

namespace {

struct FieldInfo {
  int offset = -1;
  uint8_t datatype = 0;
};

FieldInfo findField(const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      FieldInfo info;
      info.offset = static_cast<int>(field.offset);
      info.datatype = field.datatype;
      return info;
    }
  }
  return {};
}

float readFieldAsFloat(const uint8_t* point, const FieldInfo& field, float fallback = 0.0f) {
  if (field.offset < 0) {
    return fallback;
  }

  const uint8_t* data = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::PointField::FLOAT32: {
      float value = 0.0f;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case sensor_msgs::PointField::FLOAT64: {
      double value = 0.0;
      std::memcpy(&value, data, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::UINT8:
      return static_cast<float>(*data);
    case sensor_msgs::PointField::INT8:
      return static_cast<float>(*reinterpret_cast<const int8_t*>(data));
    case sensor_msgs::PointField::UINT16: {
      uint16_t value = 0;
      std::memcpy(&value, data, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::INT16: {
      int16_t value = 0;
      std::memcpy(&value, data, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::UINT32: {
      uint32_t value = 0;
      std::memcpy(&value, data, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::INT32: {
      int32_t value = 0;
      std::memcpy(&value, data, sizeof(value));
      return static_cast<float>(value);
    }
    default:
      return fallback;
  }
}

geometry_msgs::Quaternion yawToQuaternion(float yaw) {
  geometry_msgs::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = std::sin(yaw * 0.5f);
  quat.w = std::cos(yaw * 0.5f);
  return quat;
}

std_msgs::ColorRGBA colorForClass(int class_id, float alpha) {
  std_msgs::ColorRGBA color;
  color.a = alpha;

  switch (class_id) {
    case 0:
      color.r = 0.1f;
      color.g = 0.85f;
      color.b = 0.25f;
      break;
    case 1:
      color.r = 0.95f;
      color.g = 0.65f;
      color.b = 0.1f;
      break;
    case 2:
      color.r = 0.2f;
      color.g = 0.55f;
      color.b = 1.0f;
      break;
    default:
      color.r = 1.0f;
      color.g = 1.0f;
      color.b = 1.0f;
      break;
  }
  return color;
}

pointpillar::lidar::CoreParameter makeCoreParameter(const std::string& engine_path) {
  pointpillar::lidar::VoxelizationParameter vp;
  vp.min_range = nvtype::Float3(0.0f, -39.68f, -3.0f);
  vp.max_range = nvtype::Float3(69.12f, 39.68f, 1.0f);
  vp.voxel_size = nvtype::Float3(0.16f, 0.16f, 4.0f);
  vp.grid_size = vp.compute_grid_size(vp.max_range, vp.min_range, vp.voxel_size);
  vp.max_voxels = 40000;
  vp.max_points_per_voxel = 32;
  vp.max_points = 300000;
  vp.num_feature = 4;

  pointpillar::lidar::PostProcessParameter pp;
  pp.min_range = vp.min_range;
  pp.max_range = vp.max_range;
  pp.feature_size = nvtype::Int2(vp.grid_size.x / 2, vp.grid_size.y / 2);

  pointpillar::lidar::CoreParameter param;
  param.voxelization = vp;
  param.lidar_model = engine_path;
  param.lidar_post = pp;
  return param;
}

class PointPillarRosNode {
 public:
  explicit PointPillarRosNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param<std::string>("input_topic", input_topic_, "/lidar_sync");
    private_nh_.param<std::string>("boxes_topic", boxes_topic_, "/pointpillar/boxes");
    private_nh_.param<std::string>("engine_path", engine_path_,
                                   "/workspace/CUDA-PointPillars/model/pointpillar.plan");
    private_nh_.param<std::string>("output_frame", output_frame_, "");
    private_nh_.param<bool>("timer", timer_, false);
    private_nh_.param<double>("marker_lifetime", marker_lifetime_sec_, 0.1);
    private_nh_.param<double>("marker_alpha", marker_alpha_, 0.65);

    auto core_param = makeCoreParameter(engine_path_);
    core_ = pointpillar::lidar::create_core(core_param);
    if (core_ == nullptr) {
      throw std::runtime_error("failed to create CUDA-PointPillars core with engine: " + engine_path_);
    }
    core_->set_timer(timer_);
    core_->print();

    checkRuntime(cudaStreamCreate(&stream_));

    boxes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(boxes_topic_, 1);
    cloud_sub_ = nh_.subscribe(input_topic_, 1, &PointPillarRosNode::cloudCallback, this);

    ROS_INFO_STREAM("CUDA-PointPillars ROS node ready. Subscribing " << input_topic_
                    << ", publishing " << boxes_topic_ << ", engine " << engine_path_);
  }

  ~PointPillarRosNode() {
    if (stream_ != nullptr) {
      checkRuntime(cudaStreamDestroy(stream_));
    }
  }

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    std::vector<float> points;
    if (!convertPointCloud(*cloud, points)) {
      return;
    }

    const int num_points = static_cast<int>(points.size() / 4);
    const auto boxes = core_->forward(points.data(), num_points, stream_);
    publishMarkers(*cloud, boxes);

    ROS_DEBUG_STREAM("PointPillars processed " << num_points << " points, detections: " << boxes.size());
  }

  bool convertPointCloud(const sensor_msgs::PointCloud2& cloud, std::vector<float>& points) const {
    const FieldInfo x = findField(cloud, "x");
    const FieldInfo y = findField(cloud, "y");
    const FieldInfo z = findField(cloud, "z");
    FieldInfo intensity = findField(cloud, "intensity");
    if (intensity.offset < 0) {
      intensity = findField(cloud, "i");
    }

    if (x.offset < 0 || y.offset < 0 || z.offset < 0) {
      ROS_WARN_THROTTLE(1.0, "PointCloud2 is missing x/y/z fields.");
      return false;
    }

    const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    if (cloud.point_step == 0 || cloud.data.size() < point_count * cloud.point_step) {
      ROS_WARN_THROTTLE(1.0, "PointCloud2 has invalid point_step or data size.");
      return false;
    }

    points.clear();
    points.reserve(point_count * 4);

    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t* point = cloud.data.data() + i * cloud.point_step;
      const float px = readFieldAsFloat(point, x);
      const float py = readFieldAsFloat(point, y);
      const float pz = readFieldAsFloat(point, z);
      const float pi = readFieldAsFloat(point, intensity, 0.0f);

      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }

      points.push_back(px);
      points.push_back(py);
      points.push_back(pz);
      points.push_back(pi);
    }

    return !points.empty();
  }

  void publishMarkers(const sensor_msgs::PointCloud2& cloud,
                      const std::vector<pointpillar::lidar::BoundingBox>& boxes) const {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear_marker;
    clear_marker.header = cloud.header;
    if (!output_frame_.empty()) {
      clear_marker.header.frame_id = output_frame_;
    }
    clear_marker.ns = "pointpillar_boxes";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    for (size_t i = 0; i < boxes.size(); ++i) {
      const auto& box = boxes[i];

      visualization_msgs::Marker marker;
      marker.header = clear_marker.header;
      marker.ns = "pointpillar_boxes";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = box.x;
      marker.pose.position.y = box.y;
      marker.pose.position.z = box.z;
      marker.pose.orientation = yawToQuaternion(box.rt);
      marker.scale.x = std::max(box.l, 0.01f);
      marker.scale.y = std::max(box.w, 0.01f);
      marker.scale.z = std::max(box.h, 0.01f);
      marker.color = colorForClass(box.id, static_cast<float>(marker_alpha_));
      marker.lifetime = ros::Duration(marker_lifetime_sec_);
      markers.markers.push_back(marker);
    }

    boxes_pub_.publish(markers);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher boxes_pub_;
  std::shared_ptr<pointpillar::lidar::Core> core_;
  cudaStream_t stream_ = nullptr;

  std::string input_topic_;
  std::string boxes_topic_;
  std::string engine_path_;
  std::string output_frame_;
  bool timer_ = false;
  double marker_lifetime_sec_ = 0.1;
  double marker_alpha_ = 0.65;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointpillar_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  try {
    PointPillarRosNode node(nh, private_nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM(e.what());
    return 1;
  }

  return 0;
}
