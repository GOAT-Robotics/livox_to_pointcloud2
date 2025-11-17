#pragma once

#ifdef ROS1
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
using PointField = sensor_msgs::PointField;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud2Ptr = sensor_msgs::PointCloud2::Ptr;
using PointCloud2ConstPtr = sensor_msgs::PointCloud2::ConstPtr;
#endif

#ifdef ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
using PointField = sensor_msgs::msg::PointField;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
#endif
#include <cstring>

namespace livox_to_pointcloud2 {

class LivoxConverter {
public:
  LivoxConverter() {
    points_msg.reset(new PointCloud2);

    const auto add_field = [this](const std::string& name, const int offset, const uint8_t datatype) {
      PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = 1;
      points_msg->fields.push_back(field);
    };

    add_field("x", 0, PointField::FLOAT32);
    add_field("y", points_msg->fields.back().offset + sizeof(float), PointField::FLOAT32);
    add_field("z", points_msg->fields.back().offset + sizeof(float), PointField::FLOAT32);
    add_field("t", points_msg->fields.back().offset + sizeof(float), PointField::UINT32);
    add_field("intensity", points_msg->fields.back().offset + sizeof(std::uint32_t), PointField::FLOAT32);
    add_field("tag", points_msg->fields.back().offset + sizeof(float), PointField::UINT8);
    add_field("line", points_msg->fields.back().offset + sizeof(std::uint8_t), PointField::UINT8);
    points_msg->is_bigendian = false;
    points_msg->point_step = sizeof(float) * 4 + sizeof(uint32_t) + sizeof(uint8_t) * 2;
    points_msg->is_dense = true;
  }

  template <typename CustomMsg>
  PointCloud2ConstPtr convert(const CustomMsg& livox_msg) {
    auto msg = std::make_shared<PointCloud2>();
    msg->header = livox_msg.header;
    msg->fields = points_msg->fields;
    msg->is_bigendian = false;
    msg->is_dense = true;
    msg->point_step = sizeof(float) * 4 + sizeof(uint32_t) + sizeof(uint8_t) * 2;
    msg->width  = livox_msg.point_num;
    msg->height = 1;
    msg->row_step = msg->width * msg->point_step;
    msg->data.resize(msg->row_step);

    uint8_t* ptr = msg->data.data();                      // single declaration
    const size_t N = static_cast<size_t>(livox_msg.point_num);

    for (size_t i = 0; i < N; ++i) {
      const auto& p = livox_msg.points[i];

      // x, y, z
      std::memcpy(ptr + msg->fields[0].offset, &p.x, sizeof(float));
      std::memcpy(ptr + msg->fields[1].offset, &p.y, sizeof(float));
      std::memcpy(ptr + msg->fields[2].offset, &p.z, sizeof(float));

      // t (offset_time) as uint32
      std::memcpy(ptr + msg->fields[3].offset, &p.offset_time, sizeof(uint32_t));

      // intensity (reflectivity may be integer in some Livox flavors) -> float
      float intensity = static_cast<float>(p.reflectivity);
      std::memcpy(ptr + msg->fields[4].offset, &intensity, sizeof(float));

      // tag, line
      std::memcpy(ptr + msg->fields[5].offset, &p.tag,  sizeof(uint8_t));
      std::memcpy(ptr + msg->fields[6].offset, &p.line, sizeof(uint8_t));

      ptr += msg->point_step;
    }

    // Optional: ensure a frame_id
    if (msg->header.frame_id.empty()) {
      msg->header.frame_id = "livox_frame";
    }

    return msg;
  }

private:
  PointCloud2Ptr points_msg;
};
}  // namespace livox_to_pointcloud2