#pragma once

#include <grid_map_msgs/GridMapCompressed.h>
#include <grid_map_msgs/GridMap.h>

#include <opencv2/core/core.hpp>

namespace grid_map {

class GridMapComp {
  public:
    struct LayerSpec {
      std::string name;
      std::string format;
      std::string type;
    };
    static void toCompressedMsg(const grid_map_msgs::GridMap& msg,
        const std::vector<LayerSpec>& layers,
        grid_map_msgs::GridMapCompressed& comp_msg);

    static void fromCompressedMsg(
        const grid_map_msgs::GridMapCompressed& comp_msg,
        grid_map_msgs::GridMap& msg);

    static void toImage(const grid_map_msgs::GridMap& msg,
        const LayerSpec& layers, cv::Mat& image);

    struct ImageDisc {
      float scale;
      float offset;
    };
    static ImageDisc discImage(cv::Mat& float_img, cv::Mat& disc_img);

  private:
    static inline uint32_t packColor(uint8_t b, uint8_t g, uint8_t r) {
      return (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 |
              static_cast<uint32_t>(b));
    }

    static inline std::array<uint8_t, 3> unpackColor(uint32_t color) {
      return {static_cast<uint8_t>(color & 0x0000ff),
              static_cast<uint8_t>((color >> 8) & 0x0000ff),
              static_cast<uint8_t>((color >> 16) & 0x0000ff)};
    }
};

} // namespace grid_map
