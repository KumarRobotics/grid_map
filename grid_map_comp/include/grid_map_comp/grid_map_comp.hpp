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
      bool is_rgb;
      bool is_char;
    };
    static void toCompressedMsg(const grid_map_msgs::GridMap& msg,
        const std::vector<LayerSpec>& layers,
        grid_map_msgs::GridMapCompressed& comp_msg);

    static void fromCompressedMsg(
        const grid_map_msgs::GridMapCompressed& comp_msg,
        grid_map_msgs::GridMap& msg);

    static void toImages(const grid_map_msgs::GridMap& msg,
        const std::vector<LayerSpec>& layers,
        std::vector<cv::Mat>& images);
};

} // namespace grid_map
