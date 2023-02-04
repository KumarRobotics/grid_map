#include "grid_map_comp/grid_map_comp.hpp"
#include <opencv2/core/core.hpp>

namespace grid_map {

void GridMapComp::toCompressedMsg(const grid_map_msgs::GridMap& msg,
    const std::vector<LayerSpec>& layers,
    grid_map_msgs::GridMapCompressed& comp_msg)
{
}

void GridMapComp::fromCompressedMsg(
    const grid_map_msgs::GridMapCompressed& comp_msg,
    grid_map_msgs::GridMap& msg)
{
}

void GridMapComp::toImages(const grid_map_msgs::GridMap& msg,
    const std::vector<LayerSpec>& layers,
    std::vector<cv::Mat>& images)
{
}

} // namespace grid_map
