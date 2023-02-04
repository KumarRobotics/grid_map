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

void GridMapComp::toImage(const grid_map_msgs::GridMap& msg,
    const LayerSpec& layer, cv::Mat& image)
{
  int layer_id = 0;
  for (const auto& layer_name : msg.layers) {
    if (layer_name == layer.name) break;
    ++layer_id;
  }

  // Layer doesn't exist
  if (layer_id >= msg.data.size()) return;

  // Yes, casting away const-ness is a dangerous thing to do
  // We pinky-promise not to modify this matrix
  cv::Mat raw_mat(msg.data[layer_id].layout.dim[0].size,
                  msg.data[layer_id].layout.dim[1].size, CV_32F, 
                  const_cast<float*>(&msg.data[layer_id].data.front()));

  // Note that we transpose in all of these b/c data is stored col-major
  // but opencv is row-major
  if (layer.is_rgb) {
    image = cv::Mat(raw_mat.cols, raw_mat.rows, CV_8UC3, cv::Scalar(255, 255, 255));

    for(int i=0; i<image.rows; i++) {
      for(int j=0; j<image.cols; j++) {
        float val = raw_mat.at<float>(j, i);
        if (std::isfinite(val)) {
          auto color = unpackColor(*reinterpret_cast<uint32_t*>(&val));
          image.at<cv::Vec3b>(i, j) = cv::Vec3b(color[0], color[1], color[2]);
        }
      }
    }
  } else if (layer.is_char) {
    image = cv::Mat(raw_mat.cols, raw_mat.rows, CV_8UC1, cv::Scalar(255));

    for(int i=0; i<image.rows; i++) {
      for(int j=0; j<image.cols; j++) {
        float val = raw_mat.at<float>(j, i);
        if (std::isfinite(val)) {
          image.at<uint8_t>(i, j) = val;
        }
      }
    }
  } else {
    cv::transpose(raw_mat, image);
  }
}

} // namespace grid_map
