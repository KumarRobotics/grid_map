#include "grid_map_comp/grid_map_comp.hpp"
#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace grid_map {

void GridMapComp::toCompressedMsg(const grid_map_msgs::GridMap& msg,
    const std::vector<LayerSpec>& layers,
    grid_map_msgs::GridMapCompressed& comp_msg)
{
  comp_msg.info = msg.info;
  comp_msg.outer_start_index = msg.outer_start_index;
  comp_msg.inner_start_index = msg.inner_start_index;

  for (const auto& layer_info : layers) {
    comp_msg.layers.push_back(layer_info.name);
    if (std::find(msg.basic_layers.begin(), msg.basic_layers.end(), layer_info.name) != 
        msg.basic_layers.end()) 
    {
      // layer is a basic layer
      comp_msg.basic_layers.push_back(layer_info.name);
    }
    // Push, then access data.  This is more efficient than assembling and then
    // pushing, which would create another copy
    comp_msg.data.push_back({});
    auto& layer = comp_msg.data.back();

    // Metadata
    layer.format = layer_info.format;
    layer.is_rgb = layer_info.is_rgb;

    cv::Mat layer_img, disc_img;
    toImage(msg, layer_info, layer_img);

    if (layer_img.depth() == CV_32F) {
      // We cannot directly encode a floating point image
      // Instead, we have to rescale and convert to uint16
      auto scale_offset = discImage(layer_img, disc_img);
      layer.scale = scale_offset.scale;
      layer.offset = scale_offset.offset;
    } else {
      // We can compress directly
      layer.scale = 1;
      layer.offset = 0;
      // This is super cheap, not a deep copy
      disc_img = layer_img;
    }

    cv::imencode(layer.format, disc_img, layer.data);
  }
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
    raw_mat.convertTo(image, CV_8UC1);
    image.setTo(std::numeric_limits<uint8_t>::max(), raw_mat != raw_mat);
    cv::transpose(image, image);
  } else {
    cv::transpose(raw_mat, image);
  }
}

GridMapComp::ImageDisc GridMapComp::discImage(
    cv::Mat& float_img, cv::Mat& disc_img) 
{
  // Get min and max of non-nan points
  double max, min;
  cv::minMaxLoc(float_img, &min, &max, 0, 0, float_img == float_img);
  // Set nans to max value
  ImageDisc scale_offset;

  // Use max-2 to make sure that max is reserved for nan
  scale_offset.scale = (max - min)/(std::numeric_limits<uint16_t>::max()-2);
  scale_offset.offset = min;
  float_img.convertTo(disc_img, CV_16UC1, 1./scale_offset.scale, 
      -scale_offset.offset/scale_offset.scale);
  disc_img.setTo(std::numeric_limits<uint16_t>::max(), float_img != float_img);

  return scale_offset;
}

} // namespace grid_map
