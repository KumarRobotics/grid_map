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
    if (std::find(msg.layers.begin(), msg.layers.end(), layer_info.name) == 
        msg.layers.end())
    {
      // Layer is not in the message, stop
      continue;
    }

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
    layer.is_rgb = layer_info.type == "rgb";

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
  // Metadata is all the same
  msg.info = comp_msg.info;
  msg.outer_start_index = comp_msg.outer_start_index;
  msg.inner_start_index = comp_msg.inner_start_index;
  msg.layers = comp_msg.layers;
  msg.basic_layers = comp_msg.basic_layers;

  for (const auto& comp_layer : comp_msg.data) {
    cv::Mat raw_decomp;
    cv::imdecode(comp_layer.data, cv::IMREAD_UNCHANGED, &raw_decomp);

    msg.data.push_back({});
    auto& layer = msg.data.back();
    layer.data.resize(raw_decomp.total());
    cv::Mat dest_mat(raw_decomp.cols, raw_decomp.rows, CV_32F, 
                    &layer.data.front());

    // Metadata
    {
      std_msgs::MultiArrayDimension dim;
      dim.label = "column_index";
      dim.size = raw_decomp.cols;
      dim.stride = raw_decomp.cols * raw_decomp.rows;
      layer.layout.dim.push_back(dim);
    }
    {
      std_msgs::MultiArrayDimension dim;
      dim.label = "row_index";
      dim.size = raw_decomp.rows;
      dim.stride = raw_decomp.rows;
      layer.layout.dim.push_back(dim);
    }
    layer.layout.data_offset = 0;

    // Actually copy over data
    if (comp_layer.is_rgb) {
      dest_mat.setTo(std::numeric_limits<float>::quiet_NaN());
      for(int i=0; i<dest_mat.rows; i++) {
        for(int j=0; j<dest_mat.cols; j++) {
          auto& val = raw_decomp.at<cv::Vec3b>(j, i);
          if (val[0] != 255 || val[1] != 255 || val[2] != 255) {
            auto color = packColor(val[0], val[1], val[2]);
            dest_mat.at<float>(i, j) = *reinterpret_cast<float*>(&color);
          }
        }
      }
    } else {
      cv::transpose(raw_decomp, raw_decomp);
      raw_decomp.convertTo(dest_mat, CV_32F, comp_layer.scale, comp_layer.offset);
      if (raw_decomp.type() == CV_16U) {
        dest_mat.setTo(std::numeric_limits<float>::quiet_NaN(), 
            raw_decomp == std::numeric_limits<uint16_t>::max());
      } else {
        dest_mat.setTo(std::numeric_limits<float>::quiet_NaN(), 
            raw_decomp == std::numeric_limits<uint8_t>::max());
      }
    }
  }
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
  if (layer.type == "rgb") {
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
  } else if (layer.type == "char") {
    raw_mat.convertTo(image, CV_8UC1);
    cv::Mat raw_copy = raw_mat.clone();
    cv::patchNaNs(raw_copy, std::numeric_limits<float>::max());
    image.setTo(std::numeric_limits<uint8_t>::max(), 
        raw_copy == std::numeric_limits<float>::max());
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
  cv::patchNaNs(float_img, std::numeric_limits<float>::max());
  cv::minMaxLoc(float_img, &min, &max, 0, 0, 
      float_img != std::numeric_limits<float>::max());
  ImageDisc scale_offset;

  // Use max-2 to make sure that max is reserved for nan
  scale_offset.scale = (max - min)/(std::numeric_limits<uint16_t>::max()-2);
  scale_offset.offset = min;

  float_img.convertTo(disc_img, CV_16UC1, 1./scale_offset.scale, 
      -scale_offset.offset/scale_offset.scale);
  // Set nans to max value
  disc_img.setTo(std::numeric_limits<uint16_t>::max(), 
      float_img == std::numeric_limits<float>::max());

  return scale_offset;
}

} // namespace grid_map
