#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <chrono>
#include "grid_map_comp/grid_map_comp.hpp"

namespace grid_map {

class Compressor {
  public:
    Compressor(ros::NodeHandle& nh) {
      nh_ = nh;

      parseLayerSpec();
      map_compressed_pub_ = nh_.advertise<grid_map_msgs::GridMapCompressed>("map/compressed", 1);
    }

    void initialize() {
      map_sub_ = nh_.subscribe("map", 1, &Compressor::mapCallback, this);
    }

  private:
    ros::NodeHandle nh_;

    ros::Publisher map_compressed_pub_;
    ros::Subscriber map_sub_;

    std::vector<GridMapComp::LayerSpec> layer_spec_;

    void mapCallback(const grid_map_msgs::GridMap::ConstPtr& map_msg) {
      grid_map_msgs::GridMapCompressed compressed_msg;

      auto start_t = std::chrono::high_resolution_clock::now();
      GridMapComp::toCompressedMsg(*map_msg, layer_spec_, compressed_msg);
      auto stop_t = std::chrono::high_resolution_clock::now();

      map_compressed_pub_.publish(compressed_msg);

      ROS_INFO_STREAM("\033[34m" << "[MapComp] Compression: " << 
        std::chrono::duration_cast<std::chrono::milliseconds>(stop_t - start_t).count() << "ms");
    }

    void parseLayerSpec() {
      // Hardcode for now
      layer_spec_.push_back({
        "color", ".png", true, false
      });
      layer_spec_.push_back({
        "elevation", ".png", false, false 
      });
      layer_spec_.push_back({
        "semantics", ".png", false, true
      });
      layer_spec_.push_back({
        "semantics_viz", ".png", true, false
      });
    }
};

} // namespace grid_map

int main(int argc, char **argv) {
  ros::init(argc, argv, "compressor");
  ros::NodeHandle nh("~");

  try {
    grid_map::Compressor node(nh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
  return 0;
}
