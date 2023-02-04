#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include "grid_map_comp/grid_map_comp.hpp"

namespace grid_map {

class Compressor {
  public:
    Compressor(ros::NodeHandle& nh) {
      nh_ = nh;

      std::string layer_config;
      nh_.getParam("layer_config", layer_config);
      parseLayerSpec(layer_config);
      
      map_compressed_pub_ = nh_.advertise<grid_map_msgs::GridMapCompressed>("map/compressed", 2);

      using namespace std;
      stringstream layer_ss;
      for (const auto& l : layer_spec_) {
        layer_ss << "    " << l.name << ": " << l.format << ", " << l.type << endl;
      }

      ROS_INFO_STREAM("\033[32m" << "[MapComp]" << endl << "[ROS] ======== Configuration ========" << endl <<
        "layer_config:" << endl << layer_ss.str() << 
        "[ROS] ====== End Configuration ======" << "\033[0m");
    }

    void initialize() {
      map_sub_ = nh_.subscribe("map", 2, &Compressor::mapCallback, this);
    }

  private:
    ros::NodeHandle nh_;

    ros::Publisher map_compressed_pub_;
    ros::Subscriber map_sub_;

    std::vector<GridMapComp::LayerSpec> layer_spec_;

    void mapCallback(const grid_map_msgs::GridMap::ConstPtr& map_msg) {
      grid_map_msgs::GridMapCompressed compressed_msg;

      using namespace std::chrono;
      auto start_t = high_resolution_clock::now();
      GridMapComp::toCompressedMsg(*map_msg, layer_spec_, compressed_msg);
      auto stop_t = high_resolution_clock::now();

      ROS_INFO_STREAM("\033[34m" << "[MapComp] Compression: " << 
        duration_cast<milliseconds>(stop_t - start_t).count() << "ms");

      map_compressed_pub_.publish(compressed_msg);
    }

    void parseLayerSpec(const std::string& path) {
      try {
        const YAML::Node config = YAML::LoadFile(path);
        for (const auto& layer_config : config) {
          layer_spec_.push_back({
            layer_config["name"].as<std::string>(),
            layer_config["format"].as<std::string>(),
            layer_config["type"].as<std::string>()
          });
        }
      } catch (const YAML::BadFile& ex) {
        throw std::invalid_argument("Could not find layer config");
      } catch (const YAML::Exception& ex) {
        throw std::invalid_argument("Layer config formatted incorrectly");
      }
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
