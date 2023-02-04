#include <ros/ros.h>
#include <chrono>
#include "grid_map_comp/grid_map_comp.hpp"

namespace grid_map {

class Decompressor {
  public:
    Decompressor(ros::NodeHandle& nh) {
      nh_ = nh;

      map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map", 2);
    }

    void initialize() {
      map_compressed_sub_ = nh_.subscribe("map/compressed", 2, 
          &Decompressor::compressedCallback, this);
    }

  private:
    ros::NodeHandle nh_;

    ros::Publisher map_pub_;
    ros::Subscriber map_compressed_sub_;

    void compressedCallback(
        const grid_map_msgs::GridMapCompressed::ConstPtr& comp_msg) 
    {
      grid_map_msgs::GridMap map_msg;

      using namespace std::chrono;
      auto start_t = high_resolution_clock::now();
      GridMapComp::fromCompressedMsg(*comp_msg, map_msg);
      auto stop_t = high_resolution_clock::now();

      ROS_INFO_STREAM("\033[34m" << "[MapDecomp] Decompression: " << 
        duration_cast<milliseconds>(stop_t - start_t).count() << "ms");

      map_pub_.publish(map_msg);
    }
};

} // namespace grid_map

int main(int argc, char **argv) {
  ros::init(argc, argv, "decompressor");
  ros::NodeHandle nh("~");

  try {
    grid_map::Decompressor node(nh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
  return 0;
}
