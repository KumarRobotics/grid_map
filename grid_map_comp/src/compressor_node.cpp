#include <ros/ros.h>
#include "grid_map_comp/grid_map_comp.hpp"

class Compressor {
  public:
    Compressor(ros::NodeHandle& nh) {
      nh_ = nh;
    }

    void initialize() {
    }

  private:
    ros::NodeHandle nh_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "compressor");
  ros::NodeHandle nh("~");

  try {
    Compressor node(nh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
  return 0;
}
