#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include "grid_map_comp/grid_map_comp.hpp"

namespace grid_map {

class Compressor {
  public:
    Compressor(ros::NodeHandle& nh) {
      nh_ = nh;

      map_compressed_pub_ = nh_.advertise<grid_map_msgs::GridMapCompressed>("map/compressed", 1);
    }

    void initialize() {
      map_sub_ = nh_.subscribe("map", 1, &Compressor::mapCallback, this);
    }

  private:
    ros::NodeHandle nh_;

    ros::Publisher map_compressed_pub_;
    ros::Subscriber map_sub_;

    void mapCallback(const grid_map_msgs::GridMap::ConstPtr& map_msg) {
      cv::Mat img;
      GridMapComp::toImage(*map_msg, {"color", "png", true, false}, img);
      cv::imshow("color", img);
      cv::waitKey(0);
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
