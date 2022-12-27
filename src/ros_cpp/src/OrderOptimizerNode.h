#include "geometry_msgs/msg/pose_stamped.hpp"
#include "msg_package/msg/order.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <tiff.h>

struct OrderDetails
{
  int order_id;
  float cx;
  float cy;
  std::vector<std::string> products;
};

struct Part
{
  std::string part_name;
  float cx;
  float cy;
  float distance;
  std::string parent_product;
};

struct ProductDetails
{
  std::string product_name;
  std::vector<Part> parts;
};

namespace fs = std::filesystem;
class OrderOptimizerNode : public rclcpp::Node
{
public:
  OrderOptimizerNode();
  ~OrderOptimizerNode() {outputFile.close();}
  std::ofstream outputFile;
  std::string file;

private:
  std::string path_;
  std::map<int, ProductDetails> products;
  bool configuration_already = false;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose;
  std::mutex mutex_;

  void msgCurrentPos(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void msgNextOrder(msg_package::msg::Order::SharedPtr msg);

  void parseOrderFile(
    fs::path file, uint32 order_id, bool & found, OrderDetails * details,
    std::vector<std::string> & pr);

  void parseConfFile(fs::path file);

  float distance(float x1, float y1, float x2, float y2);

  void PublishMarkerArray(std::vector<std::pair<float, Part>> vector);

  visualization_msgs::msg::Marker AMRMarker();

  visualization_msgs::msg::Marker Pickup(std::pair<float, Part> pair);

  std::vector<std::pair<float, Part>> FindShortestPath(OrderDetails details);

  void PathOutput(
    msg_package::msg::Order::SharedPtr msg, std::vector<std::pair<float, Part>> vector,
    OrderDetails details);
};
