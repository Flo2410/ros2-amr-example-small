
#include "OrderOptimizerNode.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OrderOptimizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

