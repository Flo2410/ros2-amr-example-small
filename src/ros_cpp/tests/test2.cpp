#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"
#include "../src/OrderOptimizerNode.h"
#include <thread>
#include <tiff.h>
#include <chrono>
#include <future>
#include <memory>



class TestSubscription : public ::testing::Test
{
  public:
    TestSubscription(){};
   ~TestSubscription(){};
};

class CallBack
{
  public:
    CallBack(){
      sub_called_future = sub_called.get_future();
    };

    int counter = 0;
    std::promise<void> sub_called;
    std::shared_future<void> sub_called_future;
    static std::string file_name;
    void callback(visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
      ++counter;
      std::cerr << "came here " << msg->markers.size() <<  std::endl;
      sub_called.set_value();
    }

    std::string GetOutPutFromFile()
    {
      std::string myText;

      std::ifstream MyReadFile(CallBack::file_name + ".txt");
      MyReadFile >> myText;
      MyReadFile.close();

      return  myText;
    }
};

std::string CallBack::file_name = "file";

TEST_F(TestSubscription, testing)
{
  CallBack t;

  //publisher for nextOrder
  auto order_node = rclcpp::Node::make_shared("NextOrderPublisher");
  auto order_node_pub = order_node->create_publisher<msg_package::msg::Order>("nextOrder", 10);

  //publisher for currentPosition
  auto position_node = rclcpp::Node::make_shared("CurrentPosPublisher");
  auto position_node_pub = order_node->create_publisher<geometry_msgs::msg::PoseStamped>("currentPosition", 10);

  // My OrderOptimizer node
  auto order_opt_sub_node = std::make_shared<OrderOptimizerNode>();
  rclcpp::Parameter str_param("path", fs::current_path().parent_path().parent_path().parent_path());
  order_opt_sub_node->set_parameter(str_param);


  // make sure of subscription count
  EXPECT_EQ(order_node_pub->get_subscription_count(), 1U);
  EXPECT_EQ(position_node_pub->get_subscription_count(), 1U);

  auto order_path_node = rclcpp::Node::make_shared("OrderPathSubscriber");
  auto order_path_node_sub = order_path_node->create_subscription<visualization_msgs::msg::MarkerArray>("order_path", 10,  std::bind(
          &CallBack::callback, &t, std::placeholders::_1));


  EXPECT_EQ(order_path_node_sub->get_publisher_count(), 1U);

  // prepare and publish current position msg
  auto current_pos_msg = geometry_msgs::msg::PoseStamped();
  current_pos_msg.pose.position.x = 100;
  current_pos_msg.pose.position.y = 350;
  position_node_pub->publish(current_pos_msg);

  // prepare and publish order msg
  auto order_message = msg_package::msg::Order();
  order_message.order_id = 1000001;
  order_message.description = "Order Info";
  order_node_pub->publish(order_message);


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(order_opt_sub_node);
  executor.spin_some(std::chrono::seconds(50));
}

TEST_F(TestSubscription, multiPublishing)
{
  CallBack t;

  //publisher for nextOrder
  auto order_node = rclcpp::Node::make_shared("NextOrderPublisher");
  auto order_node_pub = order_node->create_publisher<msg_package::msg::Order>("nextOrder", 10);

  //publisher for currentPosition
  auto position_node = rclcpp::Node::make_shared("CurrentPosPublisher");
  auto position_node_pub = order_node->create_publisher<geometry_msgs::msg::PoseStamped>("currentPosition", 10);

  // My OrderOptimizer node
  auto order_opt_sub_node = std::make_shared<OrderOptimizerNode>();
  rclcpp::Parameter str_param("path", fs::current_path().parent_path().parent_path().parent_path());
  order_opt_sub_node->set_parameter(str_param);


  // make sure of subscription count
  EXPECT_EQ(order_node_pub->get_subscription_count(), 1U);
  EXPECT_EQ(position_node_pub->get_subscription_count(), 1U);

  auto order_path_node = rclcpp::Node::make_shared("OrderPathSubscriber");
  auto order_path_node_sub = order_path_node->create_subscription<visualization_msgs::msg::MarkerArray>("order_path", 10,  std::bind(
          &CallBack::callback, &t, std::placeholders::_1));


  EXPECT_EQ(order_path_node_sub->get_publisher_count(), 1U);

  // prepare and publish current position msg
  auto current_pos_msg = geometry_msgs::msg::PoseStamped();
  current_pos_msg.pose.position.x = 100;
  current_pos_msg.pose.position.y = 350;
  position_node_pub->publish(current_pos_msg);

  // prepare and publish order msg
  auto order_message = msg_package::msg::Order();
  order_message.order_id = 1000001;
  order_message.description = "Order Info 1";
  order_node_pub->publish(order_message);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(order_opt_sub_node);
  executor.spin_some(std::chrono::seconds(50));


  order_message = msg_package::msg::Order();
  order_message.order_id = 1100001;
  order_message.description = "Order Info 2";
  order_node_pub->publish(order_message);
  executor.spin_some(std::chrono::seconds(50));


  order_message = msg_package::msg::Order();
  order_message.order_id = 11001;
  order_message.description = "Wrong Order Info 3";
  order_node_pub->publish(order_message);
  executor.spin_some(std::chrono::seconds(50));

  order_message = msg_package::msg::Order();
  order_message.order_id = 1200016;
  order_message.description = "Wrong Order Info 4";
  order_node_pub->publish(order_message);
  executor.spin_some(std::chrono::seconds(50));
}


TEST_F(TestSubscription, OnlyOnePublish)
{
  CallBack t;

  //publisher for nextOrder
  auto order_node = rclcpp::Node::make_shared("NextOrderPublisher");
  auto order_node_pub = order_node->create_publisher<msg_package::msg::Order>("nextOrder", 10);

  // My OrderOptimizer node
  auto order_opt_sub_node = std::make_shared<OrderOptimizerNode>();
  rclcpp::Parameter str_param("path", fs::current_path().parent_path().parent_path());
  order_opt_sub_node->set_parameter(str_param);
  CallBack::file_name = order_opt_sub_node->file;

  // make sure of subscription count
  EXPECT_EQ(order_node_pub->get_subscription_count(), 1U);


  auto order_message = msg_package::msg::Order();
  order_message.order_id = 1000001;
  order_message.description = "first order";
  order_node_pub->publish(order_message);


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(order_opt_sub_node);
  executor.spin_some(std::chrono::seconds(50));

  std::string myText = t.GetOutPutFromFile();
  ASSERT_FALSE(myText.find("Error:") != std::string::npos);
}


TEST_F(TestSubscription, wrongOrderID)
{
  CallBack t;

  //publisher for nextOrder
  auto order_node = rclcpp::Node::make_shared("NextOrderPublisher");
  auto order_node_pub = order_node->create_publisher<msg_package::msg::Order>("nextOrder", 10);

  //publisher for currentPosition
  auto position_node = rclcpp::Node::make_shared("CurrentPosPublisher");
  auto position_node_pub = order_node->create_publisher<geometry_msgs::msg::PoseStamped>("currentPosition", 10);

  // My OrderOptimizer node
  auto order_opt_sub_node = std::make_shared<OrderOptimizerNode>();
  rclcpp::Parameter str_param("path", fs::current_path().parent_path().parent_path());
  order_opt_sub_node->set_parameter(str_param);
  CallBack::file_name = order_opt_sub_node->file;


  EXPECT_EQ(order_node_pub->get_subscription_count(), 1U);
  EXPECT_EQ(position_node_pub->get_subscription_count(), 1U);

  auto order_path_node = rclcpp::Node::make_shared("OrderPathSubscriber");
  auto order_path_node_sub = order_path_node->create_subscription<visualization_msgs::msg::MarkerArray>("order_path", 10, std::bind(
          &CallBack::callback, &t, std::placeholders::_1));


  EXPECT_EQ(order_path_node_sub->get_publisher_count(), 1U);

  auto current_pos_msg = geometry_msgs::msg::PoseStamped();
  current_pos_msg.pose.position.x = 100;
  current_pos_msg.pose.position.y = 350;
  position_node_pub->publish(current_pos_msg);

  auto order_message = msg_package::msg::Order();
  order_message.order_id = 55555;
  order_message.description = "Wrong order";
  order_node_pub->publish(order_message);


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(order_opt_sub_node);
  executor.spin_some(std::chrono::seconds(50));



  std::string myText = t.GetOutPutFromFile();
  ASSERT_FALSE(myText.find("Error:") != std::string::npos);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return all_successful;
}
