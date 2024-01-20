#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber(const std::string &topic)
      : Node("image_listener"), topic_(topic) {
    RCLCPP_INFO(this->get_logger(),
                "Initializing ImageSubscriber with topic: %s", topic.c_str());
    cv::namedWindow("view");
    cv::startWindowThread();
  }

  ~ImageSubscriber() { cv::destroyWindow("view"); }

  void init() {
    RCLCPP_INFO(this->get_logger(), "Initializing Image Transport");
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    // Subscribe to the base topic and let image_transport handle the transport
    subscriber_ =
        it_->subscribe(topic_, 1, &ImageSubscriber::imageCallback, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to base topic: %s",
                topic_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(10);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.",
                   msg->encoding.c_str());
    }
  }

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber subscriber_;
  std::string topic_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("ImageSubscriberMain"), "Node started");

  // Separate ROS arguments from custom arguments
  auto args = rclcpp::NodeOptions().arguments();
  std::string topic = "camera/image";
  for (size_t i = 1; i < args.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("ImageSubscriberMain"), "Argument [%ld]: %s",
                i, args[i].c_str());
    if (args[i].find("--ros-args") == std::string::npos) {
      topic = args[i];
      break;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ImageSubscriberMain"), "Using topic: %s",
              topic.c_str());

  auto node = std::make_shared<ImageSubscriber>(topic);
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
