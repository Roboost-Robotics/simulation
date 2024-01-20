#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher(int video_source)
      : Node("image_publisher"), video_source_(video_source) {
    cap_.open(video_source_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video source %d",
                   video_source_);
      rclcpp::shutdown();
    }
  }

  void init() {
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    publisher_ = it_->advertise("camera/image", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // Adjust the rate as needed
        std::bind(&ImagePublisher::timerCallback, this));
  }

private:
  void timerCallback() {
    cv::Mat frame;
    if (cap_.read(frame)) {
      // Check if grabbed frame is actually full with some content
      if (!frame.empty()) {
        // Rotate frame 180 degrees
        cv::flip(frame, frame, -1);

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                       .toImageMsg();
        publisher_.publish(msg);
        cv::waitKey(1);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to capture frame from video source");
    }
  }

  cv::VideoCapture cap_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher publisher_;
  int video_source_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Usage: image_publisher <video_source>");
    return 1;
  }

  int video_source;
  std::istringstream video_sourceCmd(argv[1]);
  if (!(video_sourceCmd >> video_source)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid video source: %s",
                 argv[1]);
    return 1;
  }

  auto node = std::make_shared<ImagePublisher>(video_source);
  node->init(); // Initialize the Image Transport after construction
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
