#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>

class LidarReader : public rclcpp::Node
{
public:
    LidarReader() : Node("lidar_reader")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ this->topic_callback(msg); });

        // Initialize OpenCV window for visualization
        cv::namedWindow("Raw Data", cv::WINDOW_NORMAL);
        cv::namedWindow("Detected Objects", cv::WINDOW_NORMAL);
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const int image_size = 500;
        const float max_range = 12.0;

        // Convert LaserScan data to Cartesian coordinates
        std::vector<cv::Point2f> points;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float angle = msg->angle_min + i * msg->angle_increment;
            float distance = msg->ranges[i];

            // Convert polar coordinates to Cartesian coordinates
            float x = distance * std::cos(angle);
            float y = distance * std::sin(angle);

            // Check if coordinates are within the valid range
            if (std::abs(x) <= max_range && std::abs(y) <= max_range)
            {
                // Normalize coordinates to fit within the range -12 to 12
                x = std::max(std::min(x, max_range), -max_range);
                y = std::max(std::min(y, max_range), -max_range);

                points.push_back(cv::Point2f(x, y));
            }
        }

        // Create a grayscale image from points
        cv::Mat img = cv::Mat::zeros(image_size, image_size, CV_8UC1);

        for (const auto& point : points)
        {
            int x = static_cast<int>((point.x + max_range) / (2 * max_range) * image_size);
            int y = static_cast<int>((max_range - point.y) / (2 * max_range) * image_size);

            // Check if coordinates are within the valid range
            if (x >= 0 && x < image_size && y >= 0 && y < image_size)
            {
                img.at<uchar>(y, x) = 255;  // Set pixel to white
            }
        }

        // Simple Thresholding
        cv::Mat thresholded_img;
        cv::threshold(img, thresholded_img, 200, 255, cv::THRESH_BINARY);

        // Find contours in the thresholded image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresholded_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw rectangles around the clusters with at least 2 points and a maximum of 10
        cv::Mat objects_img = img.clone();

        for (const auto& contour : contours)
        {
            if (contour.size() >= 10 && contour.size() <= 20)
            {
                // Find the bounding rectangle for each contour
                cv::Rect boundingRect = cv::boundingRect(contour);

                // Draw the bounding rectangle in red
                cv::rectangle(objects_img, boundingRect, cv::Scalar(255, 0, 0), 2);
            }
        }

        // Visualize
        cv::imshow("Raw Data", img);
        cv::imshow("Detected Objects", objects_img);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
