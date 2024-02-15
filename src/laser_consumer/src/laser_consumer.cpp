#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>

class LidarReader : public rclcpp::Node
{
public:
    LidarReader() : Node("lidar_reader"), frame_counter_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ this->topic_callback(msg); });

        // Initialize previous clusters for the first frame
        previous_clusters.clear();

        // Initialize OpenCV window for visualization
        cv::namedWindow("Detected Objects", cv::WINDOW_NORMAL);

        // Initialize variables for scan rate calculation
        last_scan_time_ = std::chrono::high_resolution_clock::now();
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const int image_size = 500;
        const float max_range = 12.0;
        const float alpha = 0.0007; // Adjust this value according to your needs
        const int min_pts = 10;     // Adjust this value according to your needs
        

        // Calculate frame time difference
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = current_time - last_scan_time_;
        frame_time_difference_ = elapsed_seconds.count();
        last_scan_time_ = current_time;

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


        // Segmentation
        std::vector<std::vector<cv::Point2f>> clusters;
        std::vector<bool> checked(points.size(), false);

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (!checked[i])
            {
                std::vector<cv::Point2f> cluster;
                std::queue<size_t> q;

                q.push(i);

                while (!q.empty())
                {
                    size_t idx = q.front();
                    q.pop();

                    if (!checked[idx])
                    {
                        float r = alpha * points.size();
                        std::vector<size_t> neighbors = findNeighbors(points, idx, r);

                        if (neighbors.size() >= min_pts)
                        {
                            for (size_t neighbor : neighbors)
                            {
                                cluster.push_back(points[neighbor]);
                                q.push(neighbor);
                            }
                        }

                        checked[idx] = true;
                    }
                }

                if (cluster.size() >= min_pts)
                {
                    clusters.push_back(cluster);
                }
            }
        }

        // Draw bounding boxes around clusters of points 
        cv::Mat objects_img = img.clone();
        
        //convert from 1 color channel to 3 color channels
        cv::cvtColor(objects_img, objects_img, cv::COLOR_GRAY2BGRA);

        for (auto& cluster : clusters) {
            std::vector<cv::Point> cluster_img;
            for (const auto& point : cluster) {
                int x = static_cast<int>((point.x + max_range) / (2 * max_range) * image_size);
                int y = static_cast<int>((max_range - point.y) / (2 * max_range) * image_size);

                if (x >= 0 && x < image_size && y >= 0 && y < image_size) {
                    cluster_img.push_back(cv::Point(x, y));
                }
            }

            // Draw bounding box
            if (!cluster_img.empty()) {
                cv::Rect bounding_box = cv::boundingRect(cluster_img);
                cv::rectangle(objects_img, bounding_box, cv::Scalar(255, 255, 255), 2);
            }
        }

        // Calculate speed based on previous and current clusters
        if (!previous_clusters.empty()) {
            // Assuming clusters_ is the current clusters found
            for (const auto& current_cluster : clusters) {
                // Find the nearest cluster in the previous frame
                float min_distance = std::numeric_limits<float>::max();
                for (const auto& prev_cluster : previous_clusters) {
                    float distance = cv::norm(calculateCentroid(current_cluster) - calculateCentroid(prev_cluster));
                    if (distance < min_distance) {
                        min_distance = distance;
                    }
                }

                // Calculate speed using the distance and time difference between frames
                float speed = min_distance / frame_time_difference_; // Assuming you have frame_time_difference_

                // Convert the speed from m/s to km/h
                speed = speed * 3.6;

                
                // Display the average speed
                cv::Point2f text_position = calculateCentroid(current_cluster);
                // Adjust the text position since the coordinates of the image are different from the coordinates of the points
                text_position.x = (text_position.x + max_range) / (2 * max_range) * image_size;
                text_position.y = (max_range - text_position.y) / (2 * max_range) * image_size;
                //slightly adjust the position of the text 10 pixels to the right and 10 pixels down
                
                text_position.y -=25;

                //truncate the average speed to integer
                speed = static_cast<int>(speed);
                int final_speed = speed;


                //put the text on the image as integer
                cv::putText(objects_img, std::to_string(final_speed), text_position, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                
                //if the speed is greater than 10km/h put the image car_icon.png on the centroid of the cluster
                if (speed > 15) {
                    cv::Mat car_icon = cv::imread("car_icon.png", cv::IMREAD_UNCHANGED);
                    cv::Point2f car_icon_position = calculateCentroid(current_cluster);
                    car_icon_position.x = (car_icon_position.x + max_range) / (2 * max_range) * image_size;
                    car_icon_position.y = (max_range - car_icon_position.y) / (2 * max_range) * image_size;
                    car_icon_position.x -= car_icon.cols / 2;
                    car_icon_position.y -= car_icon.rows / 2;
                    car_icon.copyTo(objects_img(cv::Rect(car_icon_position.x, car_icon_position.y, car_icon.cols, car_icon.rows)));
                }
            }
        }

        clusters_ = clusters;

        // Display images
        cv::imshow("Detected Objects", objects_img);

        // Increment frame counter
        frame_counter_++;
    }

    // Find neighbors for a point used in the segmentation
    std::vector<size_t> findNeighbors(const std::vector<cv::Point2f>& points, size_t idx, float radius)
    {
        std::vector<size_t> neighbors;
        const cv::Point2f& current_point = points[idx];

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (i != idx)
            {
                const cv::Point2f& other_point = points[i];
                float distance = cv::norm(current_point - other_point);
                
                //calculate the distance from the center of the point to the origin
                float distance_from_origin = cv::norm(current_point);

                //adjust radius based on distance from origin to account for laser dispersion
                float adjustedRadius = radius * (1+ (sin(0.9)*distance_from_origin));

                if (distance <= adjustedRadius)
                {
                    neighbors.push_back(i);
                }
            }
        }

        return neighbors;
    }

    // Calculate the centroid of a cluster
    cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& cluster)
    {
        cv::Point2f centroid(0, 0);

        for (const auto& point : cluster)
        {
            centroid += point;
        }

        centroid *= 1.0 / cluster.size();

        return centroid;
    }

     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    int frame_counter_;
    std::vector<std::vector<cv::Point2f>> previous_clusters;
    std::vector<std::vector<cv::Point2f>> clusters_;
    std::chrono::high_resolution_clock::time_point last_scan_time_;
    double frame_time_difference_; // Change frame_time_difference_ to double
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
