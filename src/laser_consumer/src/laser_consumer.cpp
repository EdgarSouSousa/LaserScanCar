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

        // Initialize OpenCV window for visualization
        cv::namedWindow("Raw Data", cv::WINDOW_NORMAL);
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
        const float stability_threshold = 2.0; // Adjust this value according to your needs

        // Calculate scan rate
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_scan_time_);
        double scan_rate = 1000.0 / elapsed_time.count();  // Convert to scans per second
        last_scan_time_ = current_time;

        // Print scan rate
        std::cout << "Scan Rate: " << scan_rate << " scans per second" << std::endl;

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

        //print the number of points
        std::cout << "Number of points: " << points.size() << std::endl;

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

        for (const auto& cluster : clusters)
        {
            // Transform cluster points back to image coordinates
            std::vector<cv::Point> cluster_img;
            for (const auto& point : cluster)
            {
                int x = static_cast<int>((point.x + max_range) / (2 * max_range) * image_size);
                int y = static_cast<int>((max_range - point.y) / (2 * max_range) * image_size);

                if (x >= 0 && x < image_size && y >= 0 && y < image_size)
                {
                    cluster_img.push_back(cv::Point(x, y));
                }
            }

            // Draw bounding box
            if (!cluster_img.empty())
            {
                cv::Rect bounding_box = cv::boundingRect(cluster_img);
                cv::rectangle(objects_img, bounding_box, cv::Scalar(255, 255, 255), 2);
            }
        }

        // Visualize
        cv::imshow("Raw Data", img);
        cv::imshow("Detected Objects", objects_img);
        cv::waitKey(1);

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

    // Check the stability of clusters over consecutive frames
    void checkClusterStability(const std::vector<std::vector<cv::Point2f>>& current_clusters)
    {
        for (size_t i = 0; i < current_clusters.size(); ++i)
        {
            const auto& current_cluster = current_clusters[i];

            // Find the corresponding cluster in the previous frame
            if (i < previous_clusters_.size())
            {
                const auto& previous_cluster = previous_clusters_[i];

                // Calculate centroids of the clusters
                cv::Point2f current_centroid = calculateCentroid(current_cluster);
                cv::Point2f previous_centroid = calculateCentroid(previous_cluster);

                // Calculate the distance between centroids
                float distance = cv::norm(current_centroid - previous_centroid);

                // Check if the cluster is stable based on the distance threshold
                if (distance < stability_threshold_)
                {
                    // Mark the cluster as stable (you can store this information for further analysis)
                    std::cout << "Cluster " << i << " is stable.\n";
                }
                else
                {
                    // Mark the cluster as unstable (you can store this information for further analysis)
                    std::cout << "Cluster " << i << " is unstable.\n";
                }
            }
        }

        // Update the previous clusters for the next frame
        previous_clusters_ = current_clusters;
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
    std::vector<std::vector<cv::Point2f>> previous_clusters_;
    float stability_threshold_ = 2.0; // Adjust this value according to your needs
    std::chrono::high_resolution_clock::time_point last_scan_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
