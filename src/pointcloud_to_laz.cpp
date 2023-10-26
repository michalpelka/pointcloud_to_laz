#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <boost/program_options.hpp>

#include "save_laz.h"

class PointCloudSubscriberNode : public rclcpp::Node
{
public:
    PointCloudSubscriberNode(const std::string& pointcloud_topic, const std::string& imu_topic, float aggregation_time ) : Node("point_cloud_subscriber_node")
    {
        // Create a subscriber to receive PointCloud2 messages
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                pointcloud_topic, rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort(),
            std::bind(&PointCloudSubscriberNode::pointCloudCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort(),
            std::bind(&PointCloudSubscriberNode::imuCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<int, std::milli>(static_cast<int>(aggregation_time*1000)), // 1000 ms = 1 second interval
            std::bind(&PointCloudSubscriberNode::timerCallback, this));

    }

private:
    void timerCallback() {

        int count = count_;

        char fn[100];
        if (buffer_.size() > 0 && imu_buffer_.size() > 0)
        {
            sprintf(fn, "pointcloud_%04d.laz", count);
            mandeye::saveLaz(fn, buffer_);
            RCLCPP_INFO(this->get_logger(), "Saved %d points to %s", buffer_.size(), fn);

            sprintf(fn, "imu_%04d.csv", count);
            std::ofstream f(fn);
            for (auto& imu : imu_buffer_)
            {
                uint64_t ts = imu.header.stamp.sec * 1e9 + imu.header.stamp.nanosec;
                f << ts << " "
                  << imu.angular_velocity.x << " "
                  << imu.angular_velocity.y << " "
                  << imu.angular_velocity.z << " "
                  << imu.linear_acceleration.x << " "
                  << imu.linear_acceleration.y << " "
                  << imu.linear_acceleration.z << std::endl;
            }
            imu_buffer_.clear();
            buffer_.clear();
            count_++;
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_buffer_.push_back(*msg);
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            mandeye::Point p;

            p.xyz = { *iter_x, *iter_y, *iter_z };
            if (std::isnan(p.xyz[0]) || std::isnan(p.xyz[1]) || std::isnan(p.xyz[2]))
                continue;
            p.timestamp = cloud_msg->header.stamp.sec * 1e9 + cloud_msg->header.stamp.nanosec ;
//            std::cout << p.timestamp << std::endl;
            if (abs(p.xyz[0]) >100 && abs(p.xyz[1]) >100 && abs(p.xyz[2]) >100)
                continue;
            buffer_.push_back(p);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
    std::vector<mandeye::Point> buffer_;
    std::vector<sensor_msgs::msg::Imu> imu_buffer_;

};

int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    rclcpp::init(argc, argv);

    std::string pointcloud_topic;
    std::string imu_topic;
    float aggregation_time = 5.0;

    po::options_description desc("Allowed options");
    desc.add_options()
            ("pointcloud_topic,p", po::value<std::string>(), "Pointcloud topic")
            ("imu_topic,i", po::value<std::string>(), "IMU topic")
            ("aggregation_time,a", po::value<float>(), "Aggregation time in seconds")
            ;

    po::variables_map vm;

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("pointcloud_topic")) {
            pointcloud_topic = vm["pointcloud_topic"].as<std::string>();
        }

        if (vm.count("imu_topic")) {
            imu_topic = vm["imu_topic"].as<std::string>();
        }

        if (vm.count("aggregation_time")) {
            aggregation_time = vm["aggregation_time"].as<float>();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Pointcloud topic: " << pointcloud_topic << std::endl;
    std::cout << "IMU topic: " << imu_topic << std::endl;
    std::cout << "Aggregation time: " << aggregation_time << " seconds" << std::endl;



    auto node = std::make_shared<PointCloudSubscriberNode>(pointcloud_topic, imu_topic, aggregation_time);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
