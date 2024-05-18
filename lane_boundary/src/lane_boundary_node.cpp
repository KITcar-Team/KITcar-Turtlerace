#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std;

class LaneBoundaryNode : public rclcpp::Node
{
public:
  LaneBoundaryNode() : Node("lane_boundary")
  {
    this->declare_parameter("track_file_path", "./KITcar_strecke.csv");
    track_file_path_ = this->get_parameter("track_file_path").as_string();
    this->declare_parameter("track_frame_id", "map");
    frame_id_ = this->get_parameter("track_frame_id").as_string();

    this->declare_parameter("track_scaling", true);
    is_track_scaling_ = this->get_parameter("track_scaling").as_bool();
    this->declare_parameter("x_dimension", 1920);
    int x_dimension = this->get_parameter("x_dimension").as_int();
    this->declare_parameter("y_dimension", 1080);
    int y_dimension = this->get_parameter("y_dimension").as_int();
    this->declare_parameter("x_offset", 0.0);
    int x_offset = this->get_parameter("x_offset").as_double();
    this->declare_parameter("y_offset", 0.0);
    int y_offset = this->get_parameter("y_offset").as_double();

    load_lane_boundary();
    if (is_track_scaling_)
      scale_track(x_dimension, y_dimension, x_offset, y_offset);

    publisher_left_ = this->create_publisher<nav_msgs::msg::Path>("lane_boundary_left", 5);
    publisher_middle_ = this->create_publisher<nav_msgs::msg::Path>("lane_boundary_middle", 5);
    publisher_right_ = this->create_publisher<nav_msgs::msg::Path>("lane_boundary_right", 5);

    timer_ = this->create_wall_timer(
        17ms, std::bind(&LaneBoundaryNode::publish_paths, this));
  }

private:
  void publish_paths()
  {
    update_path(path_left_);
    publisher_left_->publish(path_left_);

    update_path(path_middle_);
    publisher_middle_->publish(path_middle_);

    update_path(path_right_);
    publisher_right_->publish(path_right_);
  }

  void update_path(nav_msgs::msg::Path &path)
  {
    rclcpp::Time stamp = path.header.stamp = this->get_clock()->now();

    path.header.frame_id = frame_id_;
    path.header.stamp = stamp;

    for (auto &pose : path.poses)
    {
      pose.header.frame_id = frame_id_;
      pose.header.stamp = stamp;
    }
  }

  void load_lane_boundary()
  {
    ifstream file(track_file_path_.c_str());
    if (!file.is_open())
    {
      RCLCPP_ERROR(get_logger(), "Failed to open track-file: %s", track_file_path_.c_str());
      return;
    }

    string line;
    double lx, ly, mx, my, rx, ry;
    while (getline(file, line))
    {
      sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &lx, &ly, &mx, &my, &rx, &ry);

      path_left_.poses.push_back(coordinates_to_pose(lx, ly));
      path_middle_.poses.push_back(coordinates_to_pose(mx, my));
      path_right_.poses.push_back(coordinates_to_pose(rx, ry));
    }

    file.close();
  }

  geometry_msgs::msg::PoseStamped coordinates_to_pose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped pose;

    pose.pose.position.x = x;
    pose.pose.position.y = y;

    pose.pose.orientation.w = 1;

    return pose;
  }

  void scale_track(int x_dimension, int y_dimension, double x_offset, double y_offset)
  {
    double max_x;
    double min_x;
    double max_y;
    double min_y;

    // Concatenate all poses to a single vector
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(path_left_.poses.size() + path_middle_.poses.size() + path_right_.poses.size());
    poses.insert(poses.end(), path_left_.poses.begin(), path_left_.poses.end());
    poses.insert(poses.end(), path_middle_.poses.begin(), path_middle_.poses.end());
    poses.insert(poses.end(), path_right_.poses.begin(), path_right_.poses.end());

    // Return if there are no poses
    if (poses.empty())
      return;

    // Find min and max values of x and y
    max_x = min_x = poses[0].pose.position.x;
    max_y = min_y = poses[0].pose.position.y;
    for (auto &pose : poses)
    {
      max_x = max(max_x, pose.pose.position.x);
      min_x = min(min_x, pose.pose.position.x);
      max_y = max(max_y, pose.pose.position.y);
      min_y = min(min_y, pose.pose.position.y);
    }

    // Calculate scale factor
    double scale_x = x_dimension / (max_x - min_x);
    double scale_y = y_dimension / (max_y - min_y);
    // Use the smaller scale factor so that the track does not get cramped
    double scale = min(scale_x, scale_y);

    scale_poses(path_left_.poses, scale, min_x, min_y, x_offset, y_offset);
    scale_poses(path_middle_.poses, scale, min_x, min_y, x_offset, y_offset);
    scale_poses(path_right_.poses, scale, min_x, min_y, x_offset, y_offset);
  }

  void scale_poses(std::vector<geometry_msgs::msg::PoseStamped> &poses, double scale,
                   double x_shift, double y_shift, double x_offset, double y_offset)
  {
    for (auto &pose : poses)
    {
      // shift poses to center, scale according to the smaller dimension and move to the given offset
      pose.pose.position.x = (pose.pose.position.x - x_shift) * scale + x_offset;
      pose.pose.position.y = (pose.pose.position.y - y_shift) * scale + y_offset;
    }
  }

  string frame_id_;
  string track_file_path_;

  bool is_track_scaling_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_left_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_middle_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_right_;

  nav_msgs::msg::Path path_left_;
  nav_msgs::msg::Path path_middle_;
  nav_msgs::msg::Path path_right_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneBoundaryNode>());
  rclcpp::shutdown();
  return 0;
}