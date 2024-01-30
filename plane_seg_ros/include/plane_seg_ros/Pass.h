#ifndef PASS_H
#define PASS_H

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>
#include "plane_seg/BlockFitter.hpp"
#include <tf2_ros/transform_listener.h>

class Pass{
  public:
    Pass(ros::NodeHandle node_);
    ~Pass() = default;

    void elevationMapCallback(const grid_map_msgs::GridMap& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void processCloud(const std::string& cloudFrame, planeseg::LabeledCloud::Ptr& inCloud,
                      Eigen::Vector3f origin, Eigen::Vector3f lookDir);
    void processFromFile(int test_example);

    /**
     * @brief publishHullsAsCloud publishes the hull points.
     */
    void publishHullsAsCloud(const std::string& cloud_frame, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ptrs, int secs, int nsecs) const;

    /**
     * @brief publishHullsAsMarkers publishes line markers. Each message is a marker, the number of points is double
     * so that it draws a line.
     */
    void publishHullsAsMarkers(const std::string& cloud_frame, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs, int secs, int nsecs) const;

    /**
     * @brief publishHullsAsMarkerArray publishes line markers similar to publishHullAsMarkers but the the data
     * are an array of markers, each marker represents a hull.
     */
    void publishHullsAsMarkerArray(const std::string& cloud_frame, std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs, int secs, int nsecs) const;
    /**
     * @brief publishHullPose publishes the pose of the hull
     * @param ref_frame is the frame to which the input pointcloud is expressed.
     */
    void publishHullPose(const std::string& ref_frame) const;
    void printResultAsJson();
    void publishResult(const std::string& cloud_frame) const;

  private:
    std::vector<double> colors_;

    ros::Subscriber point_cloud_sub_, grid_map_sub_, pose_sub_;
    ros::Publisher received_cloud_pub_, hull_cloud_pub_, hull_markers_pub_, look_pose_pub_, hull_marker_array_pub_;

    std::string fixed_frame_ = "odometry/world";  // Frame in which all results are published. "odom" for backwards-compatibility. Likely should be "map".

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    planeseg::BlockFitter::Result result_;
    planeseg::BlockFitter::Settings settings_;

    // usefull in case pointcloud is wrt fixed frame
    std::string camera_frame_;
    bool pcld_in_fixed_frame_;
};

#endif // PASS_H
