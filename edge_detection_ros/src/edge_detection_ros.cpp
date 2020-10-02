#include "edge_detection_ros/edge_detection_ros.h"

namespace towr {

    EdgeDetectionRos::EdgeDetectionRos(ros::NodeHandle &node_handle, double min_length, double min_height) :
            node_handle_(node_handle), EdgeDetection(node_handle, min_length, min_height) {

      elevation_map_sub_ = node_handle_.subscribe("elevation_mapping/elevation_map", 1, &towr::EdgeDetectionRos::UpdateEdges, this);
      anymal_state_sub_ = node_handle_.subscribe("/state_estimator/anymal_state", 1, &towr::EdgeDetectionRos::ReadAnymalState, this);
      edge_pub_ = node_handle_.advertise<edge_detection::EdgeArray>("/edge_detection/edge_array", 1000);
      //state_world_yaw_prev_ = 0.0;

    }

    EdgeDetectionRos::~EdgeDetectionRos() {
    }

    void EdgeDetectionRos::ReadAnymalState(const anymal_msgs::AnymalState & anymal_state_msg) {
      robot_state_[0] = anymal_state_msg.pose.pose.position.x;
      robot_state_[1]= anymal_state_msg.pose.pose.position.y;

      Eigen::Quaterniond state_world_q(anymal_state_msg.pose.pose.orientation.w, anymal_state_msg.pose.pose.orientation.x, anymal_state_msg.pose.pose.orientation.y, anymal_state_msg.pose.pose.orientation.z);
      Eigen::Vector3d euler = state_world_q.toRotationMatrix().eulerAngles(2, 1, 0);
      double yaw = euler[0]; //pitch = euler[1]; roll = euler[2];
      robot_state_[2] = yaw;
    }

    void EdgeDetectionRos::UpdateEdges(const grid_map_msgs::GridMap&  grid_map_in) {
      advance(robot_state_, grid_map_in);

      if(edgeFound()){

        Eigen::Vector2d middle_point_wf = getPointAlongEdgeInWorldFrame();
        edge_detection::EdgeArray edges_array;
        edge_detection::Edge new_edge;
        geometry_msgs::Pose2D edge_pose;
        new_edge.step_height = getNextStepHeight();
        edge_pose.x = middle_point_wf[0];
        edge_pose.y = middle_point_wf[1];
        edge_pose.theta = getEdgeYawAngleInWorldFrame();
        new_edge.pose = edge_pose;
        edges_array.edges.push_back(new_edge);
        edge_pub_.publish(edges_array);
      }

      plotEdges();
    }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "edge_detection_ros");
  ros::NodeHandle node_handle;
  towr::EdgeDetectionRos edge_detection_ros(node_handle, 0.8, 0.03);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  return 0;
}
