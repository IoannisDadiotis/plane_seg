#include <plane_seg_ros/Pass.h>
#include <plane_seg_ros/common/Convenience.h>

// ros
#include <ros/package.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <plane_seg_ros/common/RosMsgConversions.h>

// ros msgs
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <plane_seg_ros/convex_hulls.h>

// tf
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <eigen_conversions/eigen_msg.h>

Eigen::Vector3f convertRobotPoseToSensorLookDir(Eigen::Isometry3d robot_pose) {
    return Eigen::Vector3f(robot_pose.linear()(0, 2), robot_pose.linear()(1, 2), robot_pose.linear()(2, 2)).normalized();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Pass::Pass(ros::NodeHandle node_):
    tfBuffer_(ros::Duration(5.0)), tfListener_(tfBuffer_) {
  // get ros topic from ros server
  std::string pointCloudTopic, elevationMapTopic, configFilePath;
  node_.getParam("/plane_seg/pointcloud_topic", pointCloudTopic);
  node_.getParam("/plane_seg/elevation_map_topic", elevationMapTopic);
  node_.getParam("/plane_seg/camera_frame", camera_frame_);
  node_.getParam("/plane_seg/configFile", configFilePath);                      // get info file
  settings_ = planeseg::loadFitterSettings(configFilePath, "fitter", true);     // get settings
  loadData::loadCppDataType(configFilePath, "fixedFrame", fixed_frame_);          // get fixed frame name

  // subscribers
  grid_map_sub_ = node_.subscribe(elevationMapTopic, 1, &Pass::elevationMapCallback, this);
  point_cloud_sub_ = node_.subscribe(pointCloudTopic, 1, &Pass::pointCloudCallback, this);

  // publishers
  received_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/received_cloud", 10);
  hull_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/plane_seg/hull_cloud", 10);
  hull_markers_pub_ = node_.advertise<visualization_msgs::Marker>("/plane_seg/hull_markers", 10);
  hull_marker_array_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/plane_seg/hull_marker_array", 10);
  look_pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/plane_seg/look_pose", 10);
  detected_hulls_pub_ = node_.advertise<plane_seg_ros::convex_hulls>("/plane_seg/detected_hulls", 10);

  // check if pointcloud frame is the fixed frame, then
  boost::shared_ptr<sensor_msgs::PointCloud2 const> pointcloudMsgPtr = nullptr;
  while (pointcloudMsgPtr == nullptr) {
      ROS_INFO_STREAM("Waiting for a first pointcloud message..");
      pointcloudMsgPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pointCloudTopic, node_);
  }
  if (pointcloudMsgPtr->header.frame_id == fixed_frame_) {
    pcld_in_fixed_frame_ = true;
    ROS_INFO_STREAM("Input pointcloud has fixed reference frame " << fixed_frame_);
  } else {
    pcld_in_fixed_frame_ = false;
    ROS_INFO_STREAM("Input pointcloud has reference frame " << fixed_frame_);
  }

  colors_ = {
       51/255.0, 160/255.0, 44/255.0,  //0
       166/255.0, 206/255.0, 227/255.0,
       178/255.0, 223/255.0, 138/255.0,//6
       31/255.0, 120/255.0, 180/255.0,
       251/255.0, 154/255.0, 153/255.0,// 12
       227/255.0, 26/255.0, 28/255.0,
       253/255.0, 191/255.0, 111/255.0,// 18
       106/255.0, 61/255.0, 154/255.0,
       255/255.0, 127/255.0, 0/255.0, // 24
       202/255.0, 178/255.0, 214/255.0,
       1.0, 0.0, 0.0, // red // 30
       0.0, 1.0, 0.0, // green
       0.0, 0.0, 1.0, // blue// 36
       1.0, 1.0, 0.0,
       1.0, 0.0, 1.0, // 42
       0.0, 1.0, 1.0,
       0.5, 1.0, 0.0,
       1.0, 0.5, 0.0,
       0.5, 0.0, 1.0,
       1.0, 0.0, 0.5,
       0.0, 0.5, 1.0,
       0.0, 1.0, 0.5,
       1.0, 0.5, 0.5,
       0.5, 1.0, 0.5,
       0.5, 0.5, 1.0,
       0.5, 0.5, 1.0,
       0.5, 1.0, 0.5,
       0.5, 0.5, 1.0};

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::elevationMapCallback(const grid_map_msgs::GridMap& msg){
  //std::cout << "got grid map / ev map\n";

  // convert message to GridMap, to PointCloud to LabeledCloud
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map, "elevation", pointCloud);
  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::fromROSMsg(pointCloud,*inCloud);

  // Expressed in is first argument
  if (tfBuffer_.canTransform(fixed_frame_, map.getFrameId(), msg.info.header.stamp, ros::Duration(0.02)))
  {
    // Look up transform of elevation map frame when it was captured
    geometry_msgs::TransformStamped fixed_frame_to_elevation_map_frame_tf;
    Eigen::Isometry3d map_T_elevation_map;

    fixed_frame_to_elevation_map_frame_tf = tfBuffer_.lookupTransform(fixed_frame_, map.getFrameId(), msg.info.header.stamp, ros::Duration(0.02));
    map_T_elevation_map = tf2::transformToEigen(fixed_frame_to_elevation_map_frame_tf);

    Eigen::Vector3f origin, lookDir;
    origin << map_T_elevation_map.translation().cast<float>();
    lookDir = convertRobotPoseToSensorLookDir(map_T_elevation_map);

    // ROS_INFO_STREAM("Received new elevation map & looked up transform -- processing.");
    processCloud(map.getFrameId(), inCloud, origin, lookDir);
  }
  else
  {
    ROS_WARN_STREAM("Cannot look up transform from '" << map.getFrameId() << "' to fixed frame ('" << fixed_frame_ <<"'). Skipping elevation map.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  pcl::fromROSMsg(*msg,*inCloud);

  std::string originalCloudFrame;
  if (pcld_in_fixed_frame_) {
      originalCloudFrame = camera_frame_;
  } else {
      originalCloudFrame = msg->header.frame_id;
  }

  // Look up transform from fixed frame to point cloud frame
  geometry_msgs::TransformStamped fixed_frame_to_cloud_frame_tf;
  Eigen::Isometry3d map_T_pointcloud;
  if (tfBuffer_.canTransform(fixed_frame_, originalCloudFrame, msg->header.stamp, ros::Duration(0.0)))
  {
    fixed_frame_to_cloud_frame_tf = tfBuffer_.lookupTransform(fixed_frame_, originalCloudFrame, msg->header.stamp, ros::Duration(0.0));
    map_T_pointcloud = tf2::transformToEigen(fixed_frame_to_cloud_frame_tf);
  }
  else
  {
    ROS_WARN_STREAM("Cannot look up transform from '" << originalCloudFrame << "' to fixed frame ('" << fixed_frame_ <<"')");
  }

  Eigen::Vector3f origin, lookDir;
  origin << map_T_pointcloud.translation().cast<float>();       // from fixed frame to point cloud frame
  lookDir = convertRobotPoseToSensorLookDir(map_T_pointcloud);

  processCloud(msg->header.frame_id, inCloud, origin, lookDir);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::processCloud(const std::string& cloudFrame, planeseg::LabeledCloud::Ptr& inCloud, Eigen::Vector3f origin, Eigen::Vector3f lookDir) {
#ifdef WITH_TIMING
  auto tic = std::chrono::high_resolution_clock::now();
#endif

  planeseg::BlockFitter fitter(settings_);
  fitter.setSensorPose(origin, lookDir);
  fitter.setCloud(inCloud);
  result_ = fitter.go();
#ifdef WITH_TIMING
  auto toc_1 = std::chrono::high_resolution_clock::now();
#endif

  // publish look pose
  if (look_pose_pub_.getNumSubscribers() > 0) {
    Eigen::Vector3f rx = lookDir;
    Eigen::Vector3f ry = rx.cross(Eigen::Vector3f::UnitZ());
    Eigen::Vector3f rz = rx.cross(ry);
    Eigen::Matrix3f rotation;
    rotation.col(0) = rx.normalized();
    rotation.col(1) = ry.normalized();
    rotation.col(2) = rz.normalized();
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = rotation;
    pose.translation() = origin;
    Eigen::Isometry3d pose_d = pose.cast<double>();

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time(0, 0);
    msg.header.frame_id = cloudFrame;
    tf::poseEigenToMsg(pose_d, msg.pose);
    look_pose_pub_.publish(msg);
  }

  // publish input cloud
  if (received_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*inCloud, output);
    output.header.stamp = ros::Time(0, 0);
    output.header.frame_id = cloudFrame;
    received_cloud_pub_.publish(output);
  }
  // publish
  publishResult(cloudFrame);

#ifdef WITH_TIMING
  auto toc_2 = std::chrono::high_resolution_clock::now();

  std::cout << "[BlockFitter] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc_1 - tic).count() << "ms" << std::endl;
  // std::cout << "[Publishing] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc_2 - toc_1).count() << "ms" << std::endl;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::printResultAsJson(){
  std::string json;

  for (int i = 0; i < (int)result_.mBlocks.size(); ++i) {
    const auto& block = result_.mBlocks[i];
    std::string dimensionString = vecToStr(block.mSize);
    std::string positionString = vecToStr(block.mPose.translation());
    std::string quaternionString = rotToStr(block.mPose.rotation());
    Eigen::Vector3f color(0.5, 0.4, 0.5);
    std::string colorString = vecToStr(color);
    float alpha = 1.0;
    std::string uuid = "0_" + std::to_string(i+1);

    json += "    \"" + uuid + "\": {\n";
    json += "      \"classname\": \"BoxAffordanceItem\",\n";
    json += "      \"pose\": [[" + positionString + "], [" +
      quaternionString + "]],\n";
    json += "      \"uuid\": \"" + uuid + "\",\n";
    json += "      \"Dimensions\": [" + dimensionString + "],\n";
    json += "      \"Color\": [" + colorString + "],\n";
    json += "      \"Alpha\": " + std::to_string(alpha) + ",\n";
    json += "      \"Name\": \" mNamePrefix " +
      std::to_string(i) + "\"\n";
    json += "    },\n";

  }

  std::cout << json << "\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::publishResult(const std::string& cloud_frame) const {
  // convert result to a vector of point clouds
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs;
  for (size_t i=0; i<result_.mBlocks.size(); ++i){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const auto& block = result_.mBlocks[i];
    cloud.points.reserve(block.mHull.size());
    for (size_t j =0; j < block.mHull.size(); ++j){
      pcl::PointXYZ pt;
      pt.x =block.mHull[j](0);
      pt.y =block.mHull[j](1);
      pt.z =block.mHull[j](2);
      cloud.points.push_back(pt);
    }
    cloud.height = cloud.points.size();
    cloud.width = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    cloud_ptr = cloud.makeShared();
    cloud_ptrs.push_back(cloud_ptr);
  }

  if (hull_cloud_pub_.getNumSubscribers() > 0) publishHullsAsCloud(cloud_frame, cloud_ptrs, 0, 0);
  if (hull_markers_pub_.getNumSubscribers() > 0) publishHullsAsMarkers(cloud_frame, cloud_ptrs, 0, 0);
  if (hull_marker_array_pub_.getNumSubscribers() > 0) publishHullsAsMarkerArray(cloud_frame, cloud_ptrs, 0, 0);
  if (detected_hulls_pub_.getNumSubscribers() > 0) publishDetectedHulls(fixed_frame_);
  publishHullPoseAsTransfomation(cloud_frame);
//  printResultAsJson();

  //pcl::PCDWriter pcd_writer_;
  //pcd_writer_.write<pcl::PointXYZ> ("/home/mfallon/out.pcd", cloud, false);
  //std::cout << "blocks: " << result_.mBlocks.size() << " blocks\n";
  //std::cout << "cloud: " << cloud.points.size() << " pts\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// combine the individual clouds into one, with a different each
void Pass::publishHullsAsCloud(const std::string& cloud_frame,
                               std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                               int secs, int nsecs) const {
  pcl::PointCloud<pcl::PointXYZRGB> combined_cloud;
  for (size_t i=0; i<cloud_ptrs.size(); ++i){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_ptrs[i], *cloud_rgb);

    int nColor = i % (colors_.size()/3);
    double r = colors_[nColor*3]*255.0;
    double g = colors_[nColor*3+1]*255.0;
    double b = colors_[nColor*3+2]*255.0;
    for (size_t j = 0; j < cloud_rgb->points.size (); j++){
        cloud_rgb->points[j].r = r;
        cloud_rgb->points[j].g = g;
        cloud_rgb->points[j].b = b;
    }
    combined_cloud += *cloud_rgb;
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(combined_cloud, output);

  output.header.stamp = ros::Time(secs, nsecs);
  output.header.frame_id = cloud_frame;
  hull_cloud_pub_.publish(output);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::publishHullsAsMarkers(const std::string& cloud_frame,
                                 std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                 int secs, int nsecs) const {
  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::Marker marker;

  // define markers
  marker.header.frame_id = cloud_frame;
  marker.header.stamp = ros::Time(secs, nsecs);
  marker.ns = "hull lines";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST; //visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0;

  for (size_t i = 0; i < cloud_ptrs.size (); i++){
    int nColor = i % (colors_.size()/3);
    double r = colors_[nColor*3]*255.0;
    double g = colors_[nColor*3+1]*255.0;
    double b = colors_[nColor*3+2]*255.0;

    for (size_t j = 1; j < cloud_ptrs[i]->points.size (); j++){
      point.x = cloud_ptrs[i]->points[j-1].x;
      point.y = cloud_ptrs[i]->points[j-1].y;
      point.z = cloud_ptrs[i]->points[j-1].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);

      //
      point.x = cloud_ptrs[i]->points[j].x;
      point.y = cloud_ptrs[i]->points[j].y;
      point.z = cloud_ptrs[i]->points[j].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);
    }

    // start to end line:
    point.x = cloud_ptrs[i]->points[0].x;
    point.y = cloud_ptrs[i]->points[0].y;
    point.z = cloud_ptrs[i]->points[0].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);

    point.x = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].x;
    point.y = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].y;
    point.z = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);
  }
  marker.frame_locked = true;
  hull_markers_pub_.publish(marker);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::publishHullsAsMarkerArray(const std::string& cloud_frame,
                                     std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptrs,
                                     int secs, int nsecs) const {
  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::MarkerArray ma;

  for (size_t i = 0; i < cloud_ptrs.size (); i++){
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud_frame;
    marker.header.stamp = ros::Time(secs, nsecs);
    marker.ns = "hull_" + std::to_string(i);
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0;

    const int nColor = i % (colors_.size()/3);
    const double r = colors_[nColor*3]*255.0;
    const double g = colors_[nColor*3+1]*255.0;
    const double b = colors_[nColor*3+2]*255.0;

    marker.points.reserve(cloud_ptrs[i]->points.size());
    marker.colors.reserve(cloud_ptrs[i]->points.size());
    for (size_t j = 1; j < cloud_ptrs[i]->points.size(); j++){
      point.x = cloud_ptrs[i]->points[j-1].x;
      point.y = cloud_ptrs[i]->points[j-1].y;
      point.z = cloud_ptrs[i]->points[j-1].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);

      point.x = cloud_ptrs[i]->points[j].x;
      point.y = cloud_ptrs[i]->points[j].y;
      point.z = cloud_ptrs[i]->points[j].z;
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);
    }

    // start to end line:
    point.x = cloud_ptrs[i]->points[0].x;
    point.y = cloud_ptrs[i]->points[0].y;
    point.z = cloud_ptrs[i]->points[0].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);

    point.x = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].x;
    point.y = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].y;
    point.z = cloud_ptrs[i]->points[ cloud_ptrs[i]->points.size()-1 ].z;
    point_color.r = r;
    point_color.g = g;
    point_color.b = b;
    point_color.a = 1.0;
    marker.colors.push_back(point_color);
    marker.points.push_back(point);

    marker.frame_locked = true;
    ma.markers.push_back(marker);
  }
  hull_marker_array_pub_.publish(ma);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::publishHullPoseAsTransfomation(const std::string& ref_frame) const {
    int i = 0;
    for (auto& i_hull: result_.mBlocks) {
        static tf2_ros::TransformBroadcaster br;
        Eigen::Isometry3d planePoseD = static_cast<Eigen::Isometry3d>(i_hull.mPose);
        geometry_msgs::TransformStamped transformStamped = tf2::eigenToTransform(planePoseD);
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = ref_frame;
        transformStamped.child_frame_id = "hull_" + std::to_string(i);
        br.sendTransform(transformStamped);
        i++;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::publishDetectedHulls(const std::string& ref_frame) const {
    plane_seg_ros::convex_hulls hullsMsg;
    hullsMsg.header.frame_id = ref_frame;
    hullsMsg.header.stamp = ros::Time::now();
    hullsMsg.hulls.reserve(result_.mBlocks.size());

    for (auto& i_hull: result_.mBlocks) {
        plane_seg_ros::convex_hull hullMsg = createConvexHullMsg(i_hull);
        hullsMsg.hulls.push_back(hullMsg);
    }
    detected_hulls_pub_.publish(hullsMsg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Pass::processFromFile(int test_example){

  // to allow ros connections to register
  sleep(2);

  std::string inFile;
  std::string home_dir = ros::package::getPath("plane_seg_ros");
  Eigen::Vector3f origin, lookDir;
  if (test_example == 0){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/data/terrain/tilted-steps.pcd";
    origin <<0.248091, 0.012443, 1.806473;
    lookDir <<0.837001, 0.019831, -0.546842;
  }else if (test_example == 1){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/data/terrain/terrain_med.pcd";
    origin << -0.028862, -0.007466, 0.087855;
    lookDir << 0.999890, -0.005120, -0.013947;
  }else if (test_example == 2){ // LIDAR example from Atlas during DRC
    inFile = home_dir + "/data/terrain/terrain_close_rect.pcd";
    origin << -0.028775, -0.005776, 0.087898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 3){ // RGBD (Realsense D435) example from ANYmal
    inFile = home_dir + "/data/terrain/anymal/ori_entrance_stair_climb/06.pcd";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 4){ // Leica map
    inFile = home_dir + "/data/leica/race_arenas/RACE_crossplaneramps_sub1cm_cropped_meshlab_icp.ply";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }else if (test_example == 5){ // Leica map
    inFile = home_dir + "/data/leica/race_arenas/RACE_stepfield_sub1cm_cropped_meshlab_icp.ply";
    origin << -0.028775, -0.005776, 0.987898;
    lookDir << 0.999956, -0.005003, 0.007958;
  }

  std::cout << "\nProcessing test example " << test_example << "\n";
  std::cout << inFile << "\n";

  std::size_t found_ply = inFile.find(".ply");
  std::size_t found_pcd = inFile.find(".pcd");

  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());
  if (found_ply!=std::string::npos){
    std::cout << "readply\n";
    pcl::io::loadPLYFile(inFile, *inCloud);
  }else if (found_pcd!=std::string::npos){
    std::cout << "readpcd\n";
    pcl::io::loadPCDFile(inFile, *inCloud);
  }else{
    std::cout << "extension not understood\n";
    return;
  }

  processCloud(fixed_frame_, inCloud, origin, lookDir);
}
