#include "plane_seg/BlockFitter.hpp"
#include <pcl_ros/features/normal_3d.h>

#include <chrono>
#include <fstream>

// PCL's octree_key.h (included from convex_hull.h) uses anonymous structs and nested anonymous unions.
// These are GNU extensions - we want to ignore warnings about them, though.

#if defined(__clang__)
# pragma clang diagnostic push
#endif

#if defined(__clang__) && defined(__has_warning)
# if __has_warning( "-Wgnu-anonymous-struct" )
#  pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
# endif
# if __has_warning( "-Wnested-anon-types" )
#  pragma clang diagnostic ignored "-Wnested-anon-types"
# endif
#endif

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/pcd_io.h>

#if defined(__clang__)
# pragma clang diagnostic pop
#endif

#include "plane_seg/PlaneFitter.hpp"
#include "plane_seg/RobustNormalEstimator.hpp"
#include "plane_seg/PlaneSegmenter.hpp"
#include "plane_seg/RectangleFitter.hpp"

using namespace planeseg;

BlockFitter::BlockFitter()
{
  setSensorPose(Eigen::Vector3f(0,0,0), Eigen::Vector3f(1,0,0));
  setBlockDimensions(Eigen::Vector3f(15+3/8.0, 15+5/8.0, 5+5/8.0)*0.0254);
  setDownsampleResolution(0.01);
  setRemoveGround(true);
  setGroundBand(1e10,1e10);
  setHeightBand(0.05, 1.0);
  setMaxRange(3.0);
  setMaxAngleFromHorizontal(45);
  setMaxAngleOfPlaneSegmenter(5);
  setAreaThresholds(0.5, 1.5);
  setRectangleFitAlgorithm(RectangleFitAlgorithm::MinimumArea);
  setDebug(true);
  setUnreachableRejection(false, 0.0, Eigen::Vector3f(0, 0, 0));
}

BlockFitter::BlockFitter(const Settings& settings)
    : settings_(settings)
{
  setSensorPose(Eigen::Vector3f(0,0,0), Eigen::Vector3f(1,0,0));
  setBlockDimensions(Eigen::Vector3f(15+3/8.0, 15+5/8.0, 5+5/8.0)*0.0254);
  setRectangleFitAlgorithm(RectangleFitAlgorithm::MinimumArea);
}

void BlockFitter::
setSensorPose(const Eigen::Vector3f& iOrigin,
              const Eigen::Vector3f& iLookDir) {
  mOrigin = iOrigin;
  mLookDir = iLookDir;
}

void BlockFitter::
setBlockDimensions(const Eigen::Vector3f& iDimensions) {
  mBlockDimensions = iDimensions;
}

void BlockFitter::
setDownsampleResolution(const float iRes) {
  settings_.downsampleResolution = iRes;
}

void BlockFitter::
setRemoveGround(const bool iVal) {
  settings_.removeGround = iVal;
}

void BlockFitter::
setGroundBand(const float iMinZ, const float iMaxZ) {
  settings_.minGroundZ = iMinZ;
  settings_.maxGroundZ = iMaxZ;
}


void BlockFitter::
setHeightBand(const float iMinHeight, const float iMaxHeight) {
  settings_.minHeightAboveGround = iMinHeight;
  settings_.maxHeightAboveGround = iMaxHeight;
}

void BlockFitter::
setMaxRange(const float iRange) {
  settings_.maxRange = iRange;
}

void BlockFitter::
setMaxAngleFromHorizontal(const float iDegrees) {
  settings_.maxAngleFromHorizontal = iDegrees;
}

void BlockFitter::
setMaxAngleOfPlaneSegmenter(const float iDegrees) {
  settings_.maxAngleOfPlaneSegmenter = iDegrees;
}

void BlockFitter::
setAreaThresholds(const float iMin, const float iMax) {
  settings_.areaThreshMin = iMin;
  settings_.areaThreshMax = iMax;
}

void BlockFitter::
setRectangleFitAlgorithm(const RectangleFitAlgorithm iAlgo) {
  mRectangleFitAlgorithm = iAlgo;
}

void BlockFitter::
setCloud(const LabeledCloud::Ptr& iCloud) {
  mCloud = iCloud;
}

void BlockFitter::
setDebug(const bool iVal) {
  settings_.debug = iVal;
}

void BlockFitter::setSettings(const Settings& settings) {
    settings_ = settings;
}

void BlockFitter::setUnreachableRejection(const bool& active, const float& distance,
                                          const Eigen::Vector3f& referencePoint) {
    settings_.removeUnreachable = std::move(active);
    settings_.maximumDistance = std::move(distance);
    settings_.reachableReference = std::move(referencePoint);
}


BlockFitter::Result BlockFitter::
go() {
  Result result;
  result.mSuccess = false;

  if (mCloud->size() < 100) return result;

  // voxelize
  LabeledCloud::Ptr cloud(new LabeledCloud());
  pcl::VoxelGrid<pcl::PointXYZL> voxelGrid;
  voxelGrid.setInputCloud(mCloud);
  voxelGrid.setLeafSize(settings_.downsampleResolution, settings_.downsampleResolution,
                        settings_.downsampleResolution);
  voxelGrid.filter(*cloud);
  for (int i = 0; i < (int)cloud->size(); ++i) cloud->points[i].label = i;

  if (settings_.debug) {
    std::cout << "Original cloud size " << mCloud->size() << std::endl;
    std::cout << "Voxelized cloud size " << cloud->size() << std::endl;
    pcl::io::savePCDFileBinary("cloud_full.pcd", *cloud);
  }

  if (cloud->size() < 100) return result;

  // pose
  cloud->sensor_origin_.head<3>() = mOrigin;
  cloud->sensor_origin_[3] = 1;
//  Eigen::Vector3f rz = mLookDir;                              // not used so better to comment
//  Eigen::Vector3f rx = rz.cross(Eigen::Vector3f::UnitZ());
//  Eigen::Vector3f ry = rz.cross(rx);
//  Eigen::Matrix3f rotation;
//  rotation.col(0) = rx.normalized();
//  rotation.col(1) = ry.normalized();
//  rotation.col(2) = rz.normalized();
//  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
//  pose.linear() = rotation;
//  pose.translation() = mOrigin;

  // ground removal
  if (settings_.removeGround) {
    Eigen::Vector4f groundPlane;

    // filter points
    float minZ = settings_.minGroundZ;
    float maxZ = settings_.maxGroundZ;
    if ((minZ > 10000) && (maxZ > 10000)) {
      std::vector<float> zVals(cloud->size());
      for (int i = 0; i < (int)cloud->size(); ++i) {
        zVals[i] = cloud->points[i].z;
      }
      std::sort(zVals.begin(), zVals.end());
      minZ = zVals[0]-0.1;
      maxZ = minZ + 0.5;
    }
    LabeledCloud::Ptr tempCloud(new LabeledCloud());
    tempCloud->reserve(cloud->size());
    for (int i = 0; i < (int)cloud->size(); ++i) {
      const Eigen::Vector3f& p = cloud->points[i].getVector3fMap();
      if ((p[2] < minZ) || (p[2] > maxZ)) continue;
      tempCloud->push_back(cloud->points[i]);
    }

    // downsample
    voxelGrid.setInputCloud(tempCloud);
    voxelGrid.setLeafSize(0.1, 0.1, 0.1);
    voxelGrid.filter(*tempCloud);

    if (tempCloud->size() < 100) return result;

    // find ground plane
    std::vector<Eigen::Vector3f> pts(tempCloud->size());
    for (int i = 0; i < (int)tempCloud->size(); ++i) {
      pts[i] = tempCloud->points[i].getVector3fMap();
    }
    const float kGroundPlaneDistanceThresh = 0.01; // TODO: param
    PlaneFitter planeFitter;
    planeFitter.setMaxDistance(kGroundPlaneDistanceThresh);
    planeFitter.setRefineUsingInliers(true);
    auto res = planeFitter.go(pts);
    groundPlane = res.mPlane;
    if (groundPlane[2] < 0) groundPlane = -groundPlane;
    if (settings_.debug) {
      std::cout << "dominant plane: " << groundPlane.transpose() << std::endl;
      std::cout << "  inliers: " << res.mInliers.size() << std::endl;
    }

    if (std::acos(groundPlane[2]) > 30*M_PI/180) {
      std::cout << "error: ground plane not found!" << std::endl;
      std::cout << "proceeding with full segmentation (may take a while)..." <<
        std::endl;
    }

    else {
      // compute convex hull
      result.mGroundPlane = groundPlane;
      {
        tempCloud.reset(new LabeledCloud());
        tempCloud->reserve(cloud->size());
        for (int i = 0; i < (int)cloud->size(); ++i) {
          Eigen::Vector3f p = cloud->points[i].getVector3fMap();
          float dist = groundPlane.head<3>().dot(p) + groundPlane[3];
          if (std::abs(dist) > kGroundPlaneDistanceThresh) continue;
          p -= (groundPlane.head<3>()*dist);
          pcl::PointXYZL cloudPt;
          cloudPt.getVector3fMap() = p;
          tempCloud->push_back(cloudPt);
        }
        pcl::ConvexHull<pcl::PointXYZL> chull;
        pcl::PointCloud<pcl::PointXYZL> hull;
        chull.setInputCloud(tempCloud);
        chull.reconstruct(hull);
        result.mGroundPolygon.resize(hull.size());
        for (int i = 0; i < (int)hull.size(); ++i) {
          result.mGroundPolygon[i] = hull[i].getVector3fMap();
        }
      }

      // remove points below or near ground
      tempCloud.reset(new LabeledCloud());
      tempCloud->reserve(cloud->size());
      for (int i = 0; i < (int)cloud->size(); ++i) {
        Eigen::Vector3f p = cloud->points[i].getVector3fMap();
        float dist = p.dot(groundPlane.head<3>()) + groundPlane[3];
        if ((dist < settings_.minHeightAboveGround) ||
            (dist > settings_.maxHeightAboveGround)) continue;
        float range = (p-mOrigin).norm();
        if (range > settings_.maxRange) continue;
        tempCloud->push_back(cloud->points[i]);
      }
      std::swap(tempCloud, cloud);
      if (settings_.debug) {
        std::cout << "Filtered cloud size " << cloud->size() << std::endl;
      }
    }
  }

  // remove kinematically unreachable points
  if (settings_.removeUnreachable) {
      LabeledCloud::Ptr tempCloud(new LabeledCloud());
      tempCloud->reserve(cloud->size());

      for (int i = 0; i < (int)cloud->size(); ++i) {
        Eigen::Vector3f p = cloud->points[i].getVector3fMap();
        float distance = (p - settings_.reachableReference).norm();
        if (distance > settings_.maximumDistance) {
            continue;
        } else {
            tempCloud->push_back(cloud->points[i]);
        }
      }
      std::swap(tempCloud, cloud);
      if (settings_.debug) {
        std::cout << "Filtered kinematically cloud size " << cloud->size() << std::endl;
      }
  }

  // normal estimation
  auto t0 = std::chrono::high_resolution_clock::now();
  if (settings_.debug) {
    std::cout << "computing normals..." << std::flush;
  }
  RobustNormalEstimator normalEstimator;
  normalEstimator.setMaxEstimationError(0.01);
  normalEstimator.setRadius(0.1);
  normalEstimator.setMaxCenterError(0.02);
  normalEstimator.setMaxIterations(100);
  NormalCloud::Ptr normals(new NormalCloud());
  normalEstimator.go(cloud, *normals);
  if (settings_.debug) {
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
    std::cout << "finished in " << dt.count()/1e3 << " sec" << std::endl;
  }

  // filter non-horizontal points
  const float maxNormalAngle = settings_.maxAngleFromHorizontal*M_PI/180;
  LabeledCloud::Ptr tempCloud(new LabeledCloud());
  NormalCloud::Ptr tempNormals(new NormalCloud());
  tempCloud->reserve(normals->size());
  tempNormals->reserve(normals->size());
  for (int i = 0; i < (int)normals->size(); ++i) {
    // const auto& norm = normals->points[i];
    // Eigen::Vector3f normal(norm.normal_x, norm.normal_y, norm.normal_z);
    float angle = std::acos(normals->points[i].normal_z);  //std::acos(normal[2]);
    if (angle > maxNormalAngle) continue;
    tempCloud->push_back(cloud->points[i]);
    tempNormals->push_back(normals->points[i]);
  }
  std::swap(tempCloud, cloud);
  std::swap(tempNormals, normals);

  if (settings_.debug) {
    std::cout << "Horizontal points remaining " << cloud->size() << std::endl;
    pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
    pcl::io::savePCDFileBinary("robust_normals.pcd", *normals);
  }

  // plane segmentation
  t0 = std::chrono::high_resolution_clock::now();
  if (settings_.debug) {
    std::cout << "segmenting planes..." << std::flush;
  }
  PlaneSegmenter segmenter;
  segmenter.setData(cloud, normals);
  segmenter.setMaxError(0.05);
  // setMaxAngle was 5 for LIDAR. changing to 10 really improved elevation map segmentation
  // I think its because the RGB-D map can be curved
  segmenter.setMaxAngle(settings_.maxAngleOfPlaneSegmenter);
  segmenter.setMinPoints(100);
  PlaneSegmenter::Result segmenterResult = segmenter.go();
  if (settings_.debug) {
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
    std::cout << "finished in " << dt.count()/1e3 << " sec" << std::endl;

    std::ofstream ofs("labels.txt");
    for (const int lab : segmenterResult.mLabels) {
      ofs << lab << std::endl;
    }
    ofs.close();

    ofs.open("planes.txt");
    for (auto it : segmenterResult.mPlanes) {
      auto& plane = it.second;
      ofs << it.first << " " << plane.transpose() << std::endl;
    }
    ofs.close();
  }

  // create point clouds
  std::unordered_map<int,std::vector<Eigen::Vector3f>> cloudMap;
  for (int i = 0; i < (int)segmenterResult.mLabels.size(); ++i) {
    int label = segmenterResult.mLabels[i];
    if (label <= 0) continue;
    cloudMap[label].push_back(cloud->points[i].getVector3fMap());
  }
  // a plane is represented as (a,b,c,d) where (a,b,c) the norma vector and d the distance of the plane from the origin
  struct Plane {
    MatrixX3f mPoints;
    Eigen::Vector4f mPlane;
  };
  std::vector<Plane> planes;
  planes.reserve(cloudMap.size());
  for (auto it : cloudMap) {                    // actually loops over hulls
    int n = it.second.size();
    Plane plane;
    plane.mPoints.resize(n,3);
    for (int i = 0; i < n; ++i) plane.mPoints.row(i) = it.second[i];
    plane.mPlane = segmenterResult.mPlanes[it.first];

    // flip normal, only consider the first point of each hull
    pcl::PointXYZ firstPoint(plane.mPoints(0, 0) , plane.mPoints(0, 1), plane.mPoints(0, 2));
    pcl::flipNormalTowardsViewpoint(firstPoint, mOrigin[0], mOrigin[1], mOrigin[2], plane.mPlane);

    planes.push_back(plane);
  }

  std::vector<RectangleFitter::Result> results;
  results.reserve(planes.size());
  for (auto& plane : planes) {
    RectangleFitter fitter;
    fitter.setDimensions(mBlockDimensions.head<2>());
    fitter.setAlgorithm((RectangleFitter::Algorithm)mRectangleFitAlgorithm);
    fitter.setData(plane.mPoints, plane.mPlane);
    auto result = fitter.go();
    results.push_back(result);
  }

  if (settings_.debug) {
    std::ofstream ofs("boxes.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      for (auto& p : res.mPolygon) {
        ofs << i << " " << p.transpose() << std::endl;
      }
    }
    ofs.close();

    ofs.open("hulls.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      for (auto& p : res.mConvexHull) {
        ofs << i << " " << p.transpose() << std::endl;
      }
    }
    ofs.close();

    ofs.open("poses.txt");
    for (int i = 0; i < (int)results.size(); ++i) {
      auto& res = results[i];
      auto transform = res.mPose;
      ofs << transform.matrix() << std::endl;
    }
    ofs.close();
  }

  for (int i = 0; i < (int)results.size(); ++i) {
    const auto& res = results[i];

    // These seems to filter out hulls which are not of similar size to the DRC
    // blocks. I think this irrelevent for general use
    // if (mBlockDimensions.head<2>().norm() > 1e-5) {
    //   float areaRatio = mBlockDimensions.head<2>().prod()/res.mConvexArea;
    //   // std::cout << mBlockDimensions.transpose() << " | " << res.mConvexArea << " | " << areaRatio << " " << i << "\n";
    //   if ((areaRatio < mAreaThreshMin) ||
    //       (areaRatio > mAreaThreshMax)) continue;
    // }

    Block block;
    block.mSize << res.mSize[0], res.mSize[1], mBlockDimensions[2];     // why hardcoded z direction?
    block.mPose = res.mPose;
    // comment offset along z axis of hull pose, not sure if needed unless for visualizaiton reasons
//    block.mPose.translation() -= block.mPose.rotation().col(2)*mBlockDimensions[2]/2;
    block.mHull = res.mConvexHull;
    result.mBlocks.push_back(block);
  }
  if (settings_.debug) {
    std::cout << "Surviving blocks: " << result.mBlocks.size() << std::endl;
  }

  result.mSuccess = true;
  return result;
}
