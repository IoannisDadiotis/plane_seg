#ifndef _planeseg_BlockFitter_hpp_
#define _planeseg_BlockFitter_hpp_

#include "Types.hpp"
#include <plane_seg/common/LoadData.h>

namespace planeseg {

class BlockFitter {
public:
  enum RectangleFitAlgorithm {
    MinimumArea,
    ClosestToPriorSize,
    MaximumHullPointOverlap
  };

  /**
   * @brief The Block struct seems to represent a detected plane/hull.
   */
  struct Block {
    Eigen::Vector3f mSize;              // size of plane
    Eigen::Isometry3f mPose;            // pose
    std::vector<Eigen::Vector3f> mHull; // vector of the points that define the hull
  };
  struct Result {
    bool mSuccess;
    std::vector<Block> mBlocks;         // number of planes/hulls detected
    Eigen::Vector4f mGroundPlane;
    std::vector<Eigen::Vector3f> mGroundPolygon;
  };

  struct Settings {
      bool debug = false;
      bool removeGround = false;
      float maxAngleOfPlaneSegmenter = 10.0;
      float maxAngleFromHorizontal = 120.0;
      float downsampleResolution = 0.01;
      float maxRange = 3.0;

      float minGroundZ;
      float maxGroundZ;
      float minHeightAboveGround;
      float maxHeightAboveGround;
      float areaThreshMin;
      float areaThreshMax;

      bool removeUnreachable = false;
      float maximumDistance = 0.0;
      Eigen::Vector3f reachableReference;
  };

public:
  BlockFitter();
  BlockFitter(const Settings& settings);

  // iLookDir is a viewing normal in computer vision coordinates
  // i.e. z is forward
  void setSensorPose(const Eigen::Vector3f& iOrigin,
                     const Eigen::Vector3f& iLookDir);
  void setBlockDimensions(const Eigen::Vector3f& iDimensions);
  void setDownsampleResolution(const float iRes);
  void setRemoveGround(const bool iVal);
  void setGroundBand(const float iMinZ, const float iMaxZ);
  void setHeightBand(const float iMinHeight, const float iMaxHeight);
  void setMaxRange(const float iRange);
  void setMaxAngleFromHorizontal(const float iDegrees);
  void setMaxAngleOfPlaneSegmenter(const float iDegrees);
  void setAreaThresholds(const float iMin, const float iMax);
  void setRectangleFitAlgorithm(const RectangleFitAlgorithm iAlgo);
  void setDebug(const bool iVal);
  void setCloud(const LabeledCloud::Ptr& iCloud);
  void setUnreachableRejection(const bool& active, const float& distance, const Eigen::Vector3f& referencePoint);
  void setSettings(const Settings& settings);

  Result go();

protected:
  Eigen::Vector3f mOrigin;
  Eigen::Vector3f mLookDir;
  Eigen::Vector3f mBlockDimensions;
  RectangleFitAlgorithm mRectangleFitAlgorithm;
  LabeledCloud::Ptr mCloud;

  Settings settings_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline BlockFitter::Settings loadFitterSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  BlockFitter::Settings fitterSettings;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### Block Fitter Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, fitterSettings.debug, fieldName + ".debug", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.removeGround, fieldName + ".removeGround", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.maxAngleOfPlaneSegmenter, fieldName + ".maxAngleOfPlaneSegmenter", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.maxAngleFromHorizontal, fieldName + ".maxAngleFromHorizontal", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.downsampleResolution, fieldName + ".downsampleResolution", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.maxRange, fieldName + ".maxRange", verbose);

  loadData::loadPtreeValue(pt, fitterSettings.removeUnreachable, fieldName + ".rejectUnreachablePoints.active", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.maximumDistance, fieldName + ".rejectUnreachablePoints.maxDistance", verbose);
//  loadData::loadStdVector(filename, fieldName + ".rejectUnreachablePoints.referencePoint", fitterSettings.reachableReference);
  loadData::loadEigenMatrix(filename, fieldName + ".rejectUnreachablePoints.referencePoint", fitterSettings.reachableReference);

  loadData::loadPtreeValue(pt, fitterSettings.minGroundZ, fieldName + ".minGroundZ", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.maxGroundZ, fieldName + ".maxGroundZ", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.minHeightAboveGround, fieldName + ".minHeightAboveGround", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.maxHeightAboveGround, fieldName + ".maxHeightAboveGround", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.areaThreshMin, fieldName + ".areaThreshMin", verbose);
  loadData::loadPtreeValue(pt, fitterSettings.areaThreshMax, fieldName + ".areaThreshMax", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return fitterSettings;
}
}

#endif
