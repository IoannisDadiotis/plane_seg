#ifndef ROSMSGCONVERSIONS_H
#define ROSMSGCONVERSIONS_H

#include <plane_seg_ros/convex_hull.h>
#include <plane_seg/BlockFitter.hpp>
#include <tf2_eigen/tf2_eigen.h>

inline plane_seg_ros::convex_hull createConvexHullMsg(const planeseg::BlockFitter::Block& block) {
    plane_seg_ros::convex_hull hullMsg;
    auto& hullPoints = block.mHull;
    hullMsg.points.reserve(hullPoints.size());
    for (auto& p: hullPoints) {
        geometry_msgs::Point pointMsg;
        pointMsg.x = p[0];
        pointMsg.y = p[1];
        pointMsg.z = p[2];
        hullMsg.points.push_back(pointMsg);
    }
    hullMsg.pose = tf2::toMsg(block.mPose.cast<double>());
    hullMsg.size.x = block.mSize[0];
    hullMsg.size.y = block.mSize[1];
    hullMsg.size.z = block.mSize[2];
    return hullMsg;
}
#endif // ROSMSGCONVERSIONS_H
