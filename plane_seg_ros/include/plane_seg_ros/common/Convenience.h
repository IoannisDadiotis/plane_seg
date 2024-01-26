#ifndef CONVENIENCE_H
#define CONVENIENCE_H

// convenience methods
inline auto vecToStr(const Eigen::Vector3f& iVec) {
  std::ostringstream oss;
  oss << iVec[0] << ", " << iVec[1] << ", " << iVec[2];
  return oss.str();
}

inline auto rotToStr(const Eigen::Matrix3f& iRot) {
  std::ostringstream oss;
  Eigen::Quaternionf q(iRot);
  oss << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z();
  return oss.str();
}

inline void quat_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2.0*(q0*q1+q2*q3), 1.0-2.0*(q1*q1+q2*q2));
  pitch = asin(2.0*(q0*q2-q3*q1));
  yaw = atan2(2.0*(q0*q3+q1*q2), 1.0-2.0*(q2*q2+q3*q3));
}

#endif // CONVENIENCE_H
