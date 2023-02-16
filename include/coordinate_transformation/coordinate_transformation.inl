/********************************************************************
 * Copyrght: jsaputra@riseup.net
 *
 * The reproduction, distribution and utilization of this file as
 * well as the communication of its contents to others without express
 * authorization is prohibited. Offenders will be held liable for the
 * payment of damages and can be prosecuted. All rights reserved
 * particularly in the event of the grant of a patent, utility model
 * or design.
 *
 ********************************************************************/

#include <vfc/geo/vfc_mat4_ops.hpp>

namespace radar {
namespace coordinate_transformation {

template <typename T>
vfc::TMatrix3<T> computeTransformationMatrix(vfc::CRadian rotation, vfc::TVector2<T> translation) {
  //    The homogeneous transformation matrix for 3D bodies
  //  Planning Algorithms
  //  Chapter 3
  //  Geometric Representations and
  //  Transformations
  //  Steven M. LaValle
  //  University of Illinois
  //  Copyright Steven M. LaValle 2006
  //  Available for downloading at http://planning.cs.uiuc.edu/

  // see formula (3.35)

  const T cos_phi{cos(rotation)};
  const T sin_phi{sin(rotation)};

  const T m00{cos_phi};
  const T m10{sin_phi};
  const T m20{0};

  const T m01{-sin_phi};
  const T m11{cos_phi};
  const T m21{0};

  const T m02{translation.x()};
  const T m12{translation.y()};
  const T m22{1};

  return vfc::TMatrix3<T>(m00, m01, m02, m10, m11, m12, m20, m21, m22);
}

template <typename T>
vfc::TMatrix3<T> computeTransformationMatrix(vfc::CDegree rotation, vfc::TVector2<T> translation) {
  return computeTransformationMatrix(vfc::CRadian(rotation), translation);
}

template <typename T>
vfc::TMatrix4<T> computeTransformationMatrix(vfc::CRadian roll,
                                             vfc::CRadian pitch,
                                             vfc::CRadian yaw,
                                             vfc::TVector3<T> position) {
  //    The homogeneous transformation matrix for 3D bodies
  //  Planning Algorithms
  //  Chapter 3
  //  Geometric Representations and
  //  Transformations
  //  Steven M. LaValle
  //  University of Illinois
  //  Copyright Steven M. LaValle 2006
  //  Available for downloading at http://planning.cs.uiuc.edu/

  // see formula (3.50)

  const T cos_r{cos(roll)};
  const T cos_p{cos(pitch)};
  const T cos_y{cos(yaw)};

  const T sin_r{sin(roll)};
  const T sin_p{sin(pitch)};
  const T sin_y{sin(yaw)};

  const T m00{cos_p * cos_y};
  const T m10{cos_p * sin_y};
  const T m20{-sin_p};
  const T m30{0};

  const T m01{sin_r * sin_p * cos_y - cos_r * sin_y};
  const T m11{sin_r * sin_p * sin_y + cos_r * cos_y};
  const T m21{sin_r * cos_p};
  const T m31{0};

  const T m02{cos_r * sin_p * cos_y + sin_r * sin_y};
  const T m12{cos_r * sin_p * sin_y - sin_r * cos_y};
  const T m22{cos_r * cos_p};
  const T m32{0};

  const T m03{position.x()};
  const T m13{position.y()};
  const T m23{position.z()};
  const T m33{1};

  return vfc::TMatrix4<T>(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32,
                          m33);
}

template <typename T>
vfc::TMatrix4<T> computeTransformationMatrix(vfc::CDegree roll,
                                             vfc::CDegree pitch,
                                             vfc::CDegree yaw,
                                             vfc::TVector3<T> position) {
  return computeTransformationMatrix(vfc::CRadian(roll), vfc::CRadian(pitch), vfc::CRadian(yaw),
                                     position);
}

template <typename T>
vfc::TMatrix2<T> createRotationMatrix(vfc::CRadian rotation) {
  const T cos_phi{cos(rotation)};
  const T sin_phi{sin(rotation)};

  const T m00{cos_phi};
  const T m10{sin_phi};

  const T m01{-sin_phi};
  const T m11{cos_phi};

  return vfc::TMatrix2<T>(m00, m01, m10, m11);
}

template <typename T>
vfc::TMatrix2<T> createRotationMatrix(vfc::CDegree rotation) {
  return createRotationMatrix(vfc::CRadian(rotation));
}


namespace with_covariance {

inline aracom_interface_radar::Feature3D conicalToCartesianPosition(
    const aracom_interface_radar::Feature& radius,
    const aracom_interface_radar::Feature& alpha,
    const aracom_interface_radar::Feature& beta) {
  // assumes location representation in cone coordinates and transforms to cartesian coordinates
  // [x, y, z] = [r*(1-sin(alpha)^2-sin(beta)^2)^(1/2), r*sin(alpha), r*sin(beta)];

  aracom_interface_radar::Feature3D position;
  const vfc::float64_t& r = radius.value;

  // common elements
  const vfc::float64_t sa = sin(alpha.value);
  const vfc::float64_t ca = cos(alpha.value);
  const vfc::float64_t sb = sin(beta.value);
  const vfc::float64_t cb = cos(beta.value);

  const vfc::float64_t rxProj = sqrt(1 - sa * sa - sb * sb);

  // cartesian position
  const vfc::float64_t x = r * rxProj;
  const vfc::float64_t y = r * sa;
  const vfc::float64_t z = r * sb;
  position.value(0) = x;
  position.value(1) = y;
  position.value(2) = z;

  // Jacobian
  // J =[ dxdr, dxdalpha, dxdbeta;
  //      dydr, dydalpha, dydbeta;
  //      dzdr, dzdalpha, dzdbeta]
  //   =[ sqrt(-sin(beta)^2-sin(alpha)^2+1),
  //   -(1.0*cos(alpha)*sin(alpha)*r)/(-sin(beta)^2-sin(alpha)^2+1)^0.5,
  //   -(1.0*cos(beta)*sin(beta)*r)/(-sin(beta)^2-sin(alpha)^2+1)^0.5;
  //      sin(alpha),                        cos(alpha)*r, 0; sin(beta), 0,
  //      cos(beta)*r];
  Eigen::Matrix3d J;
  J << rxProj, -ca * sa * r / rxProj, -cb * sb * r / rxProj, sa, ca * r, 0, sb, 0, cb * r;

  // cone coordinate covariance
  Eigen::Matrix3d coneCov;
  coneCov << radius.variance, 0, 0, 0, alpha.variance, 0, 0, 0, beta.variance;

  // cartesian covariance (using linearization with Jacobi matrix (error propagation), valid for
  // small angular uncertainty)
  position.covariance = J * coneCov * (J.transpose());
  return position;
}

}  // namespace with_covariance
}  // namespace coordinate_transformation
}  // namespace radar
