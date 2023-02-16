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

#ifndef COORDINATE_TRANSFORMATION_COORDINATE_TRANSFORMATION_H
#define COORDINATE_TRANSFORMATION_COORDINATE_TRANSFORMATION_H

#include <radar_interface/interface_datatypes/location.h>

#include <vfc/core/vfc_trig.hpp>
#include <vfc/geo/vfc_matvec2_ops.hpp>
#include <vfc/geo/vfc_matvec3_ops.hpp>
#include <vfc/geo/vfc_matvec4_ops.hpp>
#include <vfc/geo/vfc_vec2.hpp>


namespace radar {
namespace coordinate_transformation {

template <typename ValueT>
vfc::TMatrix3<ValueT> computeTransformationMatrix(vfc::CRadian rotation,
                                                  vfc::TVector2<ValueT> translation);


template <typename ValueT>
vfc::TMatrix3<ValueT> computeTransformationMatrix(vfc::CDegree rotation,
                                                  vfc::TVector2<ValueT> translation);


template <typename ValueT>
vfc::TMatrix4<ValueT> computeTransformationMatrix(vfc::CRadian roll,
                                                  vfc::CRadian pitch,
                                                  vfc::CRadian yaw,
                                                  vfc::TVector3<ValueT> position);


template <typename ValueT>
vfc::TMatrix4<ValueT> computeTransformationMatrix(vfc::CDegree roll,
                                                  vfc::CDegree pitch,
                                                  vfc::CDegree yaw,
                                                  vfc::TVector3<ValueT> position);


template <typename ValueT = vfc::float64_t>
vfc::TMatrix2<ValueT> createRotationMatrix(vfc::CRadian rotation);
template <typename ValueT = vfc::float64_t>
vfc::TMatrix2<ValueT> createRotationMatrix(vfc::CDegree rotation);


namespace with_covariance {

/// @brief Converts a point given in conical coordinates \f$ (r, \alpha, \beta) \f$ to cartesian
/// coordinates \f$ (x, y z) \f$. Variances given for \f$ (r, \alpha, \beta) \f$ will result in the
/// corresponding covariances in \f$ (x, y z) \f$.
/// @remark For \f$ \beta = 0 \f$ this function is equivalent to the conversion from polar to
/// cartesian coordinates.
/// @param[in] radius Distance from the origin to the point
/// @param[in] alpha Azimuth angle spanning a cone around the y-axis
/// @param[in] beta Elevation angle spanning a cone around the z-axis
/// @returns Cartesian coordinates of the converted point together with a 3x3 covariance matrix
/// @see For further information see [Cone Coordinate System for
/// Locations](https://athena.daimler.com/confluence/x/mk5UB)
inline aracom_interface_radar::Feature3D conicalToCartesianPosition(
    const aracom_interface_radar::Feature& radius,
    const aracom_interface_radar::Feature& alpha,
    const aracom_interface_radar::Feature& beta);

}  // namespace with_covariance

}  // namespace coordinate_transformation
}  // namespace radar

#include "coordinate_transformation.inl"
#endif  // COORDINATE_TRANSFORMATION_COORDINATE_TRANSFORMATION_H
