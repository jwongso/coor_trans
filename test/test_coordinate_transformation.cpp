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

#include "coordinate_transformation/coordinate_transformation.h"

#include <gtest/gtest.h>
#include <math.h>
#include <vfc/core/vfc_trig.hpp>

namespace radar {
namespace test_coordinate_transformation {

// TODO Check conical coordinates typical, polar coordinates (beta = 0) typical, implausible inputs
// (variances)

// ===== Test Fixture for Test Suite definition =====

/// @brief Test fixture for normal usage of
/// radar::coordinate_transformation::with_covariance::conicalToCartesianPosition method
class RadarCoordinateTransformation_conicalToCartesianPosition_NormalTest : public ::testing::Test {
 protected:
  // TODO Check requirements!!! Account for float calculation tolerance to accept result?
  double allowedToleranceCartesian = 0.001;  // 1 mm
};


///@addtogroup conicalToCartesianPosition
///@{
//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test (Check again when test requirements are changed)
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] The conicalToCartesianPosition method shall return correct
/// cartesian representation (vector, w/o covariance), when typical conical input is provided
/// .
TEST_F(RadarCoordinateTransformation_conicalToCartesianPosition_NormalTest,
       shallTransformToCartesianCoordinatesWhenTypicalConicalInput) {
  ///@par Teststep 1: Generate test data.
  aracom_interface_radar::Feature3D expected_cartesian;
  const aracom_interface_radar::Feature kInputRadius = {20.0, 0.5};
  const aracom_interface_radar::Feature kInputAlpha = {0.35, 0.0005};
  const aracom_interface_radar::Feature kInputBeta = {0.02, 0.0001};

  // Setup expected result (based on code calculation)
  expected_cartesian.value(0) = 18.783196181403557;   // x
  expected_cartesian.value(1) = 6.8579561491090271;   // y
  expected_cartesian.value(2) = 0.39997333386666162;  // z


  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const aracom_interface_radar::Feature3D kResultCartesian =
      radar::coordinate_transformation::with_covariance::conicalToCartesianPosition(
          kInputRadius, kInputAlpha, kInputBeta);
  EXPECT_NEAR(kResultCartesian.value(0), expected_cartesian.value(0),
              allowedToleranceCartesian);  // x
  EXPECT_NEAR(kResultCartesian.value(1), expected_cartesian.value(1),
              allowedToleranceCartesian);  // y
  EXPECT_NEAR(kResultCartesian.value(2), expected_cartesian.value(2),
              allowedToleranceCartesian);  // z
}

//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test (Check again when test requirements are changed)
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] The conicalToCartesianPosition method shall return correct
/// cartesian representation (vector, w/o covariance), when typical polar input is provided
/// .
TEST_F(RadarCoordinateTransformation_conicalToCartesianPosition_NormalTest,
       shallTransformToCartesianCoordinatesWhenTypicalPolarInput) {
  ///@par Teststep 1: Generate test data.
  aracom_interface_radar::Feature3D expected_cartesian;
  const aracom_interface_radar::Feature kInputRadius = {20.0, 0.5};
  const aracom_interface_radar::Feature kInputAlpha = {0.35, 0.0005};
  const aracom_interface_radar::Feature kInputBeta = {0.0, 0.0};

  // Setup expected result (based on code calculation)
  expected_cartesian.value(0) = 18.787454256947576;  // x
  expected_cartesian.value(1) = 6.8579561491090271;  // y
  expected_cartesian.value(2) = 0.0;                 // z

  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const aracom_interface_radar::Feature3D kResultCartesian =
      radar::coordinate_transformation::with_covariance::conicalToCartesianPosition(
          kInputRadius, kInputAlpha, kInputBeta);
  EXPECT_NEAR(kResultCartesian.value(0), expected_cartesian.value(0),
              allowedToleranceCartesian);  // x
  EXPECT_NEAR(kResultCartesian.value(1), expected_cartesian.value(1),
              allowedToleranceCartesian);  // y
  EXPECT_NEAR(kResultCartesian.value(2), expected_cartesian.value(2),
              allowedToleranceCartesian);  // z
}


//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test (Check again when test requirements are changed)
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] The conicalToCartesianPosition method shall return correct
/// cartesian representation (vector, w/o covariance), when typical conical input with negative
/// variances is provided (value vector is not affected by variances, variances are not checked)
/// .
TEST_F(RadarCoordinateTransformation_conicalToCartesianPosition_NormalTest,
       shallTransformToCartesianCoordinatesWhenTypicalConicalInputWithNegativeVariances) {
  ///@par Teststep 1: Generate test data.
  aracom_interface_radar::Feature3D expected_cartesian;
  const aracom_interface_radar::Feature kInputRadius = {20.0, -0.5};
  const aracom_interface_radar::Feature kInputAlpha = {0.35, -0.0005};
  const aracom_interface_radar::Feature kInputBeta = {0.02, -0.0001};

  // Setup expected result (based on code calculation)
  expected_cartesian.value(0) = 18.783196181403557;   // x
  expected_cartesian.value(1) = 6.8579561491090271;   // y
  expected_cartesian.value(2) = 0.39997333386666162;  // z

  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const aracom_interface_radar::Feature3D kResultCartesian =
      radar::coordinate_transformation::with_covariance::conicalToCartesianPosition(
          kInputRadius, kInputAlpha, kInputBeta);
  EXPECT_NEAR(kResultCartesian.value(0), expected_cartesian.value(0),
              allowedToleranceCartesian);  // x
  EXPECT_NEAR(kResultCartesian.value(1), expected_cartesian.value(1),
              allowedToleranceCartesian);  // y
  EXPECT_NEAR(kResultCartesian.value(2), expected_cartesian.value(2),
              allowedToleranceCartesian);  // z
}

//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test (Check again when test requirements are changed)
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] The conicalToCartesianPosition method shall return correct
/// cartesian representation (vector, w/o covariance), when typical polar input with negative
/// variances is provided (value vector is not affected by variances, variances are not checked)
/// .
TEST_F(RadarCoordinateTransformation_conicalToCartesianPosition_NormalTest,
       shallTransformToCartesianCoordinatesWhenTypicalPolarInputWithNegativeVariances) {
  ///@par Teststep 1: Generate test data.
  aracom_interface_radar::Feature3D expected_cartesian;
  const aracom_interface_radar::Feature kInputRadius = {20.0, -0.5};
  const aracom_interface_radar::Feature kInputAlpha = {0.35, -0.0005};
  const aracom_interface_radar::Feature kInputBeta = {0.0, 0.0};

  // Setup expected result (based on code calculation)
  expected_cartesian.value(0) = 18.787454256947576;  // x
  expected_cartesian.value(1) = 6.8579561491090271;  // y
  expected_cartesian.value(2) = 0.0;                 // z

  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const aracom_interface_radar::Feature3D kResultCartesian =
      radar::coordinate_transformation::with_covariance::conicalToCartesianPosition(
          kInputRadius, kInputAlpha, kInputBeta);
  EXPECT_NEAR(kResultCartesian.value(0), expected_cartesian.value(0),
              allowedToleranceCartesian);  // x
  EXPECT_NEAR(kResultCartesian.value(1), expected_cartesian.value(1),
              allowedToleranceCartesian);  // y
  EXPECT_NEAR(kResultCartesian.value(2), expected_cartesian.value(2),
              allowedToleranceCartesian);  // z
}
///@} //end of doxygen test group conicalToCartesianPosition

///@addtogroup computeTransformationMatrix
///@{
//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test (Check again when test requirements are changed)
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] Shall calculate the transformation matrix according to Matrix
/// Transformation Formula 3.35 shown in
/// the link: https://athena.daimler.com/confluence/display/ATROB/Preprocessing+formulas
/// .
TEST(computeTransformationMatrix, shallCalculateTransformationMatrixWhen2DTransformationInRadian) {
  ///@par Teststep 1: Generate test data.

  const vfc::CRadian kRotation = vfc::CRadian(1.);
  const vfc::TVector2<vfc::float64_t> kTranslation = {2., 3.};

  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const vfc::TMatrix3<double> kResult =
      radar::coordinate_transformation::computeTransformationMatrix(kRotation, kTranslation);

  EXPECT_EQ(kResult(0, 0), cos(1));
  EXPECT_EQ(kResult(0, 1), -sin(1));
  EXPECT_EQ(kResult(0, 2), kTranslation[0]);

  EXPECT_EQ(kResult(1, 0), sin(1));
  EXPECT_EQ(kResult(1, 1), cos(1));
  EXPECT_EQ(kResult(1, 2), kTranslation[1]);

  EXPECT_EQ(kResult(2, 0), 0);
  EXPECT_EQ(kResult(2, 1), 0);
  EXPECT_EQ(kResult(2, 2), 1);
}

//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test (Check again when test requirements are changed)
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] Shall calculate the transformation matrix according to Matrix
/// Transformation Formula 3.35 shown in
/// the link: https://athena.daimler.com/confluence/display/ATROB/Preprocessing+formulas
/// .
TEST(computeTransformationMatrix, shallCalculateTransformationMatrixWhen2DTransformationInDegree) {
  ///@par Teststep 1: Generate test data.
  const double kAllowedToleranceConversion = 0.00001;
  const vfc::CDegree kRotation = vfc::CDegree(57.295779513);
  const vfc::TVector2<vfc::float64_t> kTranslation = {2., 3.};

  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const vfc::TMatrix3<double> kResult =
      radar::coordinate_transformation::computeTransformationMatrix(kRotation, kTranslation);

  EXPECT_NEAR(kResult(0, 0), cos(1), kAllowedToleranceConversion);
  EXPECT_NEAR(kResult(0, 1), -sin(1), kAllowedToleranceConversion);
  EXPECT_NEAR(kResult(0, 2), kTranslation[0], kAllowedToleranceConversion);

  EXPECT_NEAR(kResult(1, 0), sin(1), kAllowedToleranceConversion);
  EXPECT_NEAR(kResult(1, 1), cos(1), kAllowedToleranceConversion);
  EXPECT_NEAR(kResult(1, 2), kTranslation[1], kAllowedToleranceConversion);

  EXPECT_NEAR(kResult(2, 0), 0, kAllowedToleranceConversion);
  EXPECT_NEAR(kResult(2, 1), 0, kAllowedToleranceConversion);
  EXPECT_NEAR(kResult(2, 2), 1, kAllowedToleranceConversion);
}

//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] Shall calculate the transformation matrix according to Matrix
/// Transformation Formula 3.50 shown in
/// the link: https://athena.daimler.com/confluence/display/ATROB/Preprocessing+formulas
/// .
TEST(computeTransformationMatrix, shallCalculateTransformationMatrixWhen3DTransformationInRadian) {
  ///@par Teststep 1: Generate test data.
  const double kAllowedToleranceTransformation = 0.00001;
  const vfc::CRadian kRoll = vfc::CRadian(1.4);
  const vfc::CRadian kPitch = vfc::CRadian(1.2);
  const vfc::CRadian kYaw = vfc::CRadian(1.);

  const vfc::TVector3<vfc::float64_t> kPosition = {2, 4, 6};

  vfc::TMatrix4<double> expected_matrix;
  expected_matrix(0, 0) = 0.19578273029;
  expected_matrix(0, 1) = 0.353233181493273;
  expected_matrix(0, 2) = 0.9148198959413531;
  expected_matrix(0, 3) = 2;

  expected_matrix(1, 0) = 0.3049135365125767;
  expected_matrix(1, 1) = 0.864705945030989;
  expected_matrix(1, 2) = -0.3991382766391277;
  expected_matrix(1, 3) = 4;

  expected_matrix(2, 0) = -0.932039085967;
  expected_matrix(2, 1) = 0.3570853513084176;
  expected_matrix(2, 2) = 0.0615889122361154;
  expected_matrix(2, 3) = 6;

  expected_matrix(3, 0) = 0;
  expected_matrix(3, 1) = 0;
  expected_matrix(3, 2) = 0;
  expected_matrix(3, 3) = 1;

  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const auto kResult =
      radar::coordinate_transformation::computeTransformationMatrix(kRoll, kPitch, kYaw, kPosition);

  for (int32_t i = 0; i < 4; i++) {
    for (int32_t j = 0; j < 4; j++) {
      EXPECT_NEAR(kResult(i, j), expected_matrix(i, j), kAllowedToleranceTransformation)
          << "Matrix Position: (i=" << i << " , j=" << j << ")";
    }
  }
}

//////////////////////////////////////////////////////////////////
/// @test Test method: Requirement Based Test
///
///@sa detailed design: TODO
///@sa tested requirement: [TEMPORARY] Shall calculate the transformation matrix according to Matrix
/// Transformation Formula 3.50 shown in
/// the link: https://athena.daimler.com/confluence/display/ATROB/Preprocessing+formulas
/// .
TEST(computeTransformationMatrix, shallCalculateTransformationMatrixWhen3DTransformationInDegree) {
  ///@par Teststep 1: Generate test data.
  const double kAllowedToleranceTransformation = 0.00001;
  const vfc::CDegree kRoll = vfc::CDegree(vfc::CRadian(1.4));
  const vfc::CDegree kPitch = vfc::CDegree(vfc::CRadian(1.2));
  const vfc::CDegree kYaw = vfc::CDegree(vfc::CRadian(1.));

  const vfc::TVector3<vfc::float64_t> kPosition = {2, 4, 6};

  vfc::TMatrix4<double> expected_matrix;
  expected_matrix(0, 0) = 0.19578273029;
  expected_matrix(0, 1) = 0.353233181493273;
  expected_matrix(0, 2) = 0.9148198959413531;
  expected_matrix(0, 3) = 2;

  expected_matrix(1, 0) = 0.3049135365125767;
  expected_matrix(1, 1) = 0.864705945030989;
  expected_matrix(1, 2) = -0.3991382766391277;
  expected_matrix(1, 3) = 4;

  expected_matrix(2, 0) = -0.932039085967;
  expected_matrix(2, 1) = 0.3570853513084176;
  expected_matrix(2, 2) = 0.0615889122361154;
  expected_matrix(2, 3) = 6;

  expected_matrix(3, 0) = 0;
  expected_matrix(3, 1) = 0;
  expected_matrix(3, 2) = 0;
  expected_matrix(3, 3) = 1;

  ///@par Teststep 2: Call function under test and check expected results
  // Call function under test
  const auto kResult =
      radar::coordinate_transformation::computeTransformationMatrix(kRoll, kPitch, kYaw, kPosition);

  for (int32_t i = 0; i < 4; i++) {
    for (int32_t j = 0; j < 4; j++) {
      EXPECT_NEAR(kResult(i, j), expected_matrix(i, j), kAllowedToleranceTransformation)
          << "Matrix Position: (i=" << i << " , j=" << j << ")";
    }
  }
}

///@} //end of doxygen test group computeTransformationMatrix

}  // namespace test_coordinate_transformation
}  // namespace radar
