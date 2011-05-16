/*!
 * @file pvat_cartesian.hpp
 * @brief Calculations related to the cartesian trajectory generation.
 *
 * Most of the equations are implemented due to the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system" report.
 *
 * @author tkornuta
 *
 * @ingroup LIB
 */
#ifndef PVAT__CARTESIAN_HPP_
#define PVAT__CARTESIAN_HPP_

#include <cmath>
#include <iostream>
#include <cassert>
//#include <vector>

#include <Eigen/Core>
#include <Eigen/Array>

#include "base/lib/mrmath/Xyz_Angle_Axis_Gamma_vector.h"
#include "base/kinematics/kinematic_model.h"
#include "base/edp/edp_exceptions.h"

namespace mrrocpp {
namespace lib {
namespace pvat {

using namespace std;

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

//! Name of violated constraint - maximum.
const std::string MAXIMUM_CONSTRAINT = "Maximum";

//! Name of violated constraint - minimum.
const std::string MINIMUM_CONSTRAINT = "Minimum";

//! Type of violated constraint.
typedef boost::error_info <struct constraint_type_, std::string> constraint_type;

//! Number of motor that caused the exception.
typedef boost::error_info <struct motor_number_, int> motor_number;

//! Segment in which a constraint was exceeded.
typedef boost::error_info <struct segment_number_, int> segment_number;

//! Desired value that caused the exception.
typedef boost::error_info <struct desired_value_, double> desired_value;

/*!
 * \brief Exception thrown when motor velocity constraint exceeded.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motor_velocity_constraint_exceeded, "Motor velocity constraint exceeded")

/*!
 * \brief Exception thrown when motor velocity constraint exceeded.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motor_acceleration_constraint_exceeded, "Motor acceleration constraint exceeded")





/**
 * @brief Creates a vector containing time slices.
 *
 * This method considers only constant (the same for every segment) time slices.
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS Number of motion segments.
 *
 * @param [out] time_deltas_ Computed time slices (one for each segment).
 * @param [in] motion_time_ Total motion time.
 */
template <unsigned int N_SEGMENTS>
void divide_motion_time_into_constant_time_deltas(Eigen::Matrix <double, N_SEGMENTS, 1> & time_deltas_, const double motion_time_)
{
	// There must be some segments (besides we cannot divide by zero).
	assert (N_SEGMENTS!=0);
	double segment_time = motion_time_ / N_SEGMENTS;

	for (int i = 0; i < N_SEGMENTS; ++i) {
		time_deltas_(i) = segment_time;
	}

	// Display results.
/*	cout<<"time deltas: ";
	for (int i = 0; i < N_SEGMENTS; ++i) {
		cout << " " << time_deltas_(i);
	}
	cout << endl;*/
}


/**
 * @brief Computes interpolation motor positions for the cartesian trajectory generation - translation between segments is linear.
 *
 *
 * Method consideres also the different motion time for different segments!
 * Number of interpolated points is equal to number of segments (thus time slices) + 1 (the starting point).
 *
 * @author tkornuta
 *
 * @tparam N_POINTS Number of interpolation points.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] motor_interpolations_ Matrix containing interpolated motor poses.
 * @param [in] motion_time_ Total motion time.
 * @param [in] time_deltas_ Times of motion for one segment (may be different for each segment!).
 * @param [in] model_ Kinematic model required for inverse kinematics computations.
 * @param [in] desired_joints_old_ Desired joint values that were required by previously received SET command (threated as current position of joints).
 * @param [in] current_end_effector_frame_ Homogeneous matrix containing current end effector pose.
 * @param [in] desired_end_effector_frame_ Homogeneous matrix containing desired end effector pose.
 */
template <unsigned int N_POINTS, unsigned int N_MOTORS>
void linear_interpolate_motor_poses(Eigen::Matrix <double, N_POINTS, N_MOTORS> & motor_interpolations_, const double motion_time_, const Eigen::Matrix <
		double, N_POINTS - 1, 1> time_deltas_, mrrocpp::kinematics::common::kinematic_model* model_, const lib::JointArray desired_joints_old_, const mrrocpp::lib::Homog_matrix& current_end_effector_frame_, const mrrocpp::lib::Homog_matrix& desired_end_effector_frame_)
{
	// Manipulator has got to have some axes.
	assert (N_MOTORS>0);
	// There must be some segments (besides we cannot divide by zero).
	assert (N_POINTS>1);
	// Check model.
	assert (model_);

	//	cout<<"Operational space: " << model->get_kinematic_model_label() << "\n";

	// Variable containing computed transformation from current end-effector post to the desired one.
	lib::Homog_matrix desired_relative_end_effector_frame;

	// Compute transformation from current to desired pose.
	desired_relative_end_effector_frame = !current_end_effector_frame_ * desired_end_effector_frame_;

	cout << desired_relative_end_effector_frame << endl;

	// Extract translation and rotation (second one in the form of angle, axis and gamma).
	Xyz_Angle_Axis_Gamma_vector relative_xyz_aa_gamma;
	desired_relative_end_effector_frame.get_xyz_angle_axis_gamma(relative_xyz_aa_gamma);

	// Compute current P, v and gamma.
	Xyz_Angle_Axis_Gamma_vector current_xyz_aa_gamma;
	current_end_effector_frame_.get_xyz_angle_axis_gamma(current_xyz_aa_gamma);

	// Delta variables.
	Xyz_Angle_Axis_Gamma_vector delta_xyz_aa_gamma;
	lib::Homog_matrix delta_ee_frame;
	// Interpolation variables.
	lib::Homog_matrix int_ee_frame;
	lib::JointArray int_joints(N_MOTORS);
	lib::MotorArray int_motors(N_MOTORS);
	// Set last joint settings.
	lib::JointArray int_joints_old = desired_joints_old_;

	// Add current position as first one, thus there will be n+1 interpolation points.
	// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
	model_->inverse_kinematics_transform(int_joints, desired_joints_old_, current_end_effector_frame_);
	// Transform joints to motors (and check motors/joints values).
	model_->i2mp_transform(int_motors, int_joints);
	// Add motors to vector - first interpolation point.
	motor_interpolations_.row(0) = int_motors.transpose();

	// Temporary variables containing motion time from start to current position (sum of time slices).
	double last_summed = time_deltas_(0);
	double total_time_factor = last_summed / motion_time_;

	// Compute interpolation points in motor positions.
	for (int i = 0; i < N_POINTS - 1; ++i) {
		// Compute delta in the angle axis gamma representation.
		delta_xyz_aa_gamma
		// Px, Py, Pz.
				<< relative_xyz_aa_gamma(0) * total_time_factor, relative_xyz_aa_gamma(1) * total_time_factor, relative_xyz_aa_gamma(2)
				* total_time_factor,
		// vx, vy, vz (constant).
		relative_xyz_aa_gamma(3), relative_xyz_aa_gamma(4), relative_xyz_aa_gamma(5),
		// Gamma.
		relative_xyz_aa_gamma(6) * total_time_factor;

		// Compute delta frame.
		delta_ee_frame.set_from_xyz_angle_axis_gamma(delta_xyz_aa_gamma);

		// Compute desired interpolation end effector frame.
		int_ee_frame = current_end_effector_frame_ * delta_ee_frame;

		//		cout << int_ee_frame << endl;

		// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
		model_->inverse_kinematics_transform(int_joints, int_joints_old, int_ee_frame);

		//		cout << int_joints << endl;

		// Transform joints to motors (and check motors/joints values).
		model_->i2mp_transform(int_motors, int_joints);

		//		cout << int_motors << endl;

		// Add motors to vector.
		motor_interpolations_.row(i + 1) = int_motors.transpose();

		// Set last joint settings.
		int_joints_old = int_joints;
		// Add time slice related to this segment.
		if (i < N_POINTS - 2) {
			last_summed += time_deltas_(i + 1);
			total_time_factor = last_summed / motion_time_;
		}
	}

	// Display all motor interpolation poses.
/*	for (unsigned int l = 0; l < N_POINTS; ++l) {
		cout << "Motor interpolation point no " << l << ": " << motor_interpolations_.row(l) << endl;
	}*/
}


/**
 * @brief Computes interpolation motor positions for the cartesian trajectory generation -
 * a parabolic velocity profile is obtained, thus translation between segments is cubic.
 *
 * The interpolation points are coputed due to the (1.10-1.17) formula from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * Method consideres also the different motion time for different segments!
 * Number of interpolated points is equal to number of segments (thus time slices) + 1 (the starting point).
 *
 * @author tkornuta
 *
 * @tparam N_POINTS Number of interpolation points.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] motor_interpolations_ Matrix containing interpolated motor poses.
 * @param [in] motion_time_ Total motion time.
 * @param [in] time_deltas_ Times of motion for one segment (may be different for each segment!).
 * @param [in] model_ Kinematic model required for inverse kinematics computations.
 * @param [in] desired_joints_old_ Desired joint values that were required by previously received SET command (threated as current position of joints).
 * @param [in] current_end_effector_frame_ Homogeneous matrix containing current end effector pose.
 * @param [in] desired_end_effector_frame_ Homogeneous matrix containing desired end effector pose.
 */
template <unsigned int N_POINTS, unsigned int N_MOTORS>
void cubic_polynomial_interpolate_motor_poses(Eigen::Matrix <double, N_POINTS, N_MOTORS> & motor_interpolations_, const double motion_time_, const Eigen::Matrix <
		double, N_POINTS - 1, 1> time_deltas_, mrrocpp::kinematics::common::kinematic_model* model_, const lib::JointArray desired_joints_old_, const mrrocpp::lib::Homog_matrix& current_end_effector_frame_, const mrrocpp::lib::Homog_matrix& desired_end_effector_frame_)
{
	// Manipulator has got to have some axes.
	assert (N_MOTORS>0);
	// There must be some segments (besides we cannot divide by zero).
	assert (N_POINTS>1);
	// Check model.
	assert (model_);

	//	cout<<"Operational space: " << model->get_kinematic_model_label() << "\n";

	// Variable containing computed transformation from current end-effector post to the desired one.
	lib::Homog_matrix desired_relative_end_effector_frame;

	// Compute transformation from current to desired pose.
	desired_relative_end_effector_frame = !current_end_effector_frame_ * desired_end_effector_frame_;

//	cout << "relative ee frame" << desired_relative_end_effector_frame << endl;

	// Extract translation and rotation (second one in the form of angle, axis and gamma).
	Xyz_Angle_Axis_Gamma_vector relative_xyz_aa_gamma;
	desired_relative_end_effector_frame.get_xyz_angle_axis_gamma(relative_xyz_aa_gamma);

//	cout << "relative xyz aa gamma" << relative_xyz_aa_gamma << endl;

	// Delta variables.
	Xyz_Angle_Axis_Gamma_vector delta_xyz_aa_gamma;
	lib::Homog_matrix delta_ee_frame;
	// Interpolation variables.
	lib::Homog_matrix int_ee_frame;
	lib::JointArray int_joints(N_MOTORS);
	lib::MotorArray int_motors(N_MOTORS);
	// Set last joint settings.
	lib::JointArray int_joints_old = desired_joints_old_;

	// Add current position as first one, thus there will be n+1 interpolation points.
	// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
	model_->inverse_kinematics_transform(int_joints, desired_joints_old_, current_end_effector_frame_);
	// Transform joints to motors (and check motors/joints values).
	model_->i2mp_transform(int_motors, int_joints);
	// Add motors to vector - first interpolation point.
	motor_interpolations_.row(0) = int_motors.transpose();

	// Temporary variables containing motion time from start to current position (sum of time slices).
	double last_summed = time_deltas_(0);
	// Compute polynomial coefficients - the (1.16) formula.
	Xyz_Angle_Axis_Gamma_vector w3 = - (2.0 * relative_xyz_aa_gamma) / (motion_time_ * motion_time_ * motion_time_);
	Xyz_Angle_Axis_Gamma_vector w2 = (3.0 * relative_xyz_aa_gamma) / (motion_time_ * motion_time_);

//	cout << "w3 "<< w3 << endl;
//	cout << "w2 "<< w2 << endl;

	// Compute interpolation points in motor positions.
	for (int i = 0; i < N_POINTS - 1; ++i) {
		// Compute delta in the angle axis gamma representation.
		delta_xyz_aa_gamma
		// Px, Py, Pz.
				<< w3(0) * last_summed * last_summed * last_summed + w2(0) * last_summed * last_summed,
				w3(1) * last_summed * last_summed * last_summed + w2(1) * last_summed * last_summed,
				w3(2) * last_summed * last_summed * last_summed + w2(2) * last_summed * last_summed,
		// vx, vy, vz (constant).
		relative_xyz_aa_gamma(3), relative_xyz_aa_gamma(4), relative_xyz_aa_gamma(5),
		// Gamma.
		w3(6) * last_summed * last_summed * last_summed + w2(6) * last_summed * last_summed,

//		cout << "delta xyz aa gamma "<< delta_xyz_aa_gamma << endl;

		// Compute delta frame.
		delta_ee_frame.set_from_xyz_angle_axis_gamma(delta_xyz_aa_gamma);

//		cout << "delta ee frame "<< delta_ee_frame << endl;

		// Compute desired interpolation end effector frame.
		int_ee_frame = current_end_effector_frame_ * delta_ee_frame;

//		cout << "interpolation ee frame "<< int_ee_frame << endl;

		// Compute inverse kinematics for desired pose. Pass previously desired joint position as current in order to receive continuous move.
		model_->inverse_kinematics_transform(int_joints, int_joints_old, int_ee_frame);

//		cout << int_joints << endl;

		// Transform joints to motors (and check motors/joints values).
		model_->i2mp_transform(int_motors, int_joints);

//		cout << int_motors << endl;

		// Add motors to vector.
		motor_interpolations_.row(i + 1) = int_motors.transpose();

		// Set last joint settings.
		int_joints_old = int_joints;
		// Add time slice related to this segment.
		if (i < N_POINTS - 2) {
			last_summed += time_deltas_(i + 1);
		}
	}

	// Display all motor interpolation poses.
/*	for (unsigned int l = 0; l < N_POINTS; ++l) {
		cout << "Motor interpolation point no " << l << ": " << motor_interpolations_.row(l) << endl;
	}*/
}

/**
 * @brief Compute motor_deltas for segments.
 *
 * Matrix in the form (equation 1.48) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] motor_deltas_for_segments_ Motor position increments to be realized in given segment.
 * @param [in] motor_interpolations_ Matrix containing interpolated motor poses.
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void compute_motor_deltas_for_segments(
		Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & motor_deltas_for_segments_,
		const Eigen::Matrix <double, N_SEGMENTS + 1, N_MOTORS> motor_interpolations_
		)
{
	for (int segment = 0; segment < N_SEGMENTS; ++segment) {
		for (int axis = 0; axis < N_MOTORS; ++axis) {
			motor_deltas_for_segments_(segment, axis) = motor_interpolations_(segment + 1, axis)
					- motor_interpolations_(segment, axis);
		}
	}

	// Display all motor increments.
/*	for (unsigned int l = 0; l < N_SEGMENTS; ++l) {
		cout << "Motor increments for segment " << l << ": " << motor_deltas_for_segments_.row(l) << endl;
	}*/
}

/**
 * @brief Computes matrix of the tau coefficients. The matrix is identical for every axis, thus is independent of axes number.
 *
 * Matrix in the form (equation 1.48) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @tparam N_SEGMENTS number of motion segments.
 *
 * @param [out] tau_coefficients_ Matrix containing time (tau) coefficients.
 * @param [in] time_deltas_ Times of motion for one segment (may be different for each segment!).
 */
template <unsigned int N_SEGMENTS>
void compute_tau_coefficients_matrix(
		Eigen::Matrix <double, N_SEGMENTS, N_SEGMENTS> & tau_coefficients_,
		const Eigen::Matrix <double, N_SEGMENTS, 1> time_deltas_
		)
{
	// Zero all matrix coefficients.
	tau_coefficients_ = Eigen::Matrix <double, N_SEGMENTS, N_SEGMENTS>::Zero(N_SEGMENTS, N_SEGMENTS);

	// First row.
	tau_coefficients_(0, 0) = time_deltas_(0) * 2.0 / 3.0;
	tau_coefficients_(0, 1) = time_deltas_(0) / 3.0;
	// Rows 2..n-2.
	for (int i = 1; i < N_SEGMENTS - 1; ++i) {
		tau_coefficients_(i, i - 1) = time_deltas_(i - 1) / 3.0;
		tau_coefficients_(i, i) = (time_deltas_(i) + time_deltas_(i - 1)) * 2.0 / 3.0;
		tau_coefficients_(i, i + 1) = time_deltas_(i) / 3.0;
	}
	// Last row.
	tau_coefficients_(N_SEGMENTS - 1, N_SEGMENTS - 2) = time_deltas_(N_SEGMENTS - 2) / 3.0;
	tau_coefficients_(N_SEGMENTS - 1, N_SEGMENTS - 1) = time_deltas_(N_SEGMENTS - 2) * 2.0 / 3.0
			+ time_deltas_(N_SEGMENTS - 1) / 2.0;

//	cout << "tau_coefficients:\n" << tau_coefficients_ << endl;
}

/**
 * @brief Computes matrix of the right side coefficients for every motor.
 *
 * Matrix in the form (equation 1.48) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] right_side_coefficients_ Right side coefficients (motor deltas).
 * @param [in] time_deltas_ Times of motion for one segment (may be different for each segment!).
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void compute_right_side_coefficients_vector(Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & right_side_coefficients_, const Eigen::Matrix <
		double, N_SEGMENTS, N_MOTORS> motor_deltas_, const Eigen::Matrix <double, N_SEGMENTS, 1> time_deltas_)
{
	for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		// First segment.
		right_side_coefficients_(0, mtr) = motor_deltas_(0, mtr) / time_deltas_(0);
		// 1..n-1 segments.
		for (int sgt = 1; sgt < N_SEGMENTS - 1; ++sgt) {
			right_side_coefficients_(sgt, mtr) = (motor_deltas_(sgt, mtr)) / (time_deltas_(sgt)) - (motor_deltas_(sgt
					- 1, mtr)) / (time_deltas_(sgt - 1));
		}

		// Last segment.
		right_side_coefficients_(N_SEGMENTS - 1, mtr) = (motor_deltas_(N_SEGMENTS - 1, mtr) * 3.0)
				/ (time_deltas_(N_SEGMENTS - 1) * 2.0) - (motor_deltas_(N_SEGMENTS - 2, mtr))
				/ (time_deltas_(N_SEGMENTS - 2));
	}

//	cout << "right_side_coefficients:\n" << right_side_coefficients_ << endl;
}

/**
 * @brief Computes 2w coefficients.
 *
 * Solves the matrix equation in the form (equation 1.48) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] m2w_ Matrix with 2w coefficients - for all segments and all motors respectively.
 * @param [in] a_ Matrix containing time (tau) coefficients.
 * @param [in] b_ Right side coefficients (motor deltas). (Computations will affect this parameters, thus it is not const!)
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void compute_motor_2w_polynomial_coefficients(
		Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m2w_,
		Eigen::Matrix <double, N_SEGMENTS, N_SEGMENTS> a_,
		Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> b_
		)
{
	// Zero all matrix coefficients.
	m2w_ = Eigen::Matrix <double, N_SEGMENTS, N_MOTORS>::Zero(N_SEGMENTS, N_MOTORS);

	// First step: substract equations, start from the last row.
	for (int sgt = N_SEGMENTS - 1; sgt > 0; --sgt) {
		// Compute fraction.
		double frac = a_(sgt, sgt) / a_(sgt - 1, sgt);
		// Multiply row sgt-1 by fraction.
		a_.row(sgt - 1) *= frac;
		b_.row(sgt - 1) *= frac;
		// Substract row k from the modified row k-1.
		a_.row(sgt - 1) -= a_.row(sgt);
		b_.row(sgt - 1) -= b_.row(sgt);
	}//: for all segments

	// Second step: obtain rest of 2w, starting from first row (2w1).
	m2w_.row(0) = b_.row(0) / a_(0, 0);
	for (int sgt = 1; sgt < N_SEGMENTS; ++sgt) {
		m2w_.row(sgt) = (b_.row(sgt) - a_(sgt, sgt - 1) * m2w_.row(sgt - 1)) / a_(sgt, sgt);
	}
//	cout << "2w:\n" << m2w_ << endl;
}


/**
 * @brief Computes 1w coefficients.
 *
 * Basing on the formulas (1.41 and 1.40) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] m1w_ Matrix with 1w coefficients - for all segments and all motors respectively.
 * @param [in] m2w_ Matrix with 2w coefficients, utilized for 1w computations.
 * @param [in] motor_deltas_ Motor position increments to be realized in given segment.
 * @param [in] taus_ Times of motion for one segment (may be different for each segment!).
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void compute_motor_1w_polynomial_coefficients(
		Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m1w_,
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> m2w_,
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> motor_deltas_,
		const Eigen::Matrix <double, N_SEGMENTS, 1> taus_
		)
{
	// Zero all matrix coefficients.
	m1w_ = Eigen::Matrix <double, N_SEGMENTS, N_MOTORS>::Zero(N_SEGMENTS, N_MOTORS);

	// Compute 1w for last segment.
	m1w_.row(N_SEGMENTS-1) = motor_deltas_.row(N_SEGMENTS-1) * 3.0 / (taus_(N_SEGMENTS-1) *2.0) - m2w_.row(N_SEGMENTS-1) * taus_(N_SEGMENTS-1) / 2.0;

	// Compute 1w for other segments.
	for (int sgt = 0; sgt < N_SEGMENTS-1; ++sgt) {
		m1w_.row(sgt) = motor_deltas_.row(sgt) / taus_(sgt) - taus_(sgt) * (m2w_.row(sgt+1) + m2w_.row(sgt) * 2.0 ) / 3.0;
	}
	// According to the 1.36, the 1w1 should be equal to zero.

//	cout << "1w:\n" << m1w_ << endl;
}


/**
 * @brief Computes 3w coefficients.
 *
 * Basing on the formulas (1.42 and 1.38) from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] m3w_ Matrix with 3w coefficients - for all segments and all motors respectively.
 * @param [in] m2w_ Matrix with 2w coefficients, utilized for 1w computations.
 * @param [in] motor_deltas_ Motor position increments to be realized in given segment.
 * @param [in] taus_ Times of motion for one segment (may be different for each segment!).
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void compute_motor_3w_polynomial_coefficients(
		Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m3w_,
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> m2w_,
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> motor_deltas_,
		const Eigen::Matrix <double, N_SEGMENTS, 1> taus_
		)
{
	// Zero all matrix coefficients.
	m3w_ = Eigen::Matrix <double, N_SEGMENTS, N_MOTORS>::Zero(N_SEGMENTS, N_MOTORS);

	// Compute 3w for last segment.
	m3w_.row(N_SEGMENTS-1) = - m2w_.row(N_SEGMENTS-1) / (taus_(N_SEGMENTS-1) *2.0) - motor_deltas_.row(N_SEGMENTS-1) / ( taus_(N_SEGMENTS-1) * taus_(N_SEGMENTS-1) * taus_(N_SEGMENTS-1) * 2.0);

	// Compute 3w for other segments.
	for (int sgt = 0; sgt < N_SEGMENTS-1; ++sgt) {
		m3w_.row(sgt) = (m2w_.row(sgt+1) - m2w_.row(sgt)) / ( taus_(sgt) *3.0);
	}

//	cout << "3w:\n" << m3w_ << endl;
}

/**
 * @brief Computes 0w coefficients.
 *
 * Basing on the (1.33) formula from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] m0w_ Matrix with 0w coefficients - for all segments and all motors respectively.
 * @param [in] motor_interpolations_ Matrix containing interpolated motor poses.
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void compute_motor_0w_polynomial_coefficients(
		Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m0w_,
		const Eigen::Matrix <double, N_SEGMENTS + 1, N_MOTORS> motor_interpolations_
		)
{
	// Zero all matrix coefficients.
	m0w_ = Eigen::Matrix <double, N_SEGMENTS, N_MOTORS>::Zero(N_SEGMENTS, N_MOTORS);

	// Compute 03w for all segments.
	for (int sgt = 0; sgt < N_SEGMENTS; ++sgt) {
		m0w_.row(sgt) = motor_interpolations_.row(sgt);
	}

//	cout << "m0w:\n" << m0w_ << endl;
}

/**
 * @brief Checks velocities constraints for all segments and motors.
 *
 * Computations based on the (1.51) formula from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [in] vmin_ Vector with minimum velocities of all motors [turns per second].
 * @param [in] vmax_ Vector with maximum velocities of all motors [turns per second].
 * @param [in] m3w_ Matrix with 3w coefficients - for all segments and all motors respectively.
 * @param [in] m2w_ Matrix with 2w coefficients - for all segments and all motors respectively.
 * @param [in] m1w_ Matrix with 1w coefficients - for all segments and all motors respectively.
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void check_velocities(
		const double vmin_[N_MOTORS],
		const double vmax_[N_MOTORS],
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m3w_,
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m2w_,
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m1w_
		)
{
	cout << "vmin:\n";
	for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		cout<< vmin_[mtr]<<" ";
	}
	cout<< endl;
	cout << "vmax:\n";
	for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		cout<< vmax_[mtr]<<" ";
	}
	cout<< endl;

	// Compute extreme velocities for all segments and motors (at once! - mi low eigen;)).
	Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> v_extremum = (m2w_.cwise() * m2w_).cwise() / (3.0 * m3w_) + m1w_;
	// Correct all NANs.
	for (int sgt = 0; sgt < N_SEGMENTS; ++sgt)
		for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
			if (m3w_(sgt, mtr) == 0.0)
				v_extremum(sgt, mtr) = m1w_ (sgt, mtr);
		}
	cout << "v_extremum:\n" << v_extremum << endl;

	// Check conditions for all segments and motors.
	for (int sgt = 0; sgt < N_SEGMENTS; ++sgt)
		for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		if (v_extremum(sgt, mtr) > vmax_[mtr])
			BOOST_THROW_EXCEPTION(nfe_motor_velocity_constraint_exceeded() << constraint_type(MAXIMUM_CONSTRAINT) << motor_number(mgr) << segment_number(sgt));
		else if (v_extremum(sgt, mtr) < vmin_[mtr])
			BOOST_THROW_EXCEPTION(nfe_motor_velocity_constraint_exceeded() << constraint_type(MINIMUM_CONSTRAINT) << motor_number(mgr) << segment_number(sgt));
		}
}


/**
 * @brief Checks velocities constraints for all segments and motors.
 *
 * Computations based on the (1.51) formula from the "Cartesian Trajectory generation for the PKM of the Swarm ItFIX system".
 *
 * @author tkornuta
 *
 * @tparam N_SEGMENTS number of motion segments.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [in] amin_ Vector with minimum accelerations (deaccelerations) of all motors [turns per second^2].
 * @param [in] amax_ Vector with maximum accelerations of all motors [turns per second^2].
 * @param [in] m3w_ Matrix with 3w coefficients - for all segments and all motors respectively.
 * @param [in] m2w_ Matrix with 2w coefficients - for all segments and all motors respectively.
 * @param [in] taus_ Times of motion for segments (may be different for each segment!).
 */
template <unsigned int N_SEGMENTS, unsigned int N_MOTORS>
void check_accelerations(
		const double amin_[N_MOTORS],
		const double amax_[N_MOTORS],
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m3w_,
		const Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> & m2w_,
		const Eigen::Matrix <double, N_SEGMENTS, 1> taus_
		)
{
	cout << "amin:\n";
	for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		cout<< amin_[mtr]<<" ";
	}
	cout<< endl;
	cout << "amax:\n";
	for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		cout<< amax_[mtr]<<" ";
	}
	cout<< endl;

	// Compute acceleration values at segment beginning.
	Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> a_start = 2.0 * m2w_;
	cout << "a_start:\n" << a_start << endl;

	// Compute acceleration values at the segment end.
	Eigen::Matrix <double, N_SEGMENTS, N_MOTORS> a_final;
	for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		a_final.col(mtr) = 6.0 * (m3w_.col(mtr).cwise() * taus_) + 2.0 * m2w_.col(mtr);
	}
	cout << "a_final:\n" << a_final << endl;

	// Check conditions for all segments and motors.
	for (int sgt = 0; sgt < N_SEGMENTS; ++sgt)
		for (int mtr = 0; mtr < N_MOTORS; ++mtr) {
		if ((a_start(sgt, mtr) > amax_[mtr]) || (a_final(sgt, mtr) > amax_[mtr]))
			BOOST_THROW_EXCEPTION(nfe_motor_acceleration_constraint_exceeded() << constraint_type(MAXIMUM_CONSTRAINT) << motor_number(mgr) << segment_number(sgt));
		else if ((a_start(sgt, mtr) < amin_[mtr]) || (a_final(sgt, mtr) > amin_[mtr]))
			BOOST_THROW_EXCEPTION(nfe_motor_acceleration_constraint_exceeded() << constraint_type(MINIMUM_CONSTRAINT) << motor_number(mgr) << segment_number(sgt));
		}
}

/**
 * @brief Computes PVT triplets for n segments, thus n+1 points.
 *
 * First point is the start position (pos, 0, tau) and the last (required by the EPOS) is (pos;0;0;).
 *
 * @author tkornuta
 *
 * @tparam N_POINTS Number of interpolation points.
 * @tparam N_MOTORS Number of manipulator motors.
 *
 * @param [out] p_ Matrix containing interpolated 'motor position poses'.
 * @param [out] v_ Matrix containing interpolated 'motor velocity poses'.
 * @param [out] t_ Vector containing interpolated 'motor time poses'.
 * @param [in] taus_ Times of motion for one segment (may be different for each segment!).
 * @param [in] m3w_ Matrix with 3w coefficients.
 * @param [in] m2w_ Matrix with 2w coefficients.
 * @param [in] m1w_ Matrix with 1w coefficients.
 * @param [in] m0w_ Matrix with 0w coefficients.
 */
template <unsigned int N_POINTS, unsigned int N_MOTORS>
void compute_pvt_triplets_for_epos(
		Eigen::Matrix <double, N_POINTS, N_MOTORS> & p_,
		Eigen::Matrix <double, N_POINTS, N_MOTORS> & v_,
		Eigen::Matrix <double, N_POINTS, 1> & t_,
		const Eigen::Matrix <double, N_POINTS-1, 1> taus_,
		const Eigen::Matrix <double, N_POINTS-1, N_MOTORS> m3w_,
		const Eigen::Matrix <double, N_POINTS-1, N_MOTORS> m2w_,
		const Eigen::Matrix <double, N_POINTS-1, N_MOTORS> m1w_,
		const Eigen::Matrix <double, N_POINTS-1, N_MOTORS> m0w_
		)
{
	// Start point.
	p_.row(0) = m0w_.row(0);
	v_.row(0) = Eigen::Matrix <double, 1, N_MOTORS>::Zero(1, N_MOTORS);

	// For all other interpolation points.
	for (int i = 1; i < N_POINTS; ++i) {
		p_.row(i) = m0w_.row(i-1) + m1w_.row(i-1)*taus_(i-1) + m2w_.row(i-1)*(taus_(i-1)*taus_(i-1)) + m3w_.row(i-1)*(taus_(i-1)*taus_(i-1)*taus_(i-1));
		v_.row(i) = m1w_.row(i-1) + 2.0*m2w_.row(i-1)*taus_(i-1) + 3.0*m3w_.row(i-1)*(taus_(i-1)*taus_(i-1));
		//  There are N_POINTS-1 segments, thus N_POINTS-1 'tau'.
		t_(i-1) = taus_(i-1);
	}
	// Set last segment movement time.
	t_(N_POINTS-1) = 0;

/*	cout<<"p "<<p_;
	cout<<"v "<<v_;
	cout<<"t "<<t_;*/
}

} // namespace pvat
} // namespace lib
} // namespace mrrocpp

#endif /* PVAT__CARTESIAN_HPP_ */

