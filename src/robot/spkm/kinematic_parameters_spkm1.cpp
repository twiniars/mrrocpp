/*!
 * @file kinematic_parameters_spkm1.cpp
 * @brief File containing the initialization of parameters of the SPKM1 agent.
 *
 * @author Tomasz Kornuta
 * @date 23-12-2011
 *
 * @ingroup SIF_KINEMATICS spkm
*/

#include "kinematic_parameters_spkm1.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm1 {

kinematic_parameters_spkm1::kinematic_parameters_spkm1()
{
	// Initialization of parameters describing the synchronisation positions (in joints).
	synchro_positions[0] = 0.242;
	synchro_positions[1] = 0.237;
	synchro_positions[2] = 0.242;
	synchro_positions[3] = 0.0;
	synchro_positions[4] = 0.0;
	synchro_positions[5] = 0.0;

	// Initialization of parameters related to conversion from motor positions to joints.
	// Parameters for conversion for linear DOFs are:
	// * Lead roller screw is equal to 5mm.
	// * The encoder has 500 CPT (Counts per turn).
	// * Quadcounts = 4 x Encoder Counts.
	// * The gear ratio is equal to 9.
	double linear_mp2i_ratio = 0.005 / (4 * 500 * 9);
	// Parameters for conversion for rotational DOFs are:
	// * The encoder has 2000 CPT (Counts per turn).
	// * Quadcounts = 4 x Encoder Counts.
	// * The gear ratio is equal to 100.
	double rotational_mp2i_ratio = -2*M_PI / (4 * 2000 * 100);
	mp2i_ratios[0] = linear_mp2i_ratio;
	mp2i_ratios[1] = linear_mp2i_ratio;
	mp2i_ratios[2] = linear_mp2i_ratio;
	mp2i_ratios[3] = rotational_mp2i_ratio;
	mp2i_ratios[4] = rotational_mp2i_ratio;
	mp2i_ratios[5] = rotational_mp2i_ratio;

	// Initialization of the encoder resolution. Equals to the Counts Per Turn (CPT) x 4.
	encoder_resolution[0] = 500*4;
	encoder_resolution[1] = 500*4;
	encoder_resolution[2] = 500*4;
	encoder_resolution[3] = 2000*4;
	encoder_resolution[4] = 2000*4;
	encoder_resolution[5] = 2000*4;

	// Initialization of upper motors limits vector.
	upper_motor_pos_limits[0] = 8000;
	upper_motor_pos_limits[1] = 8000;
	upper_motor_pos_limits[2] = 8000;
	upper_motor_pos_limits[3] = 320000;
	upper_motor_pos_limits[4] = 90000;
	upper_motor_pos_limits[5] = 310000;

	// Initialization of lower motors limits vector.
	// Those are the "unsafe" (in terms that robot can hit its "shell" from inside) values.;
	lower_motor_pos_limits[0] = -350000;
	lower_motor_pos_limits[1] = -350000;
	lower_motor_pos_limits[2] = -350000;
	lower_motor_pos_limits[3] = -320000;
	lower_motor_pos_limits[4] = -200000;
	lower_motor_pos_limits[5] = -310000;

	// Initialization of upper joints vector.
	// Those are the "safe" limits, not related to synchronization sensors positions.;
	upper_joints_limits[0] = 0.28;
	upper_joints_limits[1] = 0.287;
	upper_joints_limits[2] = 0.28;
	upper_joints_limits[3] = 2.7489;
	upper_joints_limits[4] = 0.7;
	upper_joints_limits[5] = 2.43;

	// Initialization of lower joints limits vector.
	// The lower values are related to positions of synchronization sensors.
	lower_joints_limits[0] = 0.242;
	lower_joints_limits[1] = 0.237;
	lower_joints_limits[2] = 0.242;
	lower_joints_limits[3] = -2.2777;
	lower_joints_limits[4] = -1.5708;
	lower_joints_limits[5] = -2.43;

	// Initialization of upper thyk alpha angle limit.
	// Those values were determined experimentally.
	upper_alpha_thyk_angle_limit[0] = 30.0;
	upper_alpha_thyk_angle_limit[1] = 50.0;
	upper_alpha_thyk_angle_limit[2] = 30.0;

	// Initialization of lower thyk alpha angle limit.
	// Those values were determined experimentally.
	lower_alpha_thyk_angle_limit[0] = -30.0;
	lower_alpha_thyk_angle_limit[1] = -50.0;
	lower_alpha_thyk_angle_limit[2] = -30.0;

	// Initialization of upper thyk beta angle limit.
	// Those values were determined experimentally.
	upper_beta_thyk_angle_limit[0] = 30.0;
	upper_beta_thyk_angle_limit[1] = 50.0;
	upper_beta_thyk_angle_limit[2] = 30.0;

	// Initialization of lower thyk beta angle limit.
	// Those values were determined experimentally.
	lower_beta_thyk_angle_limit[0] = -30.0;
	lower_beta_thyk_angle_limit[1] = -50.0;
	lower_beta_thyk_angle_limit[2] = -30.0;

	// Lower platform: Initialize the jb coordinate of P1A in O(ib,jb,kb).
	lA = -0.05;

	// Lower platform: Initialize the ib coordinate of P1B in O(ib,jb,kb).
	lB = 0.18;

	// Lower platform: Initialize the jb coordinate of P1C in O(ib,jb,kb).
	lC = 0.05;

	// Upper platform: Initialize the j coordinate of P4A in P(ijk).
	uA = -0.05;

	// Upper platform: Initialize the i coordinate of P5B in P(ijk).
	uB = 0.086;

	// Upper platform: Initialize the j coordinate of P4C in P(ijk).
	uC = 0.05;

	// Initialization of vector representing a translation from P (middle of upper P platform) and S (middle of the spherical wrist).
	P_S_P << 0, 0, 0.0905;

	// Initialization of transformation from P (middle of upper P platform) and S (middle of the spherical wrist).
	P_S_T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0.0905, 1;

	// Initialization of transformation from W (SW end-effector) to S (middle of the spherical wrist).
	// The W_S_T is the inversion of:
	// S_W_P = [0.0; 0; 0.0725];
	// S_W_R = [1, 0, 0; 0 1 0; 0, 0, 1]
	W_S_T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -0.0725, 1;
}

}

}

}