/*!
 * @file
 * @brief File contains limit_force generator definition
 * @author mkula, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "base/ecp/ecp_robot.h"
#include "ecp_g_limit_force.h"
#include "math.h"
#include "string.h"
#include "base/lib/mrmath/k_vector.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			limit_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

limit_force::limit_force(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) : common::generator::constant_velocity(_ecp_task, pose_spec, axes_num)
{
	generator_name = ecp_mp::generator::ECP_GEN_LIMIT_FORCE;
	max_force_ot = _ecp_task.config.value <float>("max_force", "[ecp_irp6ot_m]");
	max_force_p = _ecp_task.config.value <float>("max_force", "[ecp_irp6p_m]");
	max_torque_ot = _ecp_task.config.value <float>("max_torque", "[ecp_irp6ot_m]");
	max_torque_p = _ecp_task.config.value <float>("max_torque", "[ecp_irp6p_m]");
}

bool limit_force::next_step()
{

	std::cout << "bias_edp_force" << node_counter << std::endl;

	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);


	force = sqrt(force_torque[0]*force_torque[0]+force_torque[1]*force_torque[1]+force_torque[2]*force_torque[2]);

	force_torque[3]=fabs(force_torque[3]);
	force_torque[4]=fabs(force_torque[4]);
	force_torque[5]=fabs(force_torque[5]);

	torque = std::max(force_torque[5], std::max(force_torque[3], force_torque[4]));

	if(force==max_force_ot || force==max_force_p){
		std::cout << "force overload" << node_counter << std::endl;
		return false;
	}

	else if(torque==max_torque_ot || torque==max_torque_p){
		std::cout << "torque overload" << node_counter << std::endl;
		return false;
	}
	else return true;

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
