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
	max_force_ot = _ecp_task.config.value <std::string>("max_force", "[ecp_irp6ot_m]");
	max_force_p = _ecp_task.config.value <std::string>("max_force", "[ecp_irp6p_m]");
	max_torque_ot = _ecp_task.config.value <std::string>("max_torque", "[ecp_irp6ot_m]");
	max_torque_p = _ecp_task.config.value <std::string>("max_torque", "[ecp_irp6p_m]");
}

bool limit_force::next_step()
{

	std::cout << "bias_edp_force" << node_counter << std::endl;

	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double wx = force_torque[0];
	double wy = force_torque[1];
	double wz = force_torque[2];

	double v = hypot(wx, wy);
	double v = hypot(v, wz);

	if(calculate_force(v, robot_name))
	/*the_robot->ecp_command.instruction_type = lib::SET;
	the_robot->ecp_command.set_type = ROBOT_MODEL_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::FORCE_BIAS;*/

	return true;
}
bool calculate_force(double force, lib::robot_name_t robot_name){


	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
