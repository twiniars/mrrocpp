/*!
 * @file
 * @brief File contains limit_force generator definition
 * @author mkula, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "base/ecp/ecp_robot.h"
#include "ecp_g_limit_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			limit_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

limit_force::limit_force(common::task::task& _ecp_task) :
		common::generator::generator(_ecp_task)
{
	generator_name = ecp_mp::generator::ECP_GEN_LIMIT_FORCE;
}

bool limit_force::first_step()
{

//	std::cout << "bias_edp_force" << node_counter << std::endl;

	/*the_robot->ecp_command.instruction_type = lib::SET;
	the_robot->ecp_command.set_type = ROBOT_MODEL_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::FORCE_BIAS;*/

	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
