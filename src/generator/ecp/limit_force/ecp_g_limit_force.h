//if !defined(_ECP_GEN_LIMIT_FORCE_H)
#define _ECP_GEN_LIMIT_FORCE_H

/*!
 * @file
 * @brief File contains limit_force generator declaration
 * @author mkula, Warsaw University of Technology
 * @ingroup generators
 */
#include "ecp_mp_g_limit_force.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/*!
 * @brief generator to send the bias (set offset) command to EDP force measurements
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class limit_force : public common::generator::generator
{
public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	limit_force(common::task::task& _ecp_task);

	/**
	 * @brief generates first step of transition function
	 * @return terminal condition value
	 */
	bool first_step();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

//#endif
