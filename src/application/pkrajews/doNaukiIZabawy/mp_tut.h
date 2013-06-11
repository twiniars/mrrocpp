#if !defined(__MP_T_PKRAJEWS_TUT)
#define __MP_T_PKRAJEWS_TUT

#include "base/mp/mp_task.h"

#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>
#include <vector>

//#include "DecisionVector.h"
#include "CubeReading.hpp"

namespace mrrocpp {
namespace mp {
namespace task {

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;
/**
 * @defgroup tut
 * @ingroup application
 * Taka aplikacyja
 */

class tut : public task
{
protected:

public:

	/**
	 * Constructor.
	 */
	tut(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	void main_task_algorithm(void);
	void configure_discode();
	Types::Mrrocpp_Proxy::CubeReading read_from_discode();
	const std::string config_section_name;
	bool run_vs;
	int vs_settle_time;
	std::string robot_name;

private:
	boost::shared_ptr<discode_sensor> discode;
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
