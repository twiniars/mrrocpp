#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <iostream>

#include "mp_tut.h"

// ecp generators to be commanded
#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/constant_velocity/ecp_mp_g_constant_velocity.h"


#include "application/visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"

// mp_robots headers
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "application/irp6_tfg/ecp_mp_g_tfg.h"
#include "robot/irp6_tfg/dp_tfg.h"

#include "robot/irp6p_m/mp_r_irp6p_m.h"

//#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
//#include "application/irp6_tfg/ecp_mp_g_tfg.h"
//#include "robot/irp6_tfg/dp_tfg.h"

#include "sensor/discode/discode_sensor.h"

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;
using mrrocpp::ecp_mp::sensor::discode::ds_exception;

namespace mrrocpp {
namespace mp {
namespace task {

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void tut::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	//ACTIVATE_MP_ROBOT(irp6p_tfg);
}

tut::tut(lib::configurator &_config) :
		task(_config)
{

	//run_vs = config.value <bool>("run_vs", config_section_name);
	//vs_settle_time = config.value <int>("vs_settle_time", config_section_name);
	//robot_name = config.value <std::string>("robot_name", config_section_name);
	configure_discode();
}

void tut::configure_discode()
{
	sr_ecp_msg->message("configure_discode");
	char config_section_name[]= {"[Discode_TEST]"};
	discode = boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor>(new mrrocpp::ecp_mp::sensor::discode::discode_sensor(config, config_section_name));

	try {
		discode->configure_sensor();
		sr_ecp_msg->message("sensor_ready");

		discode_sensor::discode_sensor_state st;
		st = discode->get_state();

		if(st == 0) sr_ecp_msg->message("sensor state 0");
		if(st == 1) sr_ecp_msg->message("sensor state 1");
		if(st == 2) sr_ecp_msg->message("sensor state 2");
		if(st == 3) sr_ecp_msg->message("sensor state 3");
		if(st == 4) sr_ecp_msg->message("sensor state 4");


	}catch(mrrocpp::ecp_mp::sensor::discode::ds_exception e){
		sr_ecp_msg->message("configure_discode error");
		sr_ecp_msg->message(e.what());
	}
}

Types::Mrrocpp_Proxy::CubeReading tut::read_from_discode()
{
	sr_ecp_msg->message("read_from_discode");

	Types::Mrrocpp_Proxy::CubeReading reading;

	try {
		discode->get_reading();
		wait_ms(500);
		reading = discode->retreive_reading <Types::Mrrocpp_Proxy::CubeReading>();
		sr_ecp_msg->message("reading recived");

	} catch (ds_exception e){
		sr_ecp_msg->message("read_from_discode error");
		sr_ecp_msg->message(e.what());
	}

	return reading;
}

void tut::main_task_algorithm(void)
{
	sr_ecp_msg->message("Tut (MP) START");
	int i;
	//Czeka na kostkę
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/cubePosition/getCube1.trj", lib::irp6ot_m::ROBOT_NAME);
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/cubePosition/getCube1.trj", lib::irp6p_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.07, lib::irp6ot_tfg::ROBOT_NAME);
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tut/trajectory_track_r.trj", lib::irp6ot_tfg::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME,lib::irp6ot_tfg::ROBOT_NAME);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	std::cout << "WŁÓŻ KOSTKĘ I PRESS ENTER\n";
	std::cout << "OK\n";
	//Bierze kostkę
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.059, lib::irp6ot_tfg::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_tfg::ROBOT_NAME);

	//Pozycja 1
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/cubePosition/showWall1.trj", lib::irp6ot_m::ROBOT_NAME);
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/cubePosition/getCube1.trj", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);

	/*while(1)
	{
		bool exist=read_from_discode().objectVisible;
		if(exist==true) break;
	}

	sr_ecp_msg->message("Both Joint");

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/trajectory_track.trj", lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/trajectory_post.trj", lib::irp6p_m::ROBOT_NAME);
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tut/trajectory_track_r.trj", lib::irp6ot_tfg::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);

	sr_ecp_msg->message("Both Bias");

	*/


	/*EXAMPLE OF USAGE
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/cubePosition/getCube1.trj", lib::irp6ot_m::ROBOT_NAME);
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/pkrajews/doNaukiIZabawy/cubePosition/getCube1.trj", lib::irp6p_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY, (int) lib::ABSOLUTE, 0.07, lib::irp6ot_tfg::ROBOT_NAME);
	//set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 0, "../../src/application/tut/trajectory_track_r.trj", lib::irp6ot_tfg::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME,lib::irp6ot_tfg::ROBOT_NAME);//, lib::irp6p_m::ROBOT_NAME);

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6ot_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, 0, "", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME, lib::irp6p_m::ROBOT_NAME);
	*/

	sr_ecp_msg->message("Pozycja do zabawy przyjęta");

}

task* return_created_mp_task(lib::configurator &_config)
{
	return new tut(_config);
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
