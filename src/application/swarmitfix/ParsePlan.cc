/*
 * ParsePlan.cc
 *
 *  Created on: Nov 9, 2011
 *      Author: ptroja
 */

#include <iostream>
#include <fstream>
#include <memory>
#include <exception>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include "serialization.h"

#include "planner.h"
#include "base/lib/mrmath/homog_matrix.h"
#include "base/lib/mrmath/mrmath.h"

int main(int argc, char *argv[])
{
	// Check for input arguments
	if(argc < 2) {
		std::cerr << "Usage: " << argv[0] << " plan_file.xml" << std::endl;
		return -1;
	}

	try {
		// XML validation settings
		xml_schema::Properties props;

		// Add XSD validation to parser's properties
		props.no_namespace_schema_location ("plan.xsd");

		//const Plan p = *plan(argv[1], xml_schema::Flags::dont_validate);
		Plan p = *plan(argv[1], 0, props);

		std::cerr << "mbase item # " << p.mbase().item().size() << std::endl;
		std::cerr << "pkm item # " << p.pkm().item().size() << std::endl;

		{
			// make an archive
			std::ofstream ofs("foo.xml");
			assert(ofs.good());
			boost::archive::xml_oarchive oa(ofs);
			oa << boost::serialization::make_nvp("item", p.pkm().item().front());
		}

		{
			// open the archive
			std::ifstream ifs("foo.xml");
			assert(ifs.good());
			boost::archive::xml_iarchive ia(ifs);

			// restore the schedule from the archive
			ia >> boost::serialization::make_nvp("item", p.pkm().item().front());
		}

		{
			// make an archive
			std::ofstream ofs("foo2.xml");
			assert(ofs.good());
			boost::archive::xml_oarchive oa(ofs);
			oa << boost::serialization::make_nvp("item", p.pkm().item().front());
		}

		// Create planner object
		//planner pp(argv[1]);

		// Start execution
		//pp.start();

	} catch (const xml_schema::Exception & e) {
		std::cerr << "Exception::what(): " << e << std::endl;
		return 1;
	}

	return 0;
}
