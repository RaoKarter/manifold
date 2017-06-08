/*
 * smp_hmc_unit_test.cc
 *
 *  Created on: Feb 14, 2017
 *      Author: Karthik Rao
 *
 *
 *      //! @verbatim
//!                                            -----------------------
//!      -----------       -----------        | DRAM0 	    	DRAMn |
//!     | processor |     | processor |       |   |  DRAM1    	  |   |
//!      -----------       -----------        |   |    | . . .    |   |
//!           |                 |              -----------------------
//!        ----------           ----------      |       HMC        |
//!        ----------           ----------      |     Crossbar     |
//!       | L1 cache |         | L1 cache |      ------------------
//!         |     |              |     |          |    |    |	 |
//!  ----------   |       ----------   |          |    |    |	 |
//! | L2 cache |  |      | L2 cache |  |          |    |    |	 |
//!  ----------   |       ----------   |          |    |    |	 |
//!       |       |            |       |          |    |    |	 |
//!        ----   |             ----   |          |    |    |	 |
//!            |  |                 |  |          |    |    |	 |
//!           -----                -----          |    |    |	 |
//!          | mux |              | mux |         |    |    |	 |
//!           -----                -----          |    |    |	 |
//!             |                    |            |    |    |	 |
//!  ---------------------------------------------------------------------------
//!       ------------         ------------       ------------      ------------
//!      | NetIntface |       | NetIntface |     | NetIntface |    | NetIntface |
//!       ------------         ------------       ------------      ------------
//!  ---------------------------------------------------------------------------
//! @endverbatim
 *
 * This file tests the HMC unit independently. The crossbar ports will send and
 * receive dummy requests to test the functionality of the unit as a whole.
 */


#include "../common/sysBuilder_llp_HUT.h"
#include "kernel/manifold.h"
#include <iostream>
#include "mpi.h"


using namespace std;
using namespace manifold::kernel;

int main(int argc, char** argv)
{
	if(argc != 2)
	{
		cerr << "Usage: smp_hmc_unit_test <config_file>" << endl;
		exit(1);
	}

	Manifold::Init(argc, argv);
	cout << "\n System has been initialized" << endl;

	SysBuilder_llp_HUT sysBuilder(argv[1]);

	sysBuilder.config_system();
	cout << "\n config system has completed" << endl;

	int N_LPs = 1; //number of LPs
	MPI_Comm_size(MPI_COMM_WORLD, &N_LPs);
	cout << "\n Number of LPs = " << N_LPs << endl;


#define REDIRECT_COUT

#ifdef REDIRECT_COUT
	// create a file into which to write debug/stats info.
	int Mytid;
	MPI_Comm_rank(MPI_COMM_WORLD, &Mytid);
	char buf[10];
	sprintf(buf, "DBG_LOG_HUT%d", Mytid);
	ofstream DBG_LOG_HUT(buf);

	//redirect cout to file.
	std::streambuf* cout_sbuf = std::cout.rdbuf(); // save original sbuf
	std::cout.rdbuf(DBG_LOG_HUT.rdbuf()); // redirect cout
#endif

	cout << "\n Going to build system" << endl;
	sysBuilder.build_system();
	cout << "\n System has been built" << endl;


	//==========================================================================
	//start simulation
	//==========================================================================
	sysBuilder.print_config(cout);
	Manifold::StopAt(sysBuilder.get_stop_tick());
    Manifold::Run();


	sysBuilder.print_stats(cerr);


#ifdef REDIRECT_COUT
	std::cout.rdbuf(cout_sbuf);
#endif

	Manifold::Finalize();
}



