/*
 * sysBuilder_llp_HUT.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: Karthik Rao
 *
 *  
 */

#include "sysBuilder_llp_HUT.h"

#include "kernel/clock.h"
#include "kernel/component.h"
#include "kernel/manifold.h"
#define MAX_NODES 36

using namespace manifold::kernel;
using namespace libconfig;

//====================================================================
//====================================================================
SysBuilder_llp_HUT :: SysBuilder_llp_HUT(const char* fname)
{
	try 
	{
		m_config.readFile(fname);
		m_config.setAutoConvert(true);
	}
	catch (FileIOException e) 
	{
		cerr << "Cannot read configuration file " << fname << endl;
		exit(1);
	}
	catch (ParseException e) 
	{
		cerr << "Cannot parse configuration file " << fname << endl;
		exit(1);
	}

	m_conf_read = false;

	m_mc_builder = 0;

	m_default_clock = 0;
}

SysBuilder_llp_HUT :: ~SysBuilder_llp_HUT()
{
	delete m_mc_builder;
	delete m_default_clock;
}

//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: config_system()
{
	assert(m_conf_read == false);
	config_components();
	m_conf_read = true;
}



//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: config_components()
{
	try 
	{
		//simulation parameters
		STOP = m_config.lookup("simulation_stop");

		try
		{
			m_DEFAULT_CLOCK_FREQ = m_config.lookup("default_clock");
			assert(m_DEFAULT_CLOCK_FREQ > 0);
		}
		catch (SettingNotFoundException e)
		{
			//if default clock not defined
			m_DEFAULT_CLOCK_FREQ = -1;
		}

		//memory controller
		const char* mem_chars = m_config.lookup("mc.type");
		string mem_str = mem_chars;
		// create HMC builder obj
		m_mc_builder = new HMC_builder(this);
		m_mc_builder->read_config(m_config);
		//xbar assignment
		//the node indices of xbar are in an array, each value between 0 and MAX_NODES-1
		MAX_VAULTS = m_config.lookup("max_vaults");
		MAX_SERDES = m_config.lookup("max_serdes");
		Setting& setting_hmc_xbar = m_config.lookup("mc.xbar.node_idx");
		num_serdes = m_config.lookup("mc.xbar.num_serdes");
		int num_xbar_nodes = setting_hmc_xbar.getLength(); //number of IRIS network nodes
		assert(num_xbar_nodes >=1 && num_xbar_nodes <= MAX_NODES);
		trans_size = m_config.lookup("mc.trans_size"); // Transaction size = cache line size

		this->mc_node_idx_vec.resize(num_xbar_nodes);

		for(int i=0; i < num_xbar_nodes; i++)
		{
			assert((int)setting_hmc_xbar[i] >=0 && (int)setting_hmc_xbar[i] < MAX_NODES);
			mc_node_idx_set.insert((int)setting_hmc_xbar[i]);
			this->mc_node_idx_vec[i] = (int)setting_hmc_xbar[i];
		}
		assert(mc_node_idx_set.size() == (unsigned)num_xbar_nodes); //verify no 2 indices are the same
        
    }
    catch (SettingNotFoundException e)
    {
		cout << e.getPath() << " not set." << endl;
		exit(1);
    }
    catch (SettingTypeException e)
    {
		cout << e.getPath() << " has incorrect type." << endl;
		exit(1);
    }
}


//====================================================================
// For QsimProxy front-end
//====================================================================
void SysBuilder_llp_HUT :: build_system()
{
	assert(m_conf_read == true);

	// create default clock, this is the Master Clock
	m_default_clock = 0;
	if(m_DEFAULT_CLOCK_FREQ > 0)
		m_default_clock = new Clock(m_DEFAULT_CLOCK_FREQ);

	cout << "\n Going to create nodes...";
	create_nodes();
	cout << "\n Nodes created" << endl;

	//connect components
	cout << "\n connecting components..." << endl;
	connect_components();
	cout << "\n connecting components done." << endl;
}

//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: create_nodes()
{
	m_node_conf.resize(MAX_NODES);
	do_partitioning_1_part(0);
	cout<<"\n Create_mcs" << endl;
   	m_mc_builder->create_mcs(mc_id_lp_map);
   	cout<<"\n Created mcs" << endl;

        dep_injection_for_mcp();
}

//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: do_partitioning_1_part(int n_lps)
{
    int lp_idx = 1; //the network is LP 0

	for(int i=0; i<MAX_NODES; i++)
	{
		bool flag = false;
		if(n_lps == 1)
			lp_idx = 0;
		else if(n_lps == 2)
			lp_idx = 1;

		if(mc_node_idx_set.find(i) != mc_node_idx_set.end())
		{ //MC node
			m_node_conf[i].type = MC_NODE;
			m_node_conf[i].lp = 0;
			mc_id_lp_map[i] = m_node_conf[i].lp;
			flag = true;
		}
		if (!flag)
		{
			m_node_conf[i].type = EMPTY_NODE;
		}
	}
}

//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: dep_injection_for_mcp()
{
	manifold::uarch::HMCDestMap *hmc_map = new manifold::uarch::HMCDestMap(
		                                mc_node_idx_vec, num_serdes, int(MAX_VAULTS / MAX_SERDES), trans_size );
	cout << "hmc_map object " << hmc_map << endl;
	cout << "HMCMap nodes size " << hmc_map->get_nodes_size() << endl;
	cout << "HMCDestMap selector bits " << dec << hmc_map->get_page_offset_bits() << endl;
	cout << "HMCDestMap selector mask " << hex << hmc_map->get_selector_mask() << endl;
	cout << "HMCDestMap cache line offset bits" << dec << hmc_map->get_byte_offset_bits() << endl;
	/*
	 * num_serdes can be 2 or 4. The number of HMCs = length(mc_node_idx_vec) / num_serdes
	 * Eg: 			length(mc_node_idx_vec) = 16
	 * 				num_serdes = 4
	 * 				number of HMCs = 16 / 4 = 4
	 */
	m_mc_builder->set_mc_map_obj(hmc_map);
}

//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: connect_components()
{
	cout << "\n Connecting network to HMC" << endl;
#ifdef HUTDEBUG
	m_mc_builder->connect_mc_network();
#endif
	cout << "\n Connected network to HMC" << endl;
}

//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: print_config(ostream& out)
{
	for(int i=0; i<MAX_NODES; i++) 
	{
		out << "node " << i;
		if(m_node_conf[i].type == CORE_NODE)
			out << "  core node";
		else if(m_node_conf[i].type == MC_NODE)
			out << "  mc node";
		else if(m_node_conf[i].type == CORE_MC_NODE)
			out << "  core and mc node";
		else if(m_node_conf[i].type == L2_NODE)
			out << "  l2 node";
		else
			out << "  empty node";
		if(m_node_conf[i].type != EMPTY_NODE)
			out << " lp= " << m_node_conf[i].lp << endl;
		else
			out << "\n";
	}

	out <<"\n********* Configuration *********\n";

#ifdef FORECAST_NULL
    out << "Forecast Null: on\n";
#else
    out << "Forecast Null: off\n";
#endif

	m_mc_builder->print_config(out);
}


//====================================================================
//====================================================================
void SysBuilder_llp_HUT :: print_stats(ostream& out)
{
    m_mc_builder->print_stats(out);
    Manifold::print_stats(out);
}
