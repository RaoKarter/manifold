#include "mc_builder.h"
#include "sysBuilder_llp.h"
#include "cache_builder.h"
#include "mcp-cache/coh_mem_req.h"
#include "CaffDRAM/McMap.h"

using namespace libconfig;
using namespace manifold::kernel;
using namespace manifold::uarch;
using namespace manifold::mcp_cache_namespace;
using namespace manifold::caffdram;
using namespace manifold::dramsim;
using namespace manifold::iris;
using namespace manifold::xbar_namespace;

void MemControllerBuilder :: print_config(std::ostream& out)
{
    out << "Memory controller:\n";
    out << "  num of nodes: " << m_NUM_MC << endl;

}




void CaffDRAM_builder :: read_config(Config& config)
{
    try {
    Setting& mc_nodes = config.lookup("mc.node_idx");
    m_NUM_MC = mc_nodes.getLength();

    m_MC_DOWNSTREAM_CREDITS = config.lookup("mc.downstream_credits");

    m_MEM_MSG_TYPE = config.lookup("network.mem_msg_type");
    m_CREDIT_MSG_TYPE = config.lookup("network.credit_msg_type");
    }
    catch (SettingNotFoundException e) {
    cout << e.getPath() << " not set." << endl;
    exit(1);
    }
    catch (SettingTypeException e) {
    cout << e.getPath() << " has incorrect type." << endl;
    exit(1);
    }

}



void CaffDRAM_builder :: create_mcs(map<int, int>& id_lp)
{
    Controller :: Set_msg_types(m_MEM_MSG_TYPE, m_CREDIT_MSG_TYPE);

    for(map<int,int>::iterator it = id_lp.begin(); it != id_lp.end(); ++it) {
        int node_id = (*it).first;
    int lp = (*it).second;
    int cid = Component :: Create<Controller>(lp, node_id, m_dram_settings, m_MC_DOWNSTREAM_CREDITS);
    m_mc_id_cid_map[node_id] = cid;
    }

}



void CaffDRAM_builder :: connect_mc_network(NetworkBuilder* net_builder)
{
    switch(net_builder->get_type()) {
        case NetworkBuilder::IRIS: {
        Iris_builder* irisBuilder = dynamic_cast<Iris_builder*>(net_builder);
        assert(irisBuilder != 0);

        const std::vector<CompId_t>& ni_cids = net_builder->get_interface_cid();
        for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it) {
            int node_id = (*it).first;
            int mc_cid = (*it).second;
            if (irisBuilder->get_topology() == "TORUS6P")
                {
                    assert(node_id >= 0 && node_id < int(ni_cids.size())/2 );
                    //????????????????????????? todo: use proper clock!!
                    //???????????????????????? todo: Mem_msg is MCP-cache
                    switch(m_sysBuilder->get_cache_builder()->get_type()) {
                    case CacheBuilder::MCP_CACHE:
                    case CacheBuilder::MCP_L1L2:
                        Manifold :: Connect(mc_cid, Controller::PORT0, &Controller::handle_request<manifold::mcp_cache_namespace::Mem_msg>,
                                            ni_cids[node_id*2+1], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT,
                                            &GenNetworkInterface<NetworkPacket>::handle_new_packet_event, Clock::Master(), Clock::Master(), 1, 1);
                        break;
                    default:
                        assert(0);
                    }
                }
            else
                {
                    assert(node_id >= 0 && node_id < int(ni_cids.size()) );
                    //????????????????????????? todo: use proper clock!!
                    //???????????????????????? todo: Mem_msg is MCP-cache
                    switch(m_sysBuilder->get_cache_builder()->get_type()) {
                    case CacheBuilder::MCP_CACHE:
                    case CacheBuilder::MCP_L1L2:
                        Manifold :: Connect(mc_cid, Controller::PORT0, &Controller::handle_request<manifold::mcp_cache_namespace::Mem_msg>,
                                            ni_cids[node_id], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT,
                                            &GenNetworkInterface<NetworkPacket>::handle_new_packet_event, Clock::Master(), Clock::Master(), 1, 1);
                        break;
                    default:
                        assert(0);
                    }
                }

        }//for
        }
        break;
        default:
        assert(0);
        break;
    }//switch

}

void CaffDRAM_builder :: set_mc_map_obj(manifold::uarch::DestMap *mc_map)
{
    manifold::caffdram::CaffDramMcMap *m = dynamic_cast<manifold::caffdram::CaffDramMcMap*>(mc_map);

    for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it) {
        int node_id = (*it).first;
        Controller* mc = manifold::kernel::Component :: GetComponent<Controller>(m_mc_id_cid_map[node_id]);

        if (mc)
            mc->set_mc_map(m);
    }
}


void CaffDRAM_builder :: print_config(std::ostream& out)
{
    MemControllerBuilder :: print_config(out);
    out << "  MC type: CaffDRAM\n";
}


void CaffDRAM_builder :: print_stats(std::ostream& out)
{
    for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it) {
        int cid = (*it).second;
    Controller* mc = Component :: GetComponent<Controller>(cid);
    if(mc)
        mc->print_stats(out);
    }
}


//####################################################################
// DramSim2
//####################################################################

void DramSim_builder :: read_config(Config& config)
{
    try {
    Setting& mc_nodes = config.lookup("mc.node_idx");
    m_NUM_MC = mc_nodes.getLength();

    try {
        Setting& mc_clocks = config.lookup("mc.clocks");
        assert((unsigned)mc_clocks.getLength() == m_NUM_MC);
        m_CLOCK_FREQ.resize(m_NUM_MC);


        for(unsigned i=0; i<m_NUM_MC; i++) {
        m_CLOCK_FREQ[i] = (double)mc_clocks[i];

        }

        m_use_default_clock = false;
    }
    catch (SettingNotFoundException e) {
        //mc clock not defined; use default
        m_use_default_clock = true;
    }

    m_MC_DOWNSTREAM_CREDITS = config.lookup("mc.downstream_credits");

    m_MEM_MSG_TYPE = config.lookup("network.mem_msg_type");
    m_CREDIT_MSG_TYPE = config.lookup("network.credit_msg_type");

    //libconfig++ cannot assigne to string directly
    const char* chars = config.lookup("mc.dramsim2.dev_file");
    m_DEV_FILE = chars;
    chars = config.lookup("mc.dramsim2.sys_file");
    m_SYS_FILE = chars;
    m_MEM_SIZE = config.lookup("mc.dramsim2.size");
    }
    catch (SettingNotFoundException e) {
    cout << e.getPath() << " not set." << endl;
    exit(1);
    }
    catch (SettingTypeException e) {
    cout << e.getPath() << " has incorrect type." << endl;
    exit(1);
    }

}



void DramSim_builder :: create_mcs(map<int, int>& id_lp)
{

    //create clocks if necessary
    if(m_use_default_clock == false) {
        if(m_CLOCK_FREQ.size() != 1 && m_CLOCK_FREQ.size() != m_NUM_MC) {
        cerr << "Wrong number of clocks for DRAMSim; requires 1 or " << m_NUM_MC << endl;
    }
        m_clocks.resize(m_CLOCK_FREQ.size());
        for(unsigned i=0; i<m_clocks.size(); i++) {
        m_clocks[i] = new manifold::kernel::Clock(m_CLOCK_FREQ[i]);
    }
    }

    Dram_sim :: Set_msg_types(m_MEM_MSG_TYPE, m_CREDIT_MSG_TYPE);

    Dram_sim_settings settings(m_DEV_FILE.c_str(), m_SYS_FILE.c_str(), m_MEM_SIZE, false, m_MC_DOWNSTREAM_CREDITS);

    Clock* clock = 0;
    if(m_use_default_clock)
    clock = m_sysBuilder->get_default_clock();
    else if(m_clocks.size() == 1)
    clock = m_clocks[0];

    int i=0;
    for(map<int,int>::iterator it = id_lp.begin(); it != id_lp.end(); ++it) {
        int node_id = (*it).first;
    int lp = (*it).second;
    if(!m_use_default_clock && m_clocks.size() > 1)
        clock = m_clocks[i++];
    int cid = Component :: Create<Dram_sim>(lp, node_id, settings, *clock);
    m_mc_id_cid_map[node_id] = cid;
    }
}



void DramSim_builder :: connect_mc_network(NetworkBuilder* net_builder)
{
    switch(net_builder->get_type()) {
        case NetworkBuilder::IRIS: {
        Iris_builder* irisBuilder = dynamic_cast<Iris_builder*>(net_builder);
        assert(irisBuilder != 0);

        const std::vector<CompId_t>& ni_cids = net_builder->get_interface_cid();
        for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it) {
            int node_id = (*it).first;
            int mc_cid = (*it).second;
            Dram_sim* dram_sim = Component :: GetComponent<Dram_sim>(mc_cid);
                    if (irisBuilder->get_topology() == "TORUS6P")
                    {
              assert(node_id >= 0 && node_id < int(ni_cids.size())/2 );
              if(dram_sim) { //no need to call Connect if MC is not in this LP
            //????????????????????????? todo: MCP use proper clock!!
            switch(m_sysBuilder->get_cache_builder()->get_type()) {
                case CacheBuilder::MCP_CACHE:
                case CacheBuilder::MCP_L1L2:
                Manifold :: Connect(mc_cid, Controller::PORT0, &Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
                            ni_cids[node_id*2+1], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT,
                            &GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
                            *(dram_sim->get_clock()), Clock::Master(), 1, 1);
                break;
                default:
                assert(0);
            }
                      }
            }
                    else
                    {
              assert(node_id >= 0 && node_id < int(ni_cids.size()) );
              if(dram_sim) { //no need to call Connect if MC is not in this LP
            //????????????????????????? todo: MCP use proper clock!!
            switch(m_sysBuilder->get_cache_builder()->get_type()) {
                case CacheBuilder::MCP_CACHE:
                case CacheBuilder::MCP_L1L2:
                Manifold :: Connect(mc_cid, Controller::PORT0, &Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
                            ni_cids[node_id], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT,
                            &GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
                            *(dram_sim->get_clock()), Clock::Master(), 1, 1);
                break;
                default:
                assert(0);
            }
              }
                    }

        }//for
        }
        break;
        default:
        assert(0);
        break;
    }//switch

}

void DramSim_builder :: set_mc_map_obj(manifold::uarch::DestMap *mc_map)
{
    for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it) {
        int node_id = (*it).first;
        Dram_sim* mc = manifold::kernel::Component :: GetComponent<Dram_sim>(m_mc_id_cid_map[node_id]);

        if (mc)
            mc->set_mc_map(mc_map);
    }
}

void DramSim_builder :: print_config(std::ostream& out)
{
    MemControllerBuilder :: print_config(out);
    out << "  MC type: DRAMSim2\n";
    out << "  device file: " << m_DEV_FILE << "\n"
        << "  system file: " << m_SYS_FILE << "\n"
    << "  size: " << m_MEM_SIZE << "\n";
    out << "  clock: ";
    if(m_use_default_clock)
        out << "default\n";
    else {
        for(unsigned i=0; i<m_CLOCK_FREQ.size()-1; i++)
        out << m_CLOCK_FREQ[i] << ", ";
    out << m_CLOCK_FREQ[m_CLOCK_FREQ.size()-1] << "\n";
    }
}


void DramSim_builder :: print_stats(std::ostream& out)
{
    for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it) {
        int cid = (*it).second;
    Dram_sim* mc = Component :: GetComponent<Dram_sim>(cid);
    if(mc)
        mc->print_stats(out);
    }
}

//####################################################################
// HMC
//####################################################################


void HMC_builder :: read_config(Config& config)
{
    try
    {
    	Setting& hmc_links = config.lookup("xbar.node_idx");
    	m_NUM_SERDES = hmc_links.getLength();
    	m_NUM_VAULTS = config.lookup("mc.num_vaults");
    	m_NUM_MC = m_NUM_VAULTS; // Just to be consistent with other types of memory controllers.
    	                         // This value is not used for HMC.
    	m_NUM_XBAR_MC_PORTS = m_NUM_VAULTS / m_NUM_SERDES;

    	// The xbar will be m_NUM_SERDES \times n_NUM_VAULTS
		try
		{
			Setting& mc_clocks = config.lookup("mc.clocks");
			assert((unsigned)mc_clocks.getLength() == m_NUM_VAULTS);
			m_CLOCK_FREQ.resize(m_NUM_VAULTS);

			for(unsigned i=0; i<m_NUM_VAULTS; i++)
				m_CLOCK_FREQ[i] = (double)mc_clocks[i];

			m_use_default_clock = false;
		}

		catch (SettingNotFoundException e)
		{
			//mc clock not defined; use default
			m_use_default_clock = true;
		}

		try
		{
			Setting& xbar_clocks = config.lookup("xbar.clocks");
			assert((unsigned)xbar_clocks.getLength() == m_NUM_SERDES);
			xbar_CLOCK_FREQ.resize(m_NUM_SERDES);

			for(unsigned i=0; i<m_NUM_SERDES; i++)
				xbar_CLOCK_FREQ[i] = (double)xbar_clocks[i];

			xbar_use_default_clock = false;
		}

		catch (SettingNotFoundException e)
		{
			//xbar clock not defined; use default
			xbar_use_default_clock = true;
		}

		m_MC_DOWNSTREAM_CREDITS = config.lookup("mc.downstream_credits");

		m_MEM_MSG_TYPE = config.lookup("network.mem_msg_type");
		m_CREDIT_MSG_TYPE = config.lookup("network.credit_msg_type");

		//libconfig++ cannot assigned to string directly
		const char* chars = config.lookup("mc.vault.dev_file");
		m_DEV_FILE = chars;
		chars = config.lookup("mc.vault.sys_file");
		m_SYS_FILE = chars;
		m_VAULT_SIZE = config.lookup("mc.vault.size");
		m_MEM_SIZE = m_NUM_VAULTS * m_VAULT_SIZE;
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

void HMC_builder :: create_mcs(map<int, int>& id_lp)
{
	// First create m_NUM_VAULTS DRAMSim2 instances

    //create clocks if necessary
    if(m_use_default_clock == false)
    {
        if(m_CLOCK_FREQ.size() != 1 && m_CLOCK_FREQ.size() != m_NUM_VAULTS)
        {
        	cerr << "Wrong number of clocks for HMC; requires 1 or " << m_NUM_VAULTS << endl;
        }
        m_clocks.resize(m_NUM_VAULTS);
        for(unsigned i=0; i<m_NUM_VAULTS; i++)
        {
        	m_clocks[i] = new manifold::kernel::Clock(m_CLOCK_FREQ[i]);
        }
    }

    Dram_sim :: Set_msg_types(m_MEM_MSG_TYPE, m_CREDIT_MSG_TYPE);

    Dram_sim_settings settings(m_DEV_FILE.c_str(), m_SYS_FILE.c_str(), m_VAULT_SIZE, false, m_MC_DOWNSTREAM_CREDITS);

    Clock* clock = 0;
    Clock* xbar_clock = 0;
    if(m_use_default_clock)
    {
    	clock = m_sysBuilder->get_default_clock();
    	xbar_clock = m_sysBuilder->get_default_clock();
    }
    else if(m_NUM_VAULTS == 1)
    {
    	clock = m_clocks[0];
    	xbar_clock = xbar_clocks[0];
    }

    int i=0;
    int node_id1 = 0;
    for(map<int,int>::iterator it = id_lp.begin(); it != id_lp.end(); ++it)
    {
        int node_id = (*it).first;
        int lp = (*it).second;
        node_id1 = node_id;
        if(!m_use_default_clock && m_NUM_VAULTS > 1)
        	clock = m_clocks[i++];
        int cid = Component :: Create<Dram_sim>(lp, node_id, settings, *clock);
        m_mc_id_cid_map[node_id] = cid;
    }

    // Now create_xbar (This is still not implemented for distributed simulation)
    int num_xbar = 0;
    num_xbar = create_xbar(m_NUM_XBAR_MC_PORTS, node_id1+1); //fixme: Need to figure out what node_id1 should be
    assert(num_xbar == m_NUM_SERDES);
}

int HMC_builder :: create_xbar(int xbar_ports,int node_id) //fixme: Need to figure out what node_id1 should be
{
	//create clocks if necessary
	if(xbar_use_default_clock == false)
	{
		if(xbar_CLOCK_FREQ.size() != 1 && xbar_CLOCK_FREQ.size() != m_NUM_SERDES)
		{
			cerr << "Wrong number of clocks for xbar; requires 1 or " << m_NUM_SERDES << endl;
		}
		xbar_clocks.resize(m_NUM_SERDES);
		for(unsigned i=0; i<m_NUM_SERDES; i++)
		{
			xbar_clocks[i] = new manifold::kernel::Clock(xbar_CLOCK_FREQ[i]);
		}
	}

	Clock* xbar_clock = 0;
	if(xbar_use_default_clock)
	{
		xbar_clock = m_sysBuilder->get_default_clock();
	}
	else if(m_NUM_SERDES == 1)
	{
		xbar_clock = xbar_clocks[0];
	}

	int i;
	for(i = 0; i < m_NUM_SERDES; i++)
	{
		int cid = Component :: Create<hmcxbar>(node_id, *xbar_clock, m_NUM_XBAR_MC_PORTS); // fixme: need to decide what are the passed variables
		m_xbar_id_cid_map[node_id] = cid;
	}
	// Return num_xbars on successful creation of xbar
	return i;
}

//TODO: RAO Need to make changes here.
void HMC_builder :: connect_mc_network(NetworkBuilder* net_builder)
{
	//First connect xbar to Network.
    switch(net_builder->get_type())
    {
        case NetworkBuilder::IRIS:
        {
			Iris_builder* irisBuilder = dynamic_cast<Iris_builder*>(net_builder);
			assert(irisBuilder != 0);

			const std::vector<CompId_t>& ni_cids = net_builder->get_interface_cid();
			// Iterate for all xbars first. Connect the ports to the network. Then connect the dramsim instances to the xbar
			for(map<int, int>::iterator it = m_xbar_id_cid_map.begin(); it != m_xbar_id_cid_map.end(); ++it)
			{
				int node_id = (*it).first;
				int xbar_cid = (*it).second;
				hmcxbar* hmc_xbar = Component :: GetComponent<hmcxbar>(xbar_cid);
				if (irisBuilder->get_topology() == "TORUS6P")
				{
					assert(0);
					break;
				}
				else
				{
					assert(node_id >= 0 && node_id < int(ni_cids.size()) );
					if(hmc_xbar)
					{
						//no need to call Connect if MC is not in this LP
						//????????????????????????? todo: MCP use proper clock!!
						switch(m_sysBuilder->get_cache_builder()->get_type())
						{
							case CacheBuilder::MCP_CACHE:
							case CacheBuilder::MCP_L1L2:
								 Manifold :: Connect(xbar_cid, hmcxbar::PORT_NET, &hmcxbar::handle_net<manifold::mcp_cache_namespace::Mem_msg>,
													   ni_cids[node_id], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT, &GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
														*(hmc_xbar->get_clock()), Clock::Master(), 1, 1); // fixme: Check the parameters passed
								break;
							default:
								assert(0);
						}
					}
				}
			}
        } // case: IRIS
		break;
		default:
			assert(0);
			break;
	}//switch

	//Now connect MCs to the xbar ports
	connect_xbar_mc();
}

void HMC_builder::connect_xbar_mc()
{
	map<int, int>::iterator it = m_mc_id_cid_map.begin();
	int node_id = (*it).first;
	int mc_cid = (*it).second;

	for(map<int, int>::iterator it_xbar = m_xbar_id_cid_map.begin(); it_xbar != m_xbar_id_cid_map.end(); ++it_xbar)
	{
		int node_id_xbar = (*it_xbar).first;
		int xbar_cid = (*it_xbar).second;

		hmcxbar* hmc_xbar = Component :: GetComponent<hmcxbar>(xbar_cid);

		for(int i = 0; i < m_NUM_XBAR_MC_PORTS; i++)
		{
			Dram_sim* dram_sim = Component :: GetComponent<Dram_sim>(mc_cid);

			assert(node_id_xbar >= 0 && node_id >= 0 && node_id_xbar < int(hmc_xbar->get_num_xbar_ports()) );
			if(hmc_xbar && dram_sim)
			{
				Manifold :: Connect(xbar_cid, hmcxbar::PORT_MC[i], &hmcxbar::handle_mc_incoming<manifold::mcp_cache_namespace::Mem_msg>,
									mc_cid, Controller::PORT0, &Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
												*(hmc_xbar->get_clock()), *(dram_sim->get_clock()), 1, 1); // fixme: Check the parameters passed

			}
			if(it == m_xbar_id_cid_map.end())
				break;
			else
			{
				it = it + 1;
				node_id = (*it).first;
				mc_cid = (*it).second;
			}
		} // Connect DRAM and xbar for loop
	} // xbar instances for loop

}


/*
			for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it)
			{
				int node_id = (*it).first;
				int mc_cid = (*it).second;
				Dram_sim* dram_sim = Component :: GetComponent<Dram_sim>(mc_cid);
				if (irisBuilder->get_topology() == "TORUS6P")
				{
					assert(node_id >= 0 && node_id < int(ni_cids.size())/2 );
					if(dram_sim)
					{
					  //no need to call Connect if MC is not in this LP
					  //????????????????????????? todo: MCP use proper clock!!
					  switch(m_sysBuilder->get_cache_builder()->get_type())
					  {
						  case CacheBuilder::MCP_CACHE:
						  case CacheBuilder::MCP_L1L2:
//							  Manifold :: Connect(mc_cid, Controller::PORT0, &Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
//											  ni_cids[node_id*2+1], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT, &GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
//											  *(dram_sim->get_clock()), Clock::Master(), 1, 1);
							  Manifold :: Connect(mc_cid, Controller::PORT0, &hmcxbar::handle_net<manifold::mcp_cache_namespace::Mem_msg>,
							  											  ni_cids[node_id*2+1], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT, &GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
							  											  *(dram_sim->get_clock()), Clock::Master(), 1, 1); // fixme: Check the parameters passed
							  break;
						  default:
							  assert(0);
					  }
					}
				}
                else
                {
                	assert(node_id >= 0 && node_id < int(ni_cids.size()) );
                	if(dram_sim)
                	{
                		//no need to call Connect if MC is not in this LP
                		//????????????????????????? todo: MCP use proper clock!!
                		switch(m_sysBuilder->get_cache_builder()->get_type())
                		{
                			case CacheBuilder::MCP_CACHE:
                			case CacheBuilder::MCP_L1L2:
//                				Manifold :: Connect(mc_cid, Controller::PORT0,
//                								&Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
//                								ni_cids[node_id], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT,
//                								&GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
//                								*(dram_sim->get_clock()), Clock::Master(), 1, 1);
  							    Manifold :: Connect(mc_cid, Controller::PORT0, &hmcxbar::handle_net<manifold::mcp_cache_namespace::Mem_msg>,
  							  											  ni_cids[node_id], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT, &GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
  							  											  *(dram_sim->get_clock()), Clock::Master(), 1, 1); // fixme: Check the parameters passed
                				break;
                			default:
                				assert(0);
                		}
                	}
				}

			}//for


*/



void HMC_builder :: set_mc_map_obj(manifold::uarch::DestMap *mc_map)
{
    for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it) {
        int node_id = (*it).first;
        Dram_sim* mc = manifold::kernel::Component :: GetComponent<Dram_sim>(m_mc_id_cid_map[node_id]);

        if (mc)
            mc->set_mc_map(mc_map);
    }
}

void HMC_builder :: print_config(std::ostream& out)
{
    out << "  MC type: HMC\n";
    out << "  num of vaults : " << m_NUM_MC << endl;
    out << "  device file: " << m_DEV_FILE << "\n"
        << "  system file: " << m_SYS_FILE << "\n"
    << "  size: " << m_MEM_SIZE << "vault size: " << m_VAULT_SIZE << "\n";
    out << "  mc_clock: ";
    if(m_use_default_clock)
        out << "default\n";
    else
    {
        for(unsigned i=0; i<m_CLOCK_FREQ.size()-1; i++)
        out << m_CLOCK_FREQ[i] << ", ";
    out << m_CLOCK_FREQ[m_CLOCK_FREQ.size()-1] << "\n";
    }

    out << "  xbar_clock: ";
	if(xbar_use_default_clock)
		out << "default\n";
	else
	{
		for(unsigned i=0; i<xbar_CLOCK_FREQ.size()-1; i++)
		out << xbar_CLOCK_FREQ[i] << ", ";
	out << xbar_CLOCK_FREQ[xbar_CLOCK_FREQ.size()-1] << "\n";
	}
}


void HMC_builder :: print_stats(std::ostream& out)
{
    for(map<int, int>::iterator it = m_mc_id_cid_map.begin(); it != m_mc_id_cid_map.end(); ++it)
    {
        int cid = (*it).second;
        Dram_sim* mc = Component :: GetComponent<Dram_sim>(cid);
        if(mc)
        	mc->print_stats(out);
    }

    for(map<int, int>::iterator it = m_xbar_id_cid_map.begin(); it != m_xbar_id_cid_map.end(); ++it)
    {
		int cid = (*it).second;
		hmcxbar* xbar = Component :: GetComponent<hmcxbar>(cid);
		if(xbar)
			xbar->print_stats(out);
	}
}

