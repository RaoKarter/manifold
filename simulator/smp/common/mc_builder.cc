#include "mc_builder.h"
#include "cache_builder.h"
#include "mcp-cache/coh_mem_req.h"
#include "CaffDRAM/McMap.h"
#ifdef HUTDEBUG
#include "sysBuilder_llp_HUT.h"
#else
#include "sysBuilder_llp.h"
#endif

using namespace libconfig;
using namespace manifold::kernel;
using namespace manifold::uarch;
using namespace manifold::mcp_cache_namespace;
using namespace manifold::caffdram;
using namespace manifold::dramsim;
using namespace manifold::iris;
using namespace manifold::hmc_xbar;

#ifdef HUTDEBUG
using namespace manifold::pkt_generator;
#endif

void MemControllerBuilder :: print_config(std::ostream& out)
{
    out << "Memory controller:\n";
    out << "  num of nodes: " << m_NUM_MC << endl;

}


#ifndef HUTDEBUG

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

        for(unsigned i=0; i<m_NUM_MC; i++)
        m_CLOCK_FREQ[i] = (double)mc_clocks[i];

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
                Manifold :: Connect(mc_cid, Controller::PORT0,
                                    &Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
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
                Manifold :: Connect(mc_cid, Controller::PORT0,
                                    &Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
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

#endif //HUTDEBUG

//####################################################################
// HMC
//####################################################################

/*
 * read_config()
 * INPUT			Config& 		: 	This is the pointer to the
 * 										config file passed to the
 * 										simulator
 * 		This function reads various configuration parameters passed to
 * 		the simulator and stores it in different class variables. For
 * 		the HMC_builder, there are multiple parameters that are explained
 * 		below, only ones pertinent to HMC are explained
 *
 * 	private:
 *		unsigned m_VAULT_SIZE; //size of each vault
 *		unsigned m_MEM_SIZE; //mem size = m_NUM_VAULTS x m_VAULT_SIZE
 *		unsigned m_NUM_VAULTS; //number of vaults in each HMC
 *		unsigned m_NUM_SERDES; //number of SerDes links to each HMC
 *		unsigned m_NUM_HMCs; // number of HMCs
 *		std::vector<int> vault_node_idx_vec; //vector that contains vault ids
 *											 //used only to decide the DRAM address map
 *		std::set<int> vault_node_idx_set; //set is used to ensure each index is unique
 *
 *		void create_drams(int lp, int vault_node_id, int hmc_cid, manifold::uarch::PageBasedMap *mc_map);
 *		//function to create drams from within create_mcs();
 *
 *	protected:
 *		bool hmc_use_default_clock;
 *		std::vector<double> hmc_CLOCK_FREQ;
 *		std::vector<manifold::kernel::Clock*> hmc_clocks; //clock for the HMC xbar
 *		std::map<int, int> m_hmc_id_cid_map; //hmc id to cid map
 *		std::map<int,int>* m_vault_id_cid_map; //vault id to cid map
 */

void HMC_builder :: read_config(Config& config)
{
    try
    {
    	int max_vaults = config.lookup("max_vaults");
    	int max_serdes = config.lookup("max_serdes");
    	Setting& hmc_links = config.lookup("mc.xbar.node_idx");
    	int num_hmc_links = hmc_links.getLength();
    	m_NUM_SERDES = config.lookup("mc.xbar.num_serdes");

    	// Check if number of SerDes links to the network is good
    	assert( (m_NUM_SERDES > 0) && (m_NUM_SERDES <= max_serdes) && ((num_hmc_links % m_NUM_SERDES) == 0) );

    	// Num of SerDes links must be 2 or 4
    	assert(m_NUM_SERDES == 2 || m_NUM_SERDES == 4);

    	// Assumption is that each HMC has same number of SerDes links
    	m_NUM_HMCs = num_hmc_links / m_NUM_SERDES;

    	// Support 1, 2, 4 or 8 HMCs
    	assert((m_NUM_HMCs == 1) || (m_NUM_HMCs == 2) || (m_NUM_HMCs == 4) || (m_NUM_HMCs == 8));

    	// Create an array of std::maps to map component ids of DRAMs
    	// to the node ids *in each HMC*.
    	// Eg: m_vault_id_cid_map[HMC_ID_NUMBER][key] = Mapped Var
    	m_vault_id_cid_map = new std::map<int,int>[m_NUM_HMCs];

    	//vault assignment
    	m_NUM_VAULTS = config.lookup("mc.num_vaults"); // number of vaults per HMC

    	// Num of vaults must be 16 or 32 as per HMC2.0 spec
   		assert(m_NUM_VAULTS >=1 && ( (m_NUM_VAULTS == max_vaults) || (m_NUM_VAULTS == (max_vaults/2) ) ) );

   		m_NUM_MC = m_NUM_VAULTS * m_NUM_HMCs; // Because each vault has a mem controller

   		// Creating a vector of length m_NUM_VAULTS
   		this->vault_node_idx_vec.resize(m_NUM_VAULTS);

		for(int i=0; i < m_NUM_VAULTS; i++)
		{
			vault_node_idx_set.insert(i);
			this->vault_node_idx_vec[i] = i;
		}
		assert(vault_node_idx_set.size() == (unsigned)m_NUM_VAULTS); //verify no 2 indices are the same

		/*
		 * Create clocks for all the DRAMSim vaults
		 * Eg:			m_NUM_VAULTS = 32
		 * 				sizeof(m_CLOCK_FREQ) = 32
		 */

		try
		{
			Setting& mc_clocks = config.lookup("mc.clocks");
			assert((unsigned)mc_clocks.getLength() == m_NUM_VAULTS );
			m_CLOCK_FREQ.resize(m_NUM_VAULTS);

			for(unsigned i=0; i< m_NUM_VAULTS; i++)
				m_CLOCK_FREQ[i] = (double)mc_clocks[i];

			m_use_default_clock = false;
		}

		catch (SettingNotFoundException e)
		{
			//mc clock not defined; use default
			m_use_default_clock = true;
		}

		/*
		 * Create clocks for all the HMC xbars instances
		 * Eg:			m_NUM_HMCs = 2
		 * 				sizeof(hmc_CLOCK_FREQ) = 2 i.e 2 clocks must be created
		 * 											each HMC xbar will have different clock
		 * 											but same clock for all network links
		 */

		try
		{
			Setting& HMC_clocks = config.lookup("xbar.clocks");
			assert((unsigned)HMC_clocks.getLength() == m_NUM_HMCs);
			hmc_CLOCK_FREQ.resize(m_NUM_HMCs);

			for(unsigned i=0; i<m_NUM_HMCs; i++)
				hmc_CLOCK_FREQ[i] = (double)HMC_clocks[i];

			hmc_use_default_clock = false;
		}
		catch (SettingNotFoundException e)
		{
			// HMC xbar clocks not defined; use default
			hmc_use_default_clock = true;
		}

		// Downstream and upstream credits are separated for HMC xbars and the DRAMs
		m_HMC_DOWNSTREAM_CREDITS = config.lookup("mc.xbar.downstream_credits");
		m_HMC_UPSTREAM_CREDITS = config.lookup("mc.xbar.upstream_credits");
		m_MC_DOWNSTREAM_CREDITS = config.lookup("mc.downstream_credits");

		m_MEM_MSG_TYPE = config.lookup("network.mem_msg_type");
		m_CREDIT_MSG_TYPE = config.lookup("network.credit_msg_type");

		//libconfig++ cannot assigned to string directly
		const char* chars = config.lookup("mc.vault.dev_file");
		m_DEV_FILE = chars;
		chars = config.lookup("mc.vault.sys_file");
		m_SYS_FILE = chars;
		m_VAULT_SIZE = config.lookup("mc.vault.size");
		m_MEM_SIZE = m_NUM_VAULTS * m_VAULT_SIZE * m_NUM_HMCs;
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

/*
 * create_mcs()
 * INPUT			map<int, int>& id_lp		= 	id_lp contains the mapping between
 * 													logical process and the hmc instance
 *
 * 					Iterations over id_lp is used to create hmc instances. This function
 * 					also calls create_drams() which creates m_NUM_VAULTS * m_NUM_HMCs
 * 					number of DRAMSim instances and connects m_NUM_VAULTS to each HMC
 *
 * 	OUTPUT
 * 					NONE
 */

void HMC_builder :: create_mcs(map<int, int>& id_lp)
{
	// First create xbars
    // create clocks if necessary
	if(hmc_use_default_clock == false)
	{
		if(hmc_CLOCK_FREQ.size() != 1 && hmc_CLOCK_FREQ.size() != (m_NUM_HMCs) )
		{
			cerr << "Wrong number of clocks for HMC; requires 1 or " << (m_NUM_HMCs) << endl;
		}
		hmc_clocks.resize(m_NUM_HMCs);
		for(unsigned i=0; i < m_NUM_HMCs; i++)
		{
			hmc_clocks[i] = new manifold::kernel::Clock(hmc_CLOCK_FREQ[i]);
		}
	}

	HMC_xbar :: Set_msg_types(m_MEM_MSG_TYPE, m_CREDIT_MSG_TYPE);

	HMC_xbar_settings hmc_settings(m_NUM_SERDES, m_NUM_VAULTS, m_HMC_DOWNSTREAM_CREDITS, m_HMC_UPSTREAM_CREDITS);

	Clock* xbar_clock = 0;

	/*
	 * Set up vault_map here based on m_NUM_VAULTS. The size of vault_node_idx_vec decides
	 * the number of bits in the address dedicated to vault addressing.
	 *
	 * Eg:			PageBasedMap(Param1, Param2)
	 * 				Param1		=	length(vault_node_idx_vec) = 16
	 * 				Param2		=	0 -> These are PageOffsetBits
	 * 		The constructor of PageBasedMap will now set m_selector_bits = 4 and
	 * 		m_selector_mask = 0xf. This means, the lower 4 bits of address which comes to
	 * 		the HMC xbar will be removed when the address is sent to the DRAM as a request.
	 * 		However, based on the vault_id, these bits will be appended back to the address
	 * 		during a response.
	 *
	 */
	manifold::uarch::PageBasedMap *vault_map = new manifold::uarch::PageBasedMap(vault_node_idx_vec, 0);

#ifdef HMCDEBUG
	cout << "Vault map object address " << vault_map << endl;
	cout << "Vault map node size " << vault_map->get_nodes_size() << endl;
	cout << "Vault map selector bits " << dec << vault_map->get_selector_bits() << endl;
	cout << "Vault map selector mask " << hex << vault_map->get_selector_mask() << dec << endl;
#endif

	hmc_settings.s_vault_map = vault_map;

	/*
	 * Iterate over all the mc_node_idx_vec i.e. over all the network nodes
	 * connected to the HMC components. Increment iterator by m_NUM_SERDES nodes
	 */

	int cid[m_NUM_HMCs];
	hmc_settings.id_lp_map = &id_lp;
	map<int,int>::iterator it = id_lp.begin();
	int node_id = (*it).first;
	int lp = (*it).second;
	/*
	 * hmc_settings at this point will contain number of network links and number of
	 * DRAM links connected to each HMC xbar component. By passing the id_lp and 'i', the Create
	 * function for HMC_xbar will internally figure out how to assign network node ids to the
	 * different network ports of the HMC xbar.
	 *
	 * Eg:			ld_lp = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
	 * 				i	  = 1 => HMC1 i.e. 2nd HMC
	 * 		Therefore, network node_ids 8, 9, 10 and 11 will be mapped to net_ports 0, 1, 2 and 3
	 * 		of HMC1.
	 */
	for(int i = 0; i < m_NUM_HMCs; i++)
	{
		if(hmc_use_default_clock)
		{
			xbar_clock = m_sysBuilder->get_default_clock();
			cid[i] = Component :: Create<HMC_xbar>(lp, i, hmc_settings, *xbar_clock);
		}
		else
		{
			xbar_clock = hmc_clocks[i];
			cid[i] = Component :: Create<HMC_xbar>(lp, i, hmc_settings, *xbar_clock);
		}
		m_hmc_id_cid_map[node_id] = cid[i];
#ifdef HMCDEBUG
		cout << "Created HMC: "<< i << " with cid " << cid[i] << endl;
#endif

		node_id += m_NUM_SERDES; // Assuming HMCs are connected to sequential nodes
	}
#ifdef HUTDEBUG
	int pkt_cid[m_NUM_SERDES * m_NUM_HMCs];
	Clock* pkt_gen_clk = m_sysBuilder->get_default_clock();
	int pkt_gen_credits = 10;
	cout << "Creating pkt_generators." << endl;
	PKT_gen :: Set_msg_types(m_MEM_MSG_TYPE, m_CREDIT_MSG_TYPE);
	int num_HMC_bits = HMCLog2(m_NUM_HMCs);

	for(int i = 0; i < m_NUM_SERDES * m_NUM_HMCs; i++)
	{
		// Create the pkt_generators
		pkt_cid[i] = Component :: Create<PKT_gen>(lp, i, *pkt_gen_clk, pkt_gen_credits, num_HMC_bits);
		m_pkt_gen_id_cid_map[i] = pkt_cid[i];
	}
	cout << "Created " << m_NUM_SERDES * m_NUM_HMCs << " pkt generators." << endl;
#endif

#ifdef HMCDEBUG
	cout << "Done creating HMCs. Going to create vaults" << endl;
#endif
	create_drams(lp, cid, vault_map);

#ifdef HMCDEBUG
	cout << "DRAMs created" << endl;
#endif
}

/*
 * create_drams()
 * INPUTS
 * 						lp 				= logical process
 * 						hmc_cid			= pointer to array containing hmc component ids
 * 						mc_map			= structure pointer to a memory mapping scheme
 * 				This function instantiates all the DRAMSims. While doing so
 * 				it associates m_NUM_VAULTS number of DRAMSims to each HMCxbar so that
 * 				each DRAMSim will know which network port it has to send the pkt to.
 *
 * OUTPUT
 * 						NONE
 *
 */
void HMC_builder :: create_drams(int lp, int* hmc_cid, manifold::uarch::PageBasedMap *mc_map)
{
	Clock* dram_clock = 0;
	Dram_sim* vault;
	int vault_cid = 0;
	//create clocks for vaults if necessary
	if(m_use_default_clock == false)
	{
		if(m_CLOCK_FREQ.size() != 1 && m_CLOCK_FREQ.size() != m_NUM_VAULTS)
		{
			cerr << "Wrong number of clocks for vaults; requires 1 or " << m_NUM_VAULTS << endl;
		}

		m_clocks.resize(m_NUM_VAULTS);
		for(unsigned i=0; i<m_NUM_VAULTS; i++)
		{
			m_clocks[i] = new manifold::kernel::Clock(m_CLOCK_FREQ[i]);
		}
	}

	Dram_sim :: Set_msg_types(m_MEM_MSG_TYPE, m_CREDIT_MSG_TYPE);

	Dram_sim_settings vault_settings(m_DEV_FILE.c_str(), m_SYS_FILE.c_str(), m_VAULT_SIZE, false, m_MC_DOWNSTREAM_CREDITS);

	for (int i = 0; i < m_NUM_HMCs; i++)
	{
		HMC_xbar* hmcxbar = Component :: GetComponent<HMC_xbar>(hmc_cid[i]);
#ifdef HMCDEBUG
		cout << "HMC: " << hmcxbar->get_hmc_id() << " cid: " << hmc_cid[i] << endl;
#endif

		for (int j = 0; j < m_NUM_VAULTS; j++)
		{
			if(m_use_default_clock)
			{
				dram_clock = m_sysBuilder->get_default_clock();
				vault_cid = Component :: Create<Dram_sim>(lp, j, vault_settings, *dram_clock);
			}
			else
			{
				dram_clock = m_clocks[j];
				vault_cid = Component :: Create<Dram_sim>(lp, j, vault_settings, *dram_clock);
			}
			m_vault_id_cid_map[i][j] = vault_cid;
#ifdef HMCDEBUG
			cout << "Created DRAM " << j << endl;
#endif

			vault = Component :: GetComponent<Dram_sim>(vault_cid);
#ifdef HMCDEBUG
			cout << "Connecting HMC: " << hmc_cid[i] <<" with hmc_mem_port " << hmcxbar->get_mem_port(j) << " with DRAM cid " << vault_cid << " PORT0" << endl;
#endif
			Manifold :: Connect(hmc_cid[i], hmcxbar->get_mem_port(j), &HMC_xbar::handle_mc_incoming<manifold::uarch::Mem_msg>,
								vault_cid, Controller::PORT0, &Dram_sim::handle_incoming<manifold::mcp_cache_namespace::Mem_msg>,
								*(hmcxbar->get_clock()), *(vault->get_clock()), 1, 1);

#ifdef HMCDEBUG
			cout << "Dram sim @ " << vault << " vault map " << mc_map << endl;
#endif
			vault->set_mc_map(mc_map);
#ifdef HMCDEBUG
			cout << "Connected HMC " << dec << hmc_cid[i] << " with DRAM " << dec << j << endl;
#endif
		}
	}

	return;
}

#ifdef HUTDEBUG
void HMC_builder :: connect_mc_network()
{
	int j = 0;
	PKT_gen* pkt_gen = NULL;
	for(map<int, int>::iterator it = m_hmc_id_cid_map.begin(); it != m_hmc_id_cid_map.end(); ++it)
	{
		int hmc_node_id = (*it).first;
		int hmc_cid = (*it).second;
		HMC_xbar* hmcxbar = Component :: GetComponent<HMC_xbar>(hmc_cid);
		cout << "Connecting HMC to pkt_generator. HMC: " << dec << hmc_node_id << " with cid: " << dec << hmc_cid << endl;
		if (hmcxbar)
		{
			for (int i = 0; i < hmcxbar->get_num_net_ports(); i++)
			{
				assert(j <= m_NUM_SERDES * m_NUM_HMCs);
				pkt_gen = Component :: GetComponent<PKT_gen>(m_pkt_gen_id_cid_map[j]);
				cout << "Connecting HMC: " << dec << hmc_cid <<" with hmc_net_port " << dec << hmcxbar->get_net_port(i) << " to pkt_generator cid " << dec << m_pkt_gen_id_cid_map[j] << " PORT0" << endl;
				Manifold :: Connect(hmc_cid, hmcxbar->get_net_port(i), &HMC_xbar::handle_net<manifold::mcp_cache_namespace::Mem_msg>,
							m_pkt_gen_id_cid_map[j], Controller::PORT0, &PKT_gen::handle_hmc_incoming<manifold::uarch::Mem_msg>,
							*(hmcxbar->get_clock()), *(pkt_gen->get_clock()), 1, 1);
				j++;
			}
		}
	}
}

#else

/*
 * connect_mc_network()
 * INPUT
 * 							NetworkBuilder* net_builder		= network object
 *
 * 				This function will connect the xbar to the network ports and also connect
 * 				the DRAMSim instances to the xbars. The network topology must be TORUS.
 * 				The PORT_NET port of the xbar connects to the TERMINAL_PORT of IRIS.
 *
 * 	OUTPUT
 * 							NONE
 */
void HMC_builder :: connect_mc_network(NetworkBuilder* net_builder)
{
	//Connect HMC xbar to Network.
    switch(net_builder->get_type())
    {
        case NetworkBuilder::IRIS:
        {
			Iris_builder* irisBuilder = dynamic_cast<Iris_builder*>(net_builder);
			assert(irisBuilder != 0);

			const std::vector<CompId_t>& ni_cids = net_builder->get_interface_cid();
			// Iterate over all HMC ports. Connect the ports to the network. Dramsims instances are already connected
			// to the HMC xbars
			for(map<int, int>::iterator it = m_hmc_id_cid_map.begin(); it != m_hmc_id_cid_map.end(); ++it)
			{
				int hmc_node_id = (*it).first;
				int hmc_cid = (*it).second;
				HMC_xbar* hmcxbar = Component :: GetComponent<HMC_xbar>(hmc_cid);
#ifdef HMCDEBUG
				cout << "Connecting HMC to network. HMC: " << dec << hmc_node_id << " with cid: " << dec << hmc_cid << endl;
#endif
				if (irisBuilder->get_topology() == "TORUS6P")
				{
					// For HMC the IRIS network should be TORUS
					assert(0);
					break;
				}
				else
				{
					assert(hmc_node_id >= 0 && hmc_node_id < int(ni_cids.size()) );
					if(hmcxbar)
					{
						//no need to call Connect if MC is not in this LP
						//????????????????????????? todo: MCP use proper clock!!
						switch(m_sysBuilder->get_cache_builder()->get_type())
						{
							case CacheBuilder::MCP_CACHE:
							case CacheBuilder::MCP_L1L2:
								for (int i = 0; i < hmcxbar->get_num_net_ports(); i++)
								{
#ifdef HMCDEBUG
									cout << "Connecting HMC: " << dec << hmc_cid <<" with hmc_net_port " << dec << hmcxbar->get_net_port(i) << " with network cid " << dec << ni_cids[hmc_node_id + i] << " TERMINAL PORT" << endl;
#endif
									Manifold :: Connect(hmc_cid, hmcxbar->get_net_port(i), &HMC_xbar::handle_net<manifold::mcp_cache_namespace::Mem_msg>,
													    ni_cids[hmc_node_id + i], GenNetworkInterface<NetworkPacket>::TERMINAL_PORT, &GenNetworkInterface<NetworkPacket>::handle_new_packet_event,
														*(hmcxbar->get_clock()), Clock::Master(), 1, 1);
								}
								break;
							default:
								assert(0);
						}
					}
				}// if TORUS
			}// HMC for loop
        } // case: IRIS
		break;
		default:
			assert(0);
			break;
	}//switch
}

#endif

void HMC_builder :: set_mc_map_obj(manifold::uarch::DestMap *hmc_map)
{
#ifdef HMCDEBUG
	cout << "HMC builder set_mc_map_obj " << hmc_map << endl;
#endif
    for(map<int, int>::iterator it = m_hmc_id_cid_map.begin(); it != m_hmc_id_cid_map.end(); ++it)
    {
        int node_id = (*it).first;
#ifdef HMCDEBUG
        cout << "set_mc_map_obj node_id=" << node_id << endl;
#endif
        HMC_xbar* hmc = manifold::kernel::Component :: GetComponent<HMC_xbar>(m_hmc_id_cid_map[node_id]);

        if (hmc)
            hmc->set_hmc_map(hmc_map);
    }
#ifdef HUTDEBUG
    for (map<int,int>::iterator it = m_pkt_gen_id_cid_map.begin(); it != m_pkt_gen_id_cid_map.end(); ++it)
    {
    	int cid = (*it).second;
    	PKT_gen* pkt_gen = manifold::kernel::Component :: GetComponent<PKT_gen>(cid);

		if (pkt_gen)
			pkt_gen->set_hmc_map(hmc_map);
    }
#endif
}

void HMC_builder :: print_config(std::ostream& out)
{
    out << "  MC type: HMC\n";
    out << "  num of HMCs : " << dec <<  m_NUM_HMCs << endl;
    out << "  num of SerDes links : " << dec << m_NUM_SERDES << endl;
    out << "  device file : " << m_DEV_FILE << "\n"
        << "  system file : " << m_SYS_FILE << "\n"
    << " Total Mem Size(MB): " << dec << m_MEM_SIZE << "\n Vault Size(MB): " << dec << m_VAULT_SIZE << "\n";
    out << "  vault_clock: ";
    if(m_use_default_clock)
        out << "default\n";
    else
    {
        for(unsigned i=0; i<m_CLOCK_FREQ.size()-1; i++)
        out << dec << m_CLOCK_FREQ[i] << ", ";
    out << m_CLOCK_FREQ[m_CLOCK_FREQ.size()-1] << "\n";
    }

    out << "  HMC_xbar_clock: ";
	if(hmc_use_default_clock)
		out << "default\n";
	else
	{
		for(unsigned i=0; i<hmc_CLOCK_FREQ.size()-1; i++)
		out << dec << hmc_CLOCK_FREQ[i] << ", ";
	out << hmc_CLOCK_FREQ[hmc_CLOCK_FREQ.size()-1] << "\n";
	}
}

void HMC_builder :: print_stats(std::ostream& out)
{
	for(map<int, int>::iterator it = m_hmc_id_cid_map.begin(); it != m_hmc_id_cid_map.end(); ++it)
	{
		int cid = (*it).second;
		HMC_xbar* xbar = Component :: GetComponent<HMC_xbar>(cid);
		if(xbar)
			xbar->print_stats(out);
	}

	for(int i = 0; i < m_NUM_HMCs; i++)
	{
		for(map<int, int>::iterator it = m_vault_id_cid_map[i].begin(); it != m_vault_id_cid_map[i].end(); ++it)
		{
			int cid = (*it).second;
			Dram_sim* vault = Component :: GetComponent<Dram_sim>(cid);
			if(vault)
				vault->print_stats(out);
		}
	}
#ifdef HUTDEBUG

	for(map<int, int>::iterator it = m_pkt_gen_id_cid_map.begin(); it != m_pkt_gen_id_cid_map.end(); ++it)
	{
		int cid = (*it).second;
		PKT_gen* Gen = Component :: GetComponent<PKT_gen>(cid);
		if(Gen)
			Gen->print_stats(out);
	}
#endif
}
