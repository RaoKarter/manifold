#ifndef MANIFOLD_HMC_XBAR_H
#define MANIFOLD_HMC_XBAR_H
#include "kernel/component-decl.h"
#include "uarch/DestMap.h"
#include "kernel/clock.h"
#include "uarch/networkPacket.h"
#include "uarch/memMsg.h"
#include <list>
#include <algorithm>
#include <iterator>
#include <string>

#ifdef HMCDEBUG
#include "mcp-cache/coh_mem_req.h"
#include "mcp-cache/cache_req.h"
#endif

namespace manifold {
namespace hmc_xbar{

//! The HMC needs a network interface. This class is a xbar that connects
//! HMC vaults to the network via a SerDes link

struct HMC_xbar_settings {
	HMC_xbar_settings(int num_net_links, int num_dram_links, int d_credits, int u_credits) :
		num_hmc_net_links(num_net_links), num_mem_links(num_dram_links), downstream_credits(d_credits), upstream_credits(u_credits)
    {}

	int num_hmc_net_links; // These are the number of serdes links per HMC
    int num_mem_links; // These are the number of DRAM instances per HMC
    int downstream_credits;
    int upstream_credits;
    std::map<int, int>* id_lp_map;
    manifold::uarch::PageBasedMap *s_vault_map;
};


class HMC_xbar : public manifold::kernel::Component {
public:
    enum {
    	MAX_NET_PORT = 4,
        MAX_MEM_PORT = 32,
    };

    HMC_xbar(int id, const HMC_xbar_settings& hmcxbar_settings, manifold::kernel::Clock&);
    ~HMC_xbar() {};

    int get_hmc_id() { return hmc_id; }

    int get_mem_port(int i) const { return m_ports[i];   }

    int get_net_port(int i) const { return net_ports[i]; }

    int get_num_mc_ports() { return hmc_num_mc_ports; }

    int get_num_net_ports() { return hmc_num_net_ports; }

    void set_hmc_map(manifold::uarch::DestMap *m);

    // This is the function where routing of the packets to different vaults takes place
    void tick();
    // Handler for incoming requests from the network
    template<typename T> void handle_net(int, manifold::uarch::NetworkPacket* pkt);
    // Handler for incoming requests from the Vault mem controllers
    template<typename T> void handle_mc_incoming(int, manifold::uarch::NetworkPacket* pkt);

    void send_credit_downstream(int);
    void send_credit_upstream(int);

    #ifdef FORECAST_NULL
    void do_output_to_net_prediction();
    #endif

    static void Set_msg_types(int mem, int credit) // Set some interface parameters
	{
		assert(Msg_type_set == false);
		MEM_MSG_TYPE = mem;
		CREDIT_MSG_TYPE = credit;
		Msg_type_set = true;
	}

    void print_stats(ostream&);

protected:
    #ifdef FORECAST_NULL
    //overwrite base class
    void remote_input_notify(manifold::kernel::Ticks_t, void* data, int port);
    #endif

    // NEED BUFFERS FOR EACH VAULT AND EACH NETWORK PORT
    std::list<manifold::uarch::NetworkPacket*>* hmc_mc_responses; //store responses from all connected vaults
    std::list<manifold::uarch::NetworkPacket*>* hmc_net_requests; //store requests from network for each vault
private:
    static int MEM_MSG_TYPE;
	static int CREDIT_MSG_TYPE;
	static bool Msg_type_set;

    int m_ports[MAX_MEM_PORT];        // Depending on the size of the HMC each xbar will have sizeof(m_ports) number of vaults connected to it
    int net_ports[MAX_NET_PORT];        // Depending on the SerDes link width of the HMC each xbar will have sizeof(net_ports) number of network ports

    manifold::kernel::Clock& hmc_clk;

    int hmc_num_mc_ports;
    int hmc_num_net_ports;
    int hmc_id;
    int* downstream_credits;     // NI credits
    int* upstream_credits;		// Vault side credits
    manifold::uarch::DestMap *hmc_map;
    manifold::uarch::PageBasedMap *hmcxbar_vault_map;
    std::map<int,int>::iterator hmc_net_it;
//    std::map<int,int> hmc_nid_portid_map;
    std::map<string,int> hmc_nid_portid_map;
    std::vector<int> vault_node_idx_vec;

    #ifdef FORECAST_NULL
    //std::list<manifold::kernel::Ticks_t> m_output_ticks;
    std::list<manifold::kernel::Ticks_t> m_input_msg_ticks;
    #endif

    //stats
    unsigned stats_num_net_incoming_msg;
    unsigned stats_num_net_outgoing_msg;
    unsigned stats_num_net_incoming_credits;
    unsigned stats_num_net_outgoing_credits;
    unsigned stats_num_mc_incoming_msg;
    unsigned stats_num_mc_outgoing_msg;
    unsigned stats_num_mc_incoming_credits;
    unsigned stats_num_mc_outgoing_credits;
};

//! handle vault responses; type T is the message type from vault memory controller: uarch::Mem_msg
template<typename T>
void HMC_xbar :: handle_mc_incoming(int in_port, manifold::uarch::NetworkPacket* pkt)
{
	// Logic to check if incoming message is from port assigned to mem controller
	int * p;
	p = std::find(m_ports, m_ports + get_num_mc_ports(), in_port);
	assert(*p == in_port);

	/*
	 * When vault sends credit message what do I do?
	 */
	if (pkt->type == CREDIT_MSG_TYPE)
	{
		upstream_credits[in_port - hmc_num_net_ports]++;
		stats_num_mc_incoming_credits++;
#ifdef HMCDEBUG
		cerr << dec << "@\t" << m_clk->NowTicks() << "\thmc_id\t" << this->get_hmc_id() << "\trcvd MC CREDIT pkt @ port\t" << dec  << in_port
				<<"\tupstream credits[" << in_port - hmc_num_net_ports << "]\t" << upstream_credits[in_port - hmc_num_net_ports]-1
				<< "->" << upstream_credits[in_port - hmc_num_net_ports] << endl;
#endif
		delete pkt;
		return;
	}

	stats_num_mc_incoming_msg++;
	// Modify the pkt address field from local address to global address
	// i.e. append HMC address bits to the lower end of the pkt address
	T* req = (T*)pkt->data;
	uint64_t pkt_laddr = req->get_addr(); // Local address now
	assert(hmc_map);
	uint64_t pkt_gaddr = hmc_map->get_global_addr(pkt_laddr, this->get_hmc_id());
	req->addr = pkt_gaddr;

#ifdef HMCDEBUG
	cerr << dec << "@\t" << m_clk->NowTicks() << "\t(B)hmc_id\t" << this->get_hmc_id() << "\trcvd MC MSG pkt @ port\t"
			<< dec << in_port << "\tsrc_id\t" << pkt->get_src() << "\tsrc_port\t" << pkt->get_src_port()
			<< "\tdst_id\t" << pkt->get_dst() << "\tdst_port\t" << pkt->get_dst_port() << "\tladdr\t"
			<< hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << endl;
#endif

	string f = to_string(pkt->get_src());
	std::map<string,int>::iterator it = hmc_nid_portid_map.find(f);
	assert (it != hmc_nid_portid_map.end());

/*
	std::map<int,int>::iterator it = hmc_nid_portid_map.find(pkt->get_dst_port());
	assert (it != hmc_nid_portid_map.end());
	pkt->set_src(it->second);
	pkt->set_src_port(pkt->get_dst_port());
	// pkt->get_dst_port() has to be handled
*/

#ifdef HMCDEBUG
	cerr << dec << "@\t" << m_clk->NowTicks() << "\t(A)hmc_id\t" << this->get_hmc_id() << "\trcvd MC MSG pkt @ port\t"
			<< dec << in_port << "\tsrc_id\t" << pkt->get_src() << "\tsrc_port\t" << pkt->get_src_port()
			<< "\tdst_id\t" << pkt->get_dst() << "\tdst_port\t" << pkt->get_dst_port() << "\tladdr\t"
			<< hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << endl;
#endif
    hmc_mc_responses[it->second].push_back(pkt);
    send_credit_downstream(in_port - hmc_num_net_ports); //Send credits to vaults (upstream_credits)
}

//! handle memory requests incoming from network; type T is Mem_msg type
template<typename T>
void HMC_xbar :: handle_net(int in_port, manifold::uarch::NetworkPacket* pkt)
{
	// Check if the pkt is coming from the network layer
	int *p;
	p = std::find(net_ports, net_ports + get_num_net_ports(), in_port);
	assert(*p == in_port);

	if (pkt->type == CREDIT_MSG_TYPE)
    {
    	downstream_credits[in_port]++;
    	stats_num_net_incoming_credits++;
#ifdef HMCDEBUG
		cerr << dec << "@\t" << m_clk->NowTicks() << "\thmc_id\t" << this->get_hmc_id() << "\trcvd NET CREDIT pkt @ port\t" << dec << in_port
				<<"\tHMCdownstream credits[" << in_port << "]\t" << downstream_credits[in_port]-1 << "->" << downstream_credits[in_port] << endl;
#endif
    	delete pkt;
		return;
    }

	stats_num_net_incoming_msg++;
	T* req = (T*)pkt->data;
    uint64_t pkt_gaddr = req->get_addr(); // Global address


    // get laddr. then figure out vault_id
    // Create pkt with address field where HMC address bits are stripped off
    uint64_t lowbits = pkt_gaddr & 0x1f;
    assert ( (lowbits ^ 0x0) == 0 );
    uint64_t pkt_laddr = hmc_map->get_local_addr(pkt_gaddr);

    req->addr = pkt_laddr;

    int vault_id = 0;
	assert(hmcxbar_vault_map);
	vault_id = hmcxbar_vault_map->lookup(req->addr);
//	pkt->set_src_port(in_port);

	// Check if local addr is within range of associated DRAMSims
	assert(vault_id >= 0 && vault_id <= hmc_num_mc_ports);

	// Push the pkt to the FIFO queue for the appropriate DRAMSim Vault
//    pkt->set_dst_port(vault_id + hmc_num_net_ports);

#ifdef HMCDEBUG
	cerr << dec << "@\t" << m_clk->NowTicks() << "\thmc_id\t" << this->get_hmc_id() << "\trcvd NET MSG pkt @ port\t"
			<< dec << in_port << "\tsrc_id\t" << pkt->get_src() << "\tpkt->src_port\t" << pkt->get_src_port()
			<< "\tdst_id\t" << pkt->get_dst() << "\tpkt->dst_port\t" << pkt->get_dst_port() << "\tladdr\t"
			<< hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << endl;
#endif
	hmc_net_requests[vault_id].push_back(pkt);
	send_credit_upstream(in_port); // Send credits to Network (downstream_credits)
}

} //hmc_xbar namespace
} //namespace manifold
#endif
