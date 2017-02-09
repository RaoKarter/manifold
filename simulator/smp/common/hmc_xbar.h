#ifndef MANIFOLD_HMC_XBAR_H
#define MANIFOLD_HMC_XBAR_H
#include "kernel/component-decl.h"
#include "kernel/clock.h"
#include "uarch/networkPacket.h"
#include <list>
#include <algorithm>

namespace manifold {
namespace xbar_namespace {

//! The HMC needs a network interface. This class is a xbar that connects
//! an HMC quadrant to the network via a SerDes link


class hmcxbar : public manifold::kernel::Component {
public:
    enum { PORT_NET=0 }; // Each link will have a xbar
    int *PORT_MC;        // Depending on the size of the HMC each xbar will have 'PORT_MC' number of vaults connected to it

    hmcxbar(int nid, manifold::kernel::Clock&, int credit_type, int num_mc_ports);
    ~hmcxbar() {};

    int get_nid() { return xbar_nid; }
    int get_num_xbar_ports() { return xbar_num_mc_ports; }
    unsigned inline hmc_log2(unsigned);

    // This is the function where routing of the packets to different vaults takes place
    void tick();
    // Handler for incoming requests from the network
    template<typename T> void handle_net(int, manifold::uarch::NetworkPacket* pkt);
    // Handler for incoming requests from the Vault mem controllers
    template<typename T> void handle_mc_incoming(int, manifold::uarch::NetworkPacket* pkt);

    void send_credit_downstream();

    #ifdef FORECAST_NULL
    void do_output_to_net_prediction();
    #endif

    void print_stats(ostream&);

protected:
    #ifdef FORECAST_NULL
    //overwrite base class
    void remote_input_notify(manifold::kernel::Ticks_t, void* data, int port);
    #endif

    // NEED BUFFERS FOR EACH QUADRANT
    std::list<manifold::uarch::NetworkPacket*> xbar_mc_requests; //store requests from all connected vaults
    std::list<manifold::uarch::NetworkPacket*>* xbar_net_requests; //store requests from network for each vault
private:
    const int CREDIT_MSG_TYPE;

    manifold::kernel::Clock& xbar_clk;

    int xbar_num_mc_ports;
    int xbar_nid;
    int downstream_credits;     // NI credits`
    #ifdef FORECAST_NULL
    //std::list<manifold::kernel::Ticks_t> m_output_ticks;
    std::list<manifold::kernel::Ticks_t> m_input_msg_ticks;
    #endif

    //stats
    unsigned stats_num_llp_incoming_msg;
    unsigned stats_num_lls_incoming_msg;
    unsigned stats_num_llp_outgoing_msg;
    unsigned stats_num_lls_outgoing_msg;
    unsigned stats_num_llp_incoming_credits;
    unsigned stats_num_lls_incoming_credits;
    unsigned stats_num_outgoing_credits;
	unsigned stats_n_reads;
	unsigned stats_n_writes;
	unsigned stats_n_reads_sent;
	uint64_t stats_totalMemLat;
	std::map<int, unsigned> stats_n_reads_per_source;
	std::map<int, unsigned> stats_n_writes_per_source;
};

unsigned inline hmcxbar::hmc_log2(unsigned value)
{
	unsigned logbase2 = 0;
	unsigned orig = value;
	value>>=1;
	while (value>0)
	{
		value >>= 1;
		logbase2++;
	}
	if ((unsigned)1<<logbase2<orig)logbase2++;
	return logbase2;
}

//! handle incoming from memory; type T is the message type from memory controller
template<typename T>
void hmcxbar :: handle_mc_incoming(int in_port, manifold::uarch::NetworkPacket* pkt)
{
	// Logic to check if incoming message is from port assigned to mem controller
	int * p;
	p = std::find(PORT_MC, PORT_MC + xbar_num_mc_ports, in_port);
	assert(*p == in_port);

	if (pkt->type == CREDIT_MSG_TYPE)
	{
		//fixme: Need to create variable for downstream credits
		downstream_credits++;
		delete pkt;
		return;
	}

    xbar_mc_requests.push_back(pkt);

}

//! handle memory requests incoming from network
template<typename T>
void hmcxbar :: handle_net(int in_port, manifold::uarch::NetworkPacket* pkt)
{
	// Check if the pkt is coming from the network layer
	assert(in_port == PORT_NET);

    if (pkt->type == CREDIT_MSG_TYPE)
    {
    	downstream_credits++;
		delete pkt;
		return;
    }

    T* req = (T*)(pkt->data);
    uint64_t pkt_addr = req->get_addr();
    uint64_t vault_mask = 0; // To extract the bottom n bits for vault mapping

    for(int i = 0; i < hmc_log2(xbar_num_mc_ports); i++)
	{
		vault_mask = (vault_mask << 1) | 1;
	}
    // ex: 32 vaults = 5 bits. log2(32) = 5 => vault_mask = 0x1F

    uint64_t vault_addr = vault_mask & pkt_addr;
#ifdef DBG_DRAMSIM
cout << "@ " << m_clk->NowTicks() << " >>> xbar " << get_nid() << " rcvd pkt_addr= " <<hex<< pkt_addr << " vault_addr= " <<hex<< vault_addr <<endl;
#endif

	assert(vault_addr > 0 && vault_addr < xbar_num_mc_ports);

	// This piece of code should change if DRAMSim parameters get changed.
	// fixme: hard coding needs to be fixed.

	// This is only for testing purposes. Delete this code later
	// DRAMSim ignores the lowest 3 bits. We are using the LSB to
	// determine vault address. Depending on the number of lower bits
	// used for vault address, the address of the pkt must be adjusted.
	// ex: 8 vaults => 3 LSBs used for vault address => No change
	//     16 vaults => 4 LSBs used for vault address => 1 bit right shift
	//     4 vaults => 2 LSBs used for vault address => 1 bit left shift
	switch(hmc_log2(xbar_num_mc_ports))
	{
		case	1:
		{
			req->addr <<= 2;
			break;
		}
		case	2:
		{
			req->addr <<= 1;
			break;
		}
		case	4:
		{
			req->addr >>= 1;
			break;
		}
		case	5:
		{
			req->addr >>= 2;
			break;
		}
		case	3:
		default: // nochange
			break;
	}

	xbar_net_requests[vault_addr].push_back(pkt);




    // Check if the address is among the vaults connected to the xbar
    //      if not queue into a buffer to send to another xbar
    //      else
    //      Based on the address, queue the packet into a buffer (one for each vault).





    if (req->is_read())
    {
		stats_n_reads++;
		stats_n_reads_per_source[pkt->get_src()]++;
#ifdef DBG_DRAMSIM
cout << "@ " << m_clk->NowTicks() << " >>> mc " << m_nid << " received LD, src= " << pkt->get_src() << " addr= " <<hex<< req->get_addr() <<dec<<endl;
#endif
    }
    else
    {
		stats_n_writes++;
		stats_n_writes_per_source[pkt->get_src()]++;
#ifdef DBG_DRAMSIM
cout << "@ " << m_clk->NowTicks() << " >>> mc " << m_nid << " received ST, src= " << pkt->get_src() << " addr= " <<hex<< req->get_addr() <<dec<<endl;
#endif
    }

    //paddr_t newAddr = m_mc_map->ripAddress(req->addr);

//    pkt->set_dst(pkt->get_src());
//    pkt->set_dst_port(pkt->get_src_port());
//    pkt->set_src(m_nid);
//    pkt->set_src_port(0);
//    pkt->type = 9;

//    assert(mc_map);
//    //put the request in the input buffer
//    m_incoming_reqs.push_back(Request(m_clk->NowTicks(), mc_map->get_local_addr(req->get_addr()), req->get_addr(), req->is_read(), pkt));

}

} // namespace xbar_namespace
} //namespace manifold
#endif
