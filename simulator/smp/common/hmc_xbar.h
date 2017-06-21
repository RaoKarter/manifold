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

#ifdef HMCXBAR
#include "mcp-cache/coh_mem_req.h"
#include "mcp-cache/cache_req.h"
#endif

#ifdef LIBKITFOX
#include "uarch/kitfoxCounter.h"
#endif

namespace manifold {
namespace hmc_xbar{

//! The HMC needs a network interface. This class is a xbar that connects
//! HMC vaults to the network via a SerDes link

struct HMC_xbar_settings {
    HMC_xbar_settings(int num_net_links, int num_dram_links, int d_credits, int u_credits, int req_flit_size, int resp_flit_size, int trans_size) :
        num_hmc_net_links(num_net_links), num_mem_links(num_dram_links), downstream_credits(d_credits), upstream_credits(u_credits),
        req_flit_size(req_flit_size), resp_flit_size(resp_flit_size), vault_trans_size(trans_size)
    {}

	int num_hmc_net_links; // These are the number of serdes links per HMC
    int num_mem_links; // These are the number of DRAM instances per HMC
    int downstream_credits;
    int upstream_credits;
    int req_flit_size;
    int resp_flit_size;
    int vault_trans_size;
    std::map<int, int>* id_lp_map;
    manifold::uarch::PageBasedMap *s_vault_map;
};

class HMC_xbar : public manifold::kernel::Component {
public:
    enum {
    	MAX_NET_PORT = 4,
        MAX_MEM_PORT = 32,
        PORT_KITFOX = 100,
    };

    HMC_xbar(int id, const HMC_xbar_settings& hmcxbar_settings, manifold::kernel::Clock&);
    ~HMC_xbar()
    {
        delete downstream_credits;     // NI credits
        delete upstream_credits;		// Vault side credits
        delete hmc_map;
        delete hmcxbar_vault_map;
        delete hmc_to_serdes_buffer;
        delete hmc_to_mc_buffer;
        delete hmc_to_serdes_buffer_msg_type;

        delete serdes_rx_dep_time;
        delete vault_rx_dep_time;
#ifdef HUTDEBUG
        delete serdes_rx_cur_clk_tick;
        delete vault_rx_cur_clk_tick;
        delete serdes_rx_pkt_type;
        delete vault_rx_pkt_type;
        delete serdes_rx_addr;
        delete vault_rx_addr;
#endif /* HUTDEBUG */

        delete stats_num_incoming_vault_msg;
        delete stats_num_outgoing_vault_msg;
        delete stats_num_incoming_vault_credits;
        delete stats_num_outgoing_vault_credits;
        delete intermediate_num_incoming_vault_credits;
        delete intermediate_num_outgoing_vault_credits;

        delete stats_num_incoming_read_req_serdes_msg;
        delete stats_num_incoming_write_req_serdes_msg;
        delete stats_num_outgoing_read_resp_serdes_msg;

        delete stats_num_incoming_serdes_credits;
        delete stats_num_outgoing_serdes_credits;
        delete intermediate_num_incoming_serdes_credits;
        delete intermediate_num_outgoing_serdes_credits;

#ifdef HMCXBAR
        delete stats_num_incoming_coh_req_serdes_msg;
        delete stats_num_incoming_coh_resp_serdes_msg;
        delete stats_num_outgoing_coh_req_serdes_msg;
        delete stats_num_outgoing_coh_resp_serdes_msg;
#endif
    }

    int get_hmc_id() { return hmc_id; }

    int get_mem_port(int i) const { return m_ports[i];   }

    int get_net_port(int i) const { return net_ports[i]; }

    int get_num_mc_ports() { return hmc_num_mc_ports; }

    int get_num_net_ports() { return hmc_num_net_ports; }

    void set_hmc_map(manifold::uarch::DestMap *m);

    // This is the function where routing of the packets to different vaults takes place
    void tick();
    // Handler for incoming requests from the SerDes
    template<typename T> void handle_serdes_incoming(int, manifold::uarch::NetworkPacket* pkt);
    // Handler for incoming requests from the Vault mem controllers
    template<typename T> void handle_vault_incoming(int, manifold::uarch::NetworkPacket* pkt);

    // These functions handle memory messages received from the cores
    // They also handle memory responses from the vaults
    // Internally they call the queue_mem_adter_xbar_delay_serdes
    // and queue_after_xbar_delay_vault
    void handle_serdes_mem_msg(manifold::uarch::NetworkPacket* pkt, int port, int vault);
    void handle_vault_responses(manifold::uarch::NetworkPacket* pkt, int serdes_port, int vault);
    void queue_mem_after_xbar_delay_serdes(manifold::uarch::NetworkPacket* pkt, int in_port, int vault, int credit_type);
    void queue_after_xbar_delay_vault(manifold::uarch::NetworkPacket* pkt, int in_port, int serdes_port, unsigned int credit_type);

#ifdef HMCXBAR
    void handle_serdes_coh_msg(manifold::uarch::NetworkPacket* pkt, int in_port, int dst);
    void queue_coh_after_xbar_delay_serdes(manifold::uarch::NetworkPacket *pkt, int in_port, int dst, unsigned int type);
#endif

    uint64_t serdes_rx_calculate_delay(int from_port, int to_port, int type);
    uint64_t vault_rx_calculate_delay(int from_port, int to_port, int type);

    void send_credit_downstream(int);
    void send_credit_upstream(int, int);

#ifdef FORECAST_NULL
    void do_output_to_net_prediction();
#endif

    static void Set_msg_types(int mem, int credit, int coh) // Set some interface parameters
	{
		assert(Msg_type_set == false);
		MEM_MSG_TYPE = mem;
		CREDIT_MSG_TYPE = credit;
        COH_MSG_TYPE = coh;
		Msg_type_set = true;
	}

    void print_stats(ostream&);
    void clear_xbar_counters();

#ifdef LIBKITFOX
    template<typename T>
    void handle_kitfox_proxy_request(int temp, T *kitfox_proxy_request);
#endif

protected:
#ifdef FORECAST_NULL
    //overwrite base class
    void remote_input_notify(manifold::kernel::Ticks_t, void* data, int port);
#endif

    // NEED BUFFERS FOR EACH VAULT AND EACH NETWORK PORT
    std::list<manifold::uarch::NetworkPacket*>* hmc_to_serdes_buffer; //store responses from all connected vaults
    std::list<manifold::uarch::NetworkPacket*>* hmc_to_mc_buffer; //store requests from network for each vault
    std::list<unsigned int>* hmc_to_serdes_buffer_msg_type; // store response msg_type from all connected vaults

    std::list<uint64_t>* serdes_rx_dep_time;
    std::list<uint64_t>* vault_rx_dep_time;

#ifdef HUTDEBUG
    std::list<uint64_t>* serdes_rx_cur_clk_tick;
    std::list<uint64_t>* vault_rx_cur_clk_tick;
    std::list<uint64_t>* serdes_rx_addr;
    std::list<uint64_t>* vault_rx_addr;
    std::list<uint64_t>* serdes_rx_pkt_type;
    std::list<uint64_t>* vault_rx_pkt_type;
#endif /* HUTDEBUG */

private:
    static int MEM_MSG_TYPE;
	static int CREDIT_MSG_TYPE;
    static int COH_MSG_TYPE;
	static bool Msg_type_set;

    int m_ports[MAX_MEM_PORT];        // Depending on the size of the HMC each xbar will have sizeof(m_ports) number of vaults connected to it
    int net_ports[MAX_NET_PORT];        // Depending on the SerDes link width of the HMC each xbar will have sizeof(net_ports) number of network ports

    manifold::kernel::Clock& xbar_clk;

    int hmc_num_mc_ports;
    int hmc_num_net_ports;
    int hmc_id;
    int* downstream_credits;     // NI credits
    int* upstream_credits;		// Vault side credits
    manifold::uarch::DestMap *hmc_map;
    manifold::uarch::PageBasedMap *hmcxbar_vault_map;
    std::map<int,int>::iterator hmc_net_it;
    std::map<string,int> hmc_nid_portid_map;
    std::vector<int> vault_node_idx_vec;
    int req_flit_size;
    int resp_flit_size;
    int MAX_NET_CREDITS;
    int MAX_VAULT_CREDITS;
    double time;
    int vault_transaction_size;

    #ifdef FORECAST_NULL
    //std::list<manifold::kernel::Ticks_t> m_output_ticks;
    std::list<manifold::kernel::Ticks_t> m_input_msg_ticks;
    #endif

    //stats
    // incoming from
    // outgoing to
    unsigned* stats_num_incoming_vault_msg;
    unsigned* stats_num_outgoing_vault_msg;
    unsigned* stats_num_incoming_vault_credits;
    unsigned* stats_num_outgoing_vault_credits;
    unsigned* intermediate_num_incoming_vault_credits;
    unsigned* intermediate_num_outgoing_vault_credits;

    unsigned* stats_num_incoming_read_req_serdes_msg;
    unsigned* stats_num_incoming_write_req_serdes_msg;
    unsigned* stats_num_outgoing_read_resp_serdes_msg;

    unsigned* stats_num_incoming_serdes_credits;
    unsigned* stats_num_outgoing_serdes_credits;
    unsigned* intermediate_num_incoming_serdes_credits;
    unsigned* intermediate_num_outgoing_serdes_credits;

#ifdef HMCXBAR
    unsigned* stats_num_incoming_coh_req_serdes_msg;
    unsigned* stats_num_incoming_coh_resp_serdes_msg;
    unsigned* stats_num_outgoing_coh_req_serdes_msg;
    unsigned* stats_num_outgoing_coh_resp_serdes_msg;
#endif

#ifdef LIBKITFOX
    manifold::uarch::hmcxbar_counter_t counter;
#endif

};

//! handle vault responses; type T is the message type from vault memory controller: uarch::Mem_msg
template<typename T>
void HMC_xbar :: handle_vault_incoming(int in_port, manifold::uarch::NetworkPacket* pkt)
{
	// Logic to check if incoming message is from port assigned to mem controller
	int * p;
	p = std::find(m_ports, m_ports + get_num_mc_ports(), in_port);
	assert(*p == in_port);

	if (pkt->type == CREDIT_MSG_TYPE)
	{
		upstream_credits[in_port - hmc_num_net_ports]++;
        assert(upstream_credits[in_port - hmc_num_net_ports] <= MAX_VAULT_CREDITS);
        stats_num_incoming_vault_credits[in_port - hmc_num_net_ports]++;
        intermediate_num_incoming_vault_credits[in_port - hmc_num_net_ports]++;
#ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\trcvd MC CREDIT pkt @ port\t" << dec  << in_port
				<<"\tupstream credits[" << in_port - hmc_num_net_ports << "]\t" << upstream_credits[in_port - hmc_num_net_ports]-1
				<< "->" << upstream_credits[in_port - hmc_num_net_ports] << endl;
#endif
		delete pkt;
		return;
	}

    stats_num_incoming_vault_msg[in_port - hmc_num_net_ports]++;

	// Modify the pkt address field from local address to global address
	// i.e. append HMC address bits to the lower end of the pkt address
	T* req = (T*)pkt->data;
    assert(req->type == 1);                 // Check if req is indeed a RPLY
    uint64_t pkt_laddr = req->get_addr();   // Local address now
	assert(hmc_map);
	uint64_t pkt_gaddr = hmc_map->get_global_addr(pkt_laddr, this->get_hmc_id());
	req->addr = pkt_gaddr;

#ifdef HMCDEBUG
    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\t(B)hmc_id\t" << this->get_hmc_id() << "\trcvd MC MSG pkt @ port\t"
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
    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\t(A)hmc_id\t" << this->get_hmc_id() << "\trcvd MC MSG pkt @ port\t"
			<< dec << in_port << "\tsrc_id\t" << pkt->get_src() << "\tsrc_port\t" << pkt->get_src_port()
			<< "\tdst_id\t" << pkt->get_dst() << "\tdst_port\t" << pkt->get_dst_port() << "\tladdr\t"
			<< hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << endl;
#endif

    /*
     * This function models xbar delays and then queues the packet in the to_serdes buffer. Once it has queued
     * the memory response pkt, it sends back a credit
     */
    //handle_vault_responses(pkt, to_port,    from_port)
    handle_vault_responses(pkt, it->second, in_port - hmc_num_net_ports);
}

//! handle memory requests incoming from network; type T is Mem_msg type
template<typename T>
void HMC_xbar :: handle_serdes_incoming(int in_port, manifold::uarch::NetworkPacket* pkt)
{
    // Check if the pkt is coming from the network layer
    int *p;
    p = std::find(net_ports, net_ports + get_num_net_ports(), in_port);
    assert(*p == in_port);

    if (pkt->type == CREDIT_MSG_TYPE)
    {
        downstream_credits[in_port]++;
        stats_num_incoming_serdes_credits[in_port]++;
        if(pkt->get_dst_port() == 0)
            intermediate_num_incoming_serdes_credits[in_port]++;
        else
            intermediate_num_incoming_serdes_credits[in_port] += resp_flit_size;
        // TODO: Need to read in the upper bound on credits
        assert(downstream_credits[in_port] <= MAX_NET_CREDITS);

#ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\trcvd SERDES CREDIT pkt @ port\t" << dec << in_port
                <<"\tHMCdownstream credits[" << in_port << "]\t" << downstream_credits[in_port]-1 << "->" << downstream_credits[in_port] << endl;
#endif
        delete pkt;
        return;
    }

    if (pkt->type == MEM_MSG_TYPE)
    {
#ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\tHMC Received PKT TYPE is MEM MSG" << endl;
#endif
        // MEM_MSG_TYPE

        T* req = (T*)pkt->data;
        uint64_t pkt_gaddr = req->get_addr(); // Global address

        // get laddr. then figure out vault_id
        // Create pkt with address field where HMC address bits are stripped off

//        uint64_t lowbits = pkt_gaddr & 0x1f;
//        assert ( (lowbits ^ 0x0) == 0 );
        assert(hmc_map);
        uint64_t pkt_laddr = hmc_map->get_local_addr(pkt_gaddr);

        req->addr = pkt_laddr;

        int vault_id = 0;
        assert(hmcxbar_vault_map);
        vault_id = hmcxbar_vault_map->lookup(req->addr);

        // Check if local addr is within range of associated DRAMSims
        assert(vault_id >= 0 && vault_id <= hmc_num_mc_ports);

#ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\trcvd NET MEM MSG pkt @ port\t"
                << dec << in_port << "\tsrc_id\t" << pkt->get_src() << "\tpkt->src_port\t" << pkt->get_src_port()
                << "\tdst_id\t" << pkt->get_dst() << "\tpkt->dst_port\t" << pkt->get_dst_port() << "\tladdr\t"
                << hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << endl;
#endif
        //handle_serdes_mem_msg(pkt, from_port, to_port)
        handle_serdes_mem_msg(pkt, in_port, vault_id);
    }

#ifdef HMCXBAR
    else if (pkt->type == COH_MSG_TYPE)
    {
#ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\tHMC Received PKT TYPE is COH MSG" << endl;
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\trcvd NET COH MSG pkt @ port\t"
                << dec << in_port << "\tsrc_id\t" << pkt->get_src() << "\tpkt->src_port\t" << pkt->get_src_port()
                << "\tdst_id\t" << pkt->get_dst() << "\tpkt->dst_port\t" << pkt->get_dst_port() << endl;
#endif
        handle_serdes_coh_msg(pkt, in_port, pkt->get_dst());
    }
#endif
    else
    {
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\trcvd NET MSG pkt @ port\t"
             << dec << in_port << "\tBAD MSG ERROR\t" << endl;
        assert(0);
    }
}


#ifdef LIBKITFOX
template <typename T>
void HMC_xbar :: handle_kitfox_proxy_request(int temp, T *kitfox_proxy_request)
{
    assert(kitfox_proxy_request->get_type() == manifold::uarch::KitFoxType::hmcxbar_type);
    assert(kitfox_proxy_request->get_id() == this->get_hmc_id());

    double meas_time_period = 0;
    meas_time_period = (xbar_clk.NowTicks() * xbar_clk.period) - time;

    cerr << "@\t" << xbar_clk.NowTicks() << "\txbar_clk\txbar" << this->get_hmc_id() << " meas_time_period: " << meas_time_period << endl;
    for(int i = 0; i < hmc_num_net_ports; i++)
    {
        counter.xbar_power += intermediate_num_outgoing_serdes_credits[i];
        cerr << "\tSerDes[" << i <<"]\tin_credits\t" << intermediate_num_incoming_serdes_credits[i] << "\tout_credits\t" << intermediate_num_outgoing_serdes_credits[i];
        cerr << "\tBW\t" << (double) ( (intermediate_num_outgoing_serdes_credits[i] + intermediate_num_incoming_serdes_credits[i] ) * req_flit_size / meas_time_period) / (1024 * 1024 * 1024) << " GB/s" << endl;
    }
    cerr << "SerDes side credits= " << counter.xbar_power << endl;
    for(int i = 0; i < hmc_num_mc_ports; i++)
    {
        counter.xbar_power += intermediate_num_outgoing_vault_credits[i];
        cerr << "\tVault[" << i <<"]\tin_credits\t" << intermediate_num_incoming_vault_credits[i] << "\tout_credits\t" << intermediate_num_outgoing_vault_credits[i];
        cerr << "\tBW\t" << (double) ( (intermediate_num_outgoing_vault_credits[i] + intermediate_num_incoming_vault_credits[i] ) * vault_transaction_size / meas_time_period) / (1024 * 1024 * 1024) << " GB/s" << endl;
    }
    cerr << "SerDes + Vault side credits= " << counter.xbar_power;
    // Power = Num of FLITS * Bytes-per-FLIT * Bits-per-Byte * nJ-per-bit
    counter.xbar_power = (double) ( ( counter.xbar_power * 16 * 8 * 2.034e-12) / meas_time_period );
    cerr << " hmc_xbar_power= " << counter.xbar_power << endl;
    kitfox_proxy_request->set_xbar_power(counter.xbar_power);

    counter.clear();
    clear_xbar_counters();

    Send(PORT_KITFOX, kitfox_proxy_request);
    time = xbar_clk.NowTicks() * xbar_clk.period;
}
#endif // LIBKITFOX

} //hmc_xbar namespace
} //namespace manifold
#endif
