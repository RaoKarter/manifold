#ifndef MANIFOLD_PKT_GENERATOR_H
#define MANIFOLD_PKT_GENERATOR_H
#include "kernel/component-decl.h"
#include "uarch/DestMap.h"
#include "kernel/clock.h"
#include "uarch/networkPacket.h"
#include "uarch/memMsg.h"
#include "mcp-cache/coh_mem_req.h"
#include "mcp-cache/cache_req.h"
#include <list>
#include <algorithm>

namespace manifold {
namespace pkt_generator{

struct gen_settings {
    gen_settings(int num_bits, int num_cpu_links, int credits, int hmcs, int total_pkts, int req_flit_size, int resp_flit_size) :
        cache_offset_bits(num_bits), num_cpus(num_cpu_links), upstream_credits(credits), num_hmcs(hmcs), total_pkts(total_pkts),
        req_flit_size(req_flit_size), resp_flit_size(resp_flit_size)
    {}

    int cache_offset_bits; // 32B = 5 bits or 64B = 6 bits
    int num_cpus; // These are the number of CPUs connected to the HMC xbar
    int upstream_credits;
    int total_pkts;
    int num_hmcs;
    int req_flit_size;
    int resp_flit_size;
};

class PKT_gen : public manifold::kernel::Component {
public:
    enum { MAX_CPU_PORTS=4,
         };

    PKT_gen(int id, const gen_settings& generator_settings, manifold::kernel::Clock&);
    ~PKT_gen()
    {
        delete upstream_credits;
//        delete pkt_addresses;
        delete num_coh_req_pkts_sent;
        delete num_coh_resp_pkts_sent;
        delete num_coh_req_pkts_rcvd;
        delete num_coh_resp_pkts_rcvd;
        delete num_mem_read_req_pkts_sent;
        delete num_mem_write_req_pkts_sent;
        delete num_mem_read_req_pkts_rcvd;
        delete num_outgoing_credits;
        delete num_incoming_credits;
        delete intermediate_outgoing_credits;
        delete intermediate_incoming_credits;
    }

	int get_id() { return pkt_gen_id; }

    int get_cpu_id(int i) const { return pkt_gen_cpu[i]; }

	void set_hmc_map(manifold::uarch::DestMap *m);

	// This is the function where routing of the packets to different vaults takes place
	void tick();
	template<typename T> void handle_hmc_incoming(int, manifold::uarch::NetworkPacket* pkt);

    void process_incoming_pkts(manifold::uarch::NetworkPacket* pkt, int port);

    void send_credit_downstream(int port);

    void clear_bw_counters(int port);

    static void Set_msg_types(int mem, int credit, int coh) // Set some interface parameters
	{
		assert(Msg_type_set == false);
        COH_MSG_TYPE = coh;
		MEM_MSG_TYPE = mem;
		CREDIT_MSG_TYPE = credit;
		Msg_type_set = true;
	}

	void print_stats(ostream&);
private:
    static int COH_MSG_TYPE;
	static int MEM_MSG_TYPE;
	static int CREDIT_MSG_TYPE;
	static bool Msg_type_set;

    int pkt_gen_cpu[MAX_CPU_PORTS];     // Number of CPUs connected to the HMC xbar
	manifold::kernel::Clock& pkt_gen_clk;

	int pkt_gen_id;
    int* upstream_credits;     // NI credits
	manifold::uarch::DestMap *pktgen_map;
    int req_flit_size;
    int resp_flit_size;

    int num_cpu_ports;
    unsigned num_select_bits;
    unsigned num_HMC;
    unsigned PKTS_COUNTER;
    unsigned MAX_TEST_PKTS;

	//stats
    unsigned* num_coh_req_pkts_sent;
    unsigned* num_coh_resp_pkts_sent;
    unsigned* num_coh_req_pkts_rcvd;
    unsigned* num_coh_resp_pkts_rcvd;
    unsigned* num_mem_read_req_pkts_sent;
    unsigned* num_mem_write_req_pkts_sent;
    unsigned* num_mem_read_req_pkts_rcvd;
    unsigned* num_outgoing_credits;
    unsigned* num_incoming_credits;

    unsigned* intermediate_incoming_credits;
    unsigned* intermediate_outgoing_credits;
};

//! handle vault responses; type T is the message type from HMC xbar uarch::Mem_msg
template<typename T>
void PKT_gen :: handle_hmc_incoming(int in_port, manifold::uarch::NetworkPacket* pkt)
{
	if (pkt->type == CREDIT_MSG_TYPE)
	{
        upstream_credits[in_port]++;
		delete pkt;
        num_incoming_credits[in_port]++;
        intermediate_incoming_credits[in_port]++;
        cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen\t" << (this->get_id() * num_cpu_ports) + in_port << "\trcvd HMC CREDIT pkt\t" << dec << in_port
                    << "\tPKTupstream credits[" << in_port << "]\t" << upstream_credits[in_port]-1 << "->" << upstream_credits[in_port] << endl;
		return;
	}
    else if (pkt->type == MEM_MSG_TYPE)
        process_incoming_pkts(pkt, in_port);
    else
    {
        cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id BAD MSG TYPE ERROR!!!!!" << endl;
        assert(0);
    }

	delete pkt;
	return;
}



} //pkt_generator namespace
} //namespace manifold
#endif
