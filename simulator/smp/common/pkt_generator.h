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

class PKT_gen : public manifold::kernel::Component {
public:
	enum { PORT0 };

	PKT_gen(int id, manifold::kernel::Clock&, int &credits, int &num_bits);
	~PKT_gen() {};

	int get_id() { return pkt_gen_id; }

	void set_hmc_map(manifold::uarch::DestMap *m);

	// This is the function where routing of the packets to different vaults takes place
	void tick();
	template<typename T> void handle_hmc_incoming(int, manifold::uarch::NetworkPacket* pkt);

	void send_credit_downstream();

	static void Set_msg_types(int mem, int credit) // Set some interface parameters
	{
		assert(Msg_type_set == false);
		MEM_MSG_TYPE = mem;
		CREDIT_MSG_TYPE = credit;
		Msg_type_set = true;
	}

	void print_stats(ostream&);
private:
	static int MEM_MSG_TYPE;
	static int CREDIT_MSG_TYPE;
	static bool Msg_type_set;

	manifold::kernel::Clock& pkt_gen_clk;

	int pkt_gen_id;
	int upstream_credits;     // NI credits
	manifold::uarch::DestMap *pktgen_map;

	//stats
	unsigned num_pkts_sent;
	unsigned num_pkts_rcvd;
	unsigned num_outgoing_credits;
	unsigned num_incoming_credits;
	unsigned num_select_bits;
	unsigned PKTS_COUNTER;
};

//! handle vault responses; type T is the message type from vault memory controller: uarch::Mem_msg
template<typename T>
void PKT_gen :: handle_hmc_incoming(int in_port, manifold::uarch::NetworkPacket* pkt)
{
	if (pkt->type == CREDIT_MSG_TYPE)
	{
		upstream_credits++;
		delete pkt;
		num_incoming_credits++;
		cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen\t" << this->get_id() << "\trcvd HMC CREDIT pkt\t" << dec << in_port
		    		<< "\tPKTupstream credits[" << this->get_id() << "]\t" << upstream_credits-1 << "->" << upstream_credits << endl;
		return;
	}

	T* req = (T*)pkt->data;

	uint64_t pkt_addr = req->get_addr(); //  Should be global address
	cerr << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << this->get_id() << "\tRESPONSE pkt_addr\t"
			<< hex << pkt_addr << "\tpkt_src\t" << dec << pkt->get_src() << "\tpkt_dst\t"<< pkt->get_dst()
			<< "\tpkt_src_port\t" << pkt->get_src_port()
			<< "\tpkt_dst_port\t" << pkt->get_dst_port() << endl;
	send_credit_downstream();
	num_pkts_rcvd++;
	delete pkt;
	return;
}



} //pkt_generator namespace
} //namespace manifold
#endif
