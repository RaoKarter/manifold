#include "pkt_generator.h"
#include "kernel/component.h"
#include "kernel/manifold.h"
#include <stdlib.h>
#define ADDR_MAX 0x7fffffffffff

using namespace manifold::kernel;
using namespace manifold::mcp_cache_namespace;


namespace manifold {
namespace pkt_generator {

int PKT_gen :: MEM_MSG_TYPE = -1;
int PKT_gen :: CREDIT_MSG_TYPE = -1;
bool PKT_gen :: Msg_type_set = false;

PKT_gen :: PKT_gen(int id, Clock& clk, int &credits, int &num_bits) :
		pkt_gen_id(id) , pkt_gen_clk(clk), upstream_credits(credits), num_select_bits(num_bits)
{
	cerr << " pkt_gen @ " << this << endl;
	assert(Msg_type_set);
	assert(MEM_MSG_TYPE != CREDIT_MSG_TYPE);

	pktgen_map = NULL;

	//register with clock
	Clock :: Register(clk, this, &PKT_gen::tick, (void(PKT_gen::*)(void)) 0 );

	num_pkts_sent = 0;
	num_pkts_rcvd = 0;
	num_outgoing_credits = 0;
	num_incoming_credits = 0;
	PKTS_COUNTER = 0;
}

void PKT_gen :: tick()
{
	if (PKTS_COUNTER >= 100)
		return;
	for (int i = 0; i < 1; i++)
	{

	//	uint64_t send_addr = 0xfffffffc0 + i;
		uint64_t send_addr = 0;
		send_addr = rand() % ADDR_MAX; // pow(2,36)
		cerr << hex << "pkt_gen_map->lookup(" << send_addr << ") = " << pktgen_map->lookup(send_addr) << dec << endl;
		int HMC_add = this->get_id() / 4;
		switch(HMC_add)
		{
		case 0: // HMC 0
			{
				send_addr = (send_addr << num_select_bits);
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		case 1: // HMC 1
			{
				send_addr = (send_addr << num_select_bits) | 1;
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		case 2: // HMC 2
			{
				send_addr = (send_addr << num_select_bits) | 2;
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		case 3: // HMC 3
			{
				send_addr = (send_addr << num_select_bits) | 3;
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		case 4: // HMC 4
			{
				send_addr = (send_addr << num_select_bits) | 4;
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		case 5: // HMC 5
			{
				send_addr = (send_addr << num_select_bits) | 5;
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		case 6: // HMC 6
			{
				send_addr = (send_addr << num_select_bits) | 6;
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		case 7: // HMC 7
			{
				send_addr = (send_addr << num_select_bits) | 7;
				send_addr = (send_addr << 5);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
				break;
			}
		default:
			{
				assert(0);
			}
		}

		manifold::mcp_cache_namespace::Mem_msg req;
		req.addr = send_addr;
		req.op_type = OpMemLd; //Mem LD

		// Choosing the pkt source randomly
		int r_src = rand() % 4; // Source id depends on the number of CPUs.
		req.src_id = r_src;
		req.dst_id = this->get_id() + 4; // Change 4 to the starting number in HMC_Unit_Test.cfg file

		manifold::uarch::NetworkPacket *pkt = new manifold::uarch::NetworkPacket;
		pkt->set_type(MEM_MSG_TYPE);
		pkt->set_src(r_src);
		pkt->set_src_port(rand() % 10);
		pkt->set_dst(this->get_id() + 4);  // The pkt_gen is connected to the HMC node with same node id
		pkt->set_dst_port(rand() % 10);
	
		*((manifold::mcp_cache_namespace::Mem_msg*)(pkt->data)) = req;
		pkt->data_size = sizeof(manifold::mcp_cache_namespace::Mem_msg);
	
		if (upstream_credits > 0)
		{
			Send(PORT0, pkt);
			upstream_credits--;
			num_pkts_sent++;
			cerr << dec << "@\t" << m_clk->NowTicks() <<"\t(tick)PKTupstream credits[" << this->get_id() << "]\t" << upstream_credits+1 << "->"
						<< upstream_credits << endl;
			cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << this->get_id()
						<< "\tREQUEST pkt_addr\t" << hex << req.get_addr() << "\tsrc_id\t"
						<< dec << pkt->get_src() << "\tdst_id\t" << pkt->get_dst()
						<< "\tpkt_src_port\t" << pkt->get_src_port()
						<< "\tpkt_dst_port\t" << pkt->get_dst_port() << endl;
		}
			PKTS_COUNTER++;
	}
}

void PKT_gen :: send_credit_downstream()
{
	manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
	credit_pkt->type = CREDIT_MSG_TYPE;
	Send(PORT0, credit_pkt);
	cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen\t" << this->get_id() << "\tsending CREDIT pkt\t" << dec << endl;
	num_outgoing_credits++;
}

void PKT_gen :: set_hmc_map(manifold::uarch::DestMap *m)
{
	cout << "PKT_gen @\t" << this << "\tset_mc_map\t" << m << endl;
	this->pktgen_map = m;
}


void PKT_gen :: print_stats(ostream& out)
{
	out << "****** PKT_GEN " << pkt_gen_id << "********* stats:" << endl
	<< "  incoming msg: " << num_pkts_rcvd << endl
	<< "  outgoing msg: " << num_pkts_sent << endl
	<< "  num outgoing credits: " << num_outgoing_credits << endl
	<< "  num incoming credits: " << num_incoming_credits << endl;
}


} // namespace pkt_generator
} //namespace manifold

