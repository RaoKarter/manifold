#include "pkt_generator.h"
#include "kernel/component.h"
#include "kernel/manifold.h"
#include <stdlib.h>
#define ADDR_MAX 0x7fffffffffff

using namespace manifold::kernel;
using namespace manifold::mcp_cache_namespace;
using namespace manifold::uarch;


namespace manifold {
namespace pkt_generator {

int PKT_gen :: COH_MSG_TYPE = -1;
int PKT_gen :: MEM_MSG_TYPE = -1;
int PKT_gen :: CREDIT_MSG_TYPE = -1;
bool PKT_gen :: Msg_type_set = false;

PKT_gen :: PKT_gen(int id, const gen_settings& generator_settings, Clock& clk) :
        pkt_gen_id(id) , pkt_gen_clk(clk)
{
	cerr << " pkt_gen @ " << this << endl;
	assert(Msg_type_set);
	assert(MEM_MSG_TYPE != CREDIT_MSG_TYPE);

    num_select_bits = generator_settings.cache_offset_bits;
    num_cpu_ports = generator_settings.num_cpus;
    num_HMC = generator_settings.num_hmcs;
    upstream_credits = new int [num_cpu_ports];
    MAX_TEST_PKTS = generator_settings.total_pkts;
    req_flit_size = generator_settings.req_flit_size;
    resp_flit_size = generator_settings.resp_flit_size;

    cerr << dec << "\n Number of Select bits\t" << num_select_bits << "\tnum_cpu_ports\t" << num_cpu_ports
            << " num HMCs " << num_HMC << endl;

	pktgen_map = NULL;

    //stats
#ifdef HMCXBAR
    num_coh_req_pkts_sent = new unsigned [num_cpu_ports];
    num_coh_resp_pkts_sent = new unsigned [num_cpu_ports];
    num_coh_req_pkts_rcvd = new unsigned [num_cpu_ports];
    num_coh_resp_pkts_rcvd = new unsigned [num_cpu_ports];
#endif
    num_mem_read_req_pkts_sent = new unsigned [num_cpu_ports];
    num_mem_write_req_pkts_sent = new unsigned [num_cpu_ports];
    num_mem_read_req_pkts_rcvd = new unsigned [num_cpu_ports];
    num_outgoing_credits = new unsigned [num_cpu_ports];
    num_incoming_credits = new unsigned [num_cpu_ports];

    intermediate_incoming_credits = new unsigned [num_cpu_ports];
    intermediate_outgoing_credits = new unsigned [num_cpu_ports];

    for (int i = 0; i < num_cpu_ports; i++)
    {
        pkt_gen_cpu[i] = i;
        upstream_credits[i] = generator_settings.upstream_credits;
#ifdef HMCXBAR
        num_coh_req_pkts_sent[i] = 0;
        num_coh_resp_pkts_sent[i] = 0;
        num_coh_req_pkts_rcvd[i] = 0;
        num_coh_resp_pkts_rcvd[i] = 0;
#endif
        num_mem_read_req_pkts_sent[i] = 0;
        num_mem_write_req_pkts_sent[i] = 0;
        num_mem_read_req_pkts_rcvd[i] = 0;
        num_outgoing_credits[i] = 0;
        num_incoming_credits[i] = 0;
        intermediate_incoming_credits[i] = 0;
        intermediate_outgoing_credits[i] = 0;
    }

	//register with clock
	Clock :: Register(clk, this, &PKT_gen::tick, (void(PKT_gen::*)(void)) 0 );

    PKTS_COUNTER = 0;
}

void PKT_gen :: tick()
{
    if (PKTS_COUNTER >= MAX_TEST_PKTS)
        return;

    for (int i = 0; i < num_cpu_ports; i++)
	{
		uint64_t send_addr = 0;
        send_addr = rand() % ADDR_MAX; // pow(2,36)

        int HMC_add = this->get_id();
        switch(HMC_add)
        {
        case 0: // HMC 0
            {
                send_addr = (send_addr << 0);
                send_addr = (send_addr << num_select_bits);
//              cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        case 1: // HMC 1
            {
                send_addr = (send_addr << 1) | 1;
                send_addr = (send_addr << num_select_bits);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        case 2: // HMC 2
            {
                send_addr = (send_addr << 2) | 2;
                send_addr = (send_addr << num_select_bits);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        case 3: // HMC 3
            {
                send_addr = (send_addr << 2) | 3;
                send_addr = (send_addr << num_select_bits);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        case 4: // HMC 4
            {
                send_addr = (send_addr << 3) | 4;
                send_addr = (send_addr << num_select_bits);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        case 5: // HMC 5
            {
                send_addr = (send_addr << 3) | 5;
                send_addr = (send_addr << num_select_bits);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        case 6: // HMC 6
            {
                send_addr = (send_addr << 3) | 6;
                send_addr = (send_addr << num_select_bits);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        case 7: // HMC 7
            {
                send_addr = (send_addr << 3) | 7;
                send_addr = (send_addr << num_select_bits);
//				cerr << hex << send_addr << dec << "\t" << HMC_add << endl;
                break;
            }
        default:
            {
                assert(0);
            }
        }

#ifdef HMCXBAR
        if (rand() % 10 > 5)
        {
            manifold::mcp_cache_namespace::Coh_msg message;
            if(rand() % 100 > 50)
                message.type = manifold::mcp_cache_namespace::Coh_msg::COH_REQ;
            else
                message.type = manifold::mcp_cache_namespace::Coh_msg::COH_RPLY;
            message.addr = send_addr;
            message.msg = rand() % 10;
            message.src_id = (this->get_id() * num_cpu_ports) + i; // Source id depends on the number of CPUs.
            message.dst_id = message.src_id;

            manifold::uarch::NetworkPacket *pkt = new manifold::uarch::NetworkPacket;
            pkt->set_type(COH_MSG_TYPE);
            pkt->set_src(message.src_id);
            pkt->set_src_port(rand() % 10);
            pkt->set_dst(message.dst_id);
            pkt->set_dst_port(rand() % 10);

            *((manifold::mcp_cache_namespace::Coh_msg*)(pkt->data)) = message;
            pkt->data_size = sizeof(manifold::mcp_cache_namespace::Coh_msg);

            if (upstream_credits[i] > 0)
            {
                Send(get_cpu_id(i), pkt);
                upstream_credits[i]--;
                if(message.type == manifold::mcp_cache_namespace::Coh_msg::COH_REQ)
                    num_coh_req_pkts_sent[i]++;
                else
                    num_coh_resp_pkts_sent[i]++;
                cerr << dec << "@\t" << m_clk->NowTicks() <<"\t(tick)PKTupstream credits["
                            << get_cpu_id(i) << "]\t" << upstream_credits[i] + 1 << "->"
                            << upstream_credits[i] << endl;
                cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + i
                            << "\tCOH REQUEST pkt_addr\t" << hex << message.addr << dec << "\tsrc_id\t"
                            << pkt->get_src() << "\tdst_id\t" << pkt->get_dst()
                            << "\tpkt_src_port\t" << pkt->get_src_port()
                            << "\tpkt_dst_port\t" << pkt->get_dst_port() << endl;
                cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + i
                            << "\tCOH msg type REQUEST\t" << (int)message.type << "\taddr\t" << hex
                            << message.addr << dec << "\tmsg\t" << message.msg << "\tsrc_id\t"
                            << message.src_id << "\tdst_id\t" << message.dst_id << endl;
                PKTS_COUNTER++;
            }
            else
            {
//                cerr << dec << "@\t" << m_clk->NowTicks() << "\tPKT_gen_id\t" << (this->get_id() * num_cpu_ports) + i
//                     << "\tPKT_GEN_SENDING_STOPPED" << endl;
            }
        }
        else
        {
#endif

        manifold::mcp_cache_namespace::Mem_msg req;
        if(rand() % 100 > 50)
            req.op_type = OpMemLd;
        else
            req.op_type = OpMemSt;
        req.addr = send_addr;
        req.type = manifold::mcp_cache_namespace::Mem_msg :: MEM_REQ;

        // Choosing the pkt source randomly
        req.src_id = (this->get_id() * num_cpu_ports) + i; // Source id depends on the number of network ports.
#ifdef HMCXBAR
        req.dst_id = req.src_id;
#else
        req.dst_id = (num_HMC * num_cpu_ports) + (this->get_id() * num_cpu_ports) + i; // Change num_cpu_ports to the starting number in HMC_Unit_Test.cfg file
#endif
//        req.dst_id = pktgen_map->lookup(send_addr);

        manifold::uarch::NetworkPacket *pkt = new manifold::uarch::NetworkPacket;
        pkt->set_type(MEM_MSG_TYPE);
        pkt->set_src(req.src_id);
        pkt->set_src_port(rand() % 10);
        pkt->set_dst(req.dst_id);  // The pkt_gen is connected to the HMC node with same node id
        pkt->set_dst_port(rand() % 10);

        *((manifold::mcp_cache_namespace::Mem_msg*)(pkt->data)) = req;
        pkt->data_size = sizeof(manifold::mcp_cache_namespace::Mem_msg);

        if (upstream_credits[i] > 0)   // Enough credits to send both LD and ST
        {
            Send(get_cpu_id(i), pkt);
            upstream_credits[i]--;
            assert(upstream_credits[i] >= 0);
            cerr << dec << "@\t" << m_clk->NowTicks() <<"\t(tick)PKTupstream credits[" << get_cpu_id(i) << "]\t" << upstream_credits[i] + 1 << "->"
                        << upstream_credits[i] << endl;
            if (req.op_type == OpMemLd)
            {
                num_mem_read_req_pkts_sent[i]++;
                cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + i
                        << "\tMEM READ REQUEST pkt_addr\t" << hex << req.get_addr() << dec << "\tsrc_id\t"
                        << pkt->get_src() << "\tdst_id\t" << pkt->get_dst()
                        << "\tpkt_src_port\t" << pkt->get_src_port()
                        << "\tpkt_dst_port\t" << pkt->get_dst_port() << endl;
            }
            else
            {
                num_mem_write_req_pkts_sent[i]++;
                cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + i
                            << "\tMEM WRITE REQUEST pkt_addr\t" << hex << req.get_addr() << dec << "\tsrc_id\t"
                            << pkt->get_src() << "\tdst_id\t" << pkt->get_dst()
                            << "\tpkt_src_port\t" << pkt->get_src_port()
                            << "\tpkt_dst_port\t" << pkt->get_dst_port() << endl;
            }
            PKTS_COUNTER++;
        }
        else
        {
//                cerr << dec << "@\t" << m_clk->NowTicks() << "\tPKT_gen_id\t" << (this->get_id() * num_cpu_ports) + i
//                     << "\tPKT_GEN_SENDING_STOPPED" << endl;
        }
#ifdef HMCXBAR
        }
#endif
        if(pkt_gen_clk.NowTicks() % 10000 == 0)
        {
            double duration = 10000.0 / pkt_gen_clk.freq;
            cerr << "int_inc_credits= " << intermediate_incoming_credits[i] << " int_out_credits= " << intermediate_outgoing_credits[i] << " duration= " << duration << endl;
            cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + i
                        << "\tBW\t" << (double) ( (intermediate_incoming_credits[i] + intermediate_outgoing_credits[i]) * 16 / duration ) / (1024 * 1024 * 1024) << " GB/s" << endl;
            clear_bw_counters(i);
        }
    }
}

void PKT_gen :: process_incoming_pkts(manifold::uarch::NetworkPacket* pkt, int port)
{
    if (pkt->type == MEM_MSG_TYPE)
    {
        manifold::uarch::Mem_msg* req = (manifold::uarch::Mem_msg*)pkt->data;

        uint64_t pkt_addr = req->get_addr(); //  Should be global address
        cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + port
                << "\tMEM READ RESPONSE pkt_addr\t"<< hex << pkt_addr << "\tpkt_src\t" << dec << pkt->get_src()
                << "\tpkt_dst\t"<< pkt->get_dst() << "\tpkt_src_port\t" << pkt->get_src_port()
                << "\tpkt_dst_port\t" << pkt->get_dst_port() << endl;

        num_mem_read_req_pkts_rcvd[port]++;
    }
#ifdef HMCXBAR
    else if (pkt->type == COH_MSG_TYPE)
    {
        manifold::mcp_cache_namespace::Coh_msg* req = (manifold::mcp_cache_namespace::Coh_msg*)pkt->data;

        uint64_t pkt_addr = req->addr; //  Should be global address
        cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + port
                << "\tCOH MSG RESPONSE pkt_addr\t"<< hex << pkt_addr << "\tpkt_src\t" << dec << pkt->get_src()
                << "\tpkt_dst\t"<< pkt->get_dst() << "\tpkt_src_port\t" << pkt->get_src_port()
                << "\tpkt_dst_port\t" << pkt->get_dst_port() << endl;
        cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen_id\t" << (this->get_id() * num_cpu_ports) + port
                    << "\tCOH msg type RESPONSE\t" << (int)req->type << "\taddr\t" << hex
                    << req->addr << dec << "\tmsg\t" << req->msg << "\tsrc_id\t"
                    << req->src_id << "\tdst_id\t" << req->dst_id << endl;
        if(req->type == 1)
            num_coh_req_pkts_rcvd[port]++;
        else
            num_coh_resp_pkts_rcvd[port]++;
    }
#endif
    else
    {
        cerr << dec << "@\t" << m_clk->NowTicks() << "\t PKT GEN Received unknown MSG TYPE. ERROR" << endl;
        assert(0);
    }
    send_credit_downstream(port);
}

void PKT_gen :: send_credit_downstream(int port)
{
	manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
	credit_pkt->type = CREDIT_MSG_TYPE;
    Send(port, credit_pkt);
    cerr << dec << "@\t" << m_clk->NowTicks() << "\tpkt_gen\t" << this->get_id() << "\tsending CREDIT pkt at PORT\t" << port << dec << endl;
    num_outgoing_credits[port]++;
    intermediate_outgoing_credits[port]++;
}

void PKT_gen :: set_hmc_map(manifold::uarch::DestMap *m)
{
	cout << "PKT_gen @\t" << this << "\tset_mc_map\t" << m << endl;
	this->pktgen_map = m;
}

void PKT_gen :: clear_bw_counters(int port)
{
    intermediate_incoming_credits[port] = 0;
    intermediate_outgoing_credits[port] = 0;
}

void PKT_gen :: print_stats(ostream& out)
{
    out << "****** PKT_GEN " << pkt_gen_id << "********* stats:" << endl;
    for(int i = 0; i < num_cpu_ports; i++)
    {
        out << "  CPU PORT[" << i <<"]" << endl
#ifdef HMCXBAR
            << "  num coh req msg sent: " << num_coh_req_pkts_sent[i] << endl
            << "  num coh req msg rcvd: " << num_coh_req_pkts_rcvd[i] << endl
            << "  num coh resp msg sent: " << num_coh_resp_pkts_sent[i] << endl
            << "  num coh resp msg rcvd: " << num_coh_resp_pkts_rcvd[i] << endl
#endif
            << "  num mem read req msg sent: " << num_mem_read_req_pkts_sent[i] << endl
            << "  num mem read resp msg rcvd: " << num_mem_read_req_pkts_rcvd[i] << endl
            << "  num mem write req msg sent: " << num_mem_write_req_pkts_sent[i] << endl
            << "  num outgoing credits: " << num_outgoing_credits[i] << endl
            << "  num incoming credits: " << num_incoming_credits[i] << endl
//            << "  Tx BW: " << ( ( ( (num_coh_req_pkts_sent[i] + num_mem_read_req_pkts_sent[i]) * req_flit_size)
//                                + ( (num_coh_resp_pkts_sent[i] + num_mem_write_req_pkts_sent[i]) * resp_flit_size ) ) * 16 ) * m_clk->freq << endl
//            << "  Rx BW: " << ( ( ( (num_coh_req_pkts_rcvd[i] + num_mem_read_req_pkts_rcvd[i]) * req_flit_size)
//                                + ( (num_coh_resp_pkts_rcvd[i]) * resp_flit_size ) ) * 16 ) * m_clk->freq << endl;
            << "  Average BW: " << (num_outgoing_credits[i] + num_incoming_credits[i]) * req_flit_size / (pkt_gen_clk.NowTicks() / pkt_gen_clk.freq ) << " GB/s" << endl;
    }

}


} // namespace pkt_generator
} //namespace manifold

