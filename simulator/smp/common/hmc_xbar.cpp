#include "hmc_xbar.h"
#include "kernel/component.h"
#include "kernel/manifold.h"
#include "mcp-cache/coh_mem_req.h"
#include "mcp-cache/cache_req.h"

using namespace manifold::kernel;
using namespace manifold::uarch;
using namespace manifold::mcp_cache_namespace;


namespace manifold {
namespace hmc_xbar {

int HMC_xbar :: MEM_MSG_TYPE = -1;
int HMC_xbar :: CREDIT_MSG_TYPE = -1;
int HMC_xbar :: COH_MSG_TYPE = -1;
bool HMC_xbar :: Msg_type_set = false;

HMC_xbar :: HMC_xbar(int id, const HMC_xbar_settings& hmcxbar_settings, Clock& clk) :
        hmc_id(id) , xbar_clk(clk)
{
	hmc_num_mc_ports = hmcxbar_settings.num_mem_links;
	hmc_num_net_ports = hmcxbar_settings.num_hmc_net_links;
	downstream_credits = new int [hmc_num_net_ports];
	upstream_credits = new int [hmc_num_mc_ports];
	this->hmcxbar_vault_map = hmcxbar_settings.s_vault_map;
    req_flit_size = hmcxbar_settings.req_flit_size;
    resp_flit_size = hmcxbar_settings.resp_flit_size;
    MAX_NET_CREDITS = hmcxbar_settings.downstream_credits;
    MAX_VAULT_CREDITS = hmcxbar_settings.upstream_credits;
    vault_transaction_size = hmcxbar_settings.vault_trans_size;

#ifdef HMCDEBUG
	cerr << dec << " HMC xbar @\t" << this << "\tvault map\t" << hmcxbar_vault_map << endl;
#endif

	for(int i = 0; i < hmc_num_net_ports; i++) // size is number of network ports
		downstream_credits[i] = hmcxbar_settings.downstream_credits;

	for(int i = 0; i < hmc_num_mc_ports; i++) // size is number of vaults
		upstream_credits[i] = hmcxbar_settings.upstream_credits;

	/*
	 * hmc_net_it is the iterator for the id_lp_map passed along with hmcxbar_settings
	 *
	 * This iterator will be initialized to point to the first network port of the HMC
	 * instance. The iterator can now be used to get rest of the network port ids to
	 * which the HMC instance is connected.
	 */
	hmc_net_it = hmcxbar_settings.id_lp_map->begin();
	std::advance (hmc_net_it, hmc_id * hmc_num_net_ports);

    assert(Msg_type_set);
    assert(MEM_MSG_TYPE != CREDIT_MSG_TYPE);

	// Create ports for hmc to communicate with the network
	assert(hmc_num_net_ports <= MAX_NET_PORT);

	string f;
	for (int i = 0; i < hmc_num_net_ports; ++i)
	{
		f = to_string((*hmc_net_it).first);
		hmc_nid_portid_map[f] = i;
//		hmc_nid_portid_map[i] = (*hmc_net_it).first;
#ifdef HMCDEBUG
		cerr << dec << "hmc_nid_portid_map[" << f << "] = " << i << endl;
//		cerr << dec << "hmc_nid_portid_map[" << i << "] = " << (*hmc_net_it).first << endl;
#endif
		net_ports[i] = i;
		std::advance(hmc_net_it, 1);
	}

	std::advance(hmc_net_it, -hmc_num_net_ports);
#ifdef HMCDEBUG
		cerr << dec << "Iterator location NOW " << (*hmc_net_it).first << endl;
#endif

	hmc_map = NULL;

	// Create ports for hmc to communicate with memory vaults
	assert(hmc_num_mc_ports <= MAX_MEM_PORT);

	int j = 0;
	for (int i = net_ports[hmc_num_net_ports-1] + 1 ; i < hmc_num_mc_ports + hmc_num_net_ports; ++i)
	{
		m_ports[j] = i;
		j++;
	}

	//register with clock
	Clock :: Register(clk, this, &HMC_xbar::tick, (void(HMC_xbar::*)(void)) 0 );
    time = 0;

    // Create a request buffer for each connected memory vault
    hmc_to_mc_buffer = new std::list<manifold::uarch::NetworkPacket*>[hmc_num_mc_ports];

    // Create a response buffer for each connected serdes port
    hmc_to_serdes_buffer = new std::list<manifold::uarch::NetworkPacket*>[hmc_num_net_ports];
    hmc_to_serdes_buffer_msg_type = new std::list<unsigned>[hmc_num_net_ports];

    serdes_rx_dep_time = new std::list<uint64_t>[hmc_num_net_ports];
    vault_rx_dep_time = new std::list<uint64_t>[hmc_num_mc_ports];

#ifdef HUTDEBUG
    serdes_rx_cur_clk_tick = new std::list<uint64_t>[hmc_num_net_ports];
    vault_rx_cur_clk_tick = new std::list<uint64_t>[hmc_num_mc_ports];
    serdes_rx_addr = new std::list<uint64_t>[hmc_num_net_ports];
    vault_rx_addr = new std::list<uint64_t>[hmc_num_mc_ports];
    serdes_rx_pkt_type = new std::list<uint64_t>[hmc_num_net_ports];
    vault_rx_pkt_type = new std::list<uint64_t>[hmc_num_mc_ports];
#endif /* HUTDEBUG */

    //stats
    stats_num_incoming_vault_msg = new unsigned [hmc_num_mc_ports];
    stats_num_outgoing_vault_msg = new unsigned [hmc_num_mc_ports];
    stats_num_incoming_vault_credits = new unsigned [hmc_num_mc_ports];
    stats_num_outgoing_vault_credits = new unsigned [hmc_num_mc_ports];
    intermediate_num_incoming_vault_credits = new unsigned [hmc_num_mc_ports];
    intermediate_num_outgoing_vault_credits = new unsigned [hmc_num_mc_ports];

    stats_num_incoming_read_req_serdes_msg = new unsigned [hmc_num_net_ports];
    stats_num_incoming_write_req_serdes_msg = new unsigned [hmc_num_net_ports];
    stats_num_outgoing_read_resp_serdes_msg = new unsigned [hmc_num_net_ports];

    stats_num_incoming_serdes_credits = new unsigned [hmc_num_net_ports];
    stats_num_outgoing_serdes_credits = new unsigned [hmc_num_net_ports];
    intermediate_num_incoming_serdes_credits =  new unsigned [hmc_num_net_ports];
    intermediate_num_outgoing_serdes_credits = new unsigned [hmc_num_net_ports];

#ifdef HMCXBAR
    stats_num_incoming_coh_req_serdes_msg = new unsigned [hmc_num_net_ports];
    stats_num_incoming_coh_resp_serdes_msg = new unsigned [hmc_num_net_ports];
    stats_num_outgoing_coh_req_serdes_msg = new unsigned [hmc_num_net_ports];
    stats_num_outgoing_coh_resp_serdes_msg = new unsigned [hmc_num_net_ports];
#endif


    for (int i = 0; i < hmc_num_mc_ports; i++)
    {
        stats_num_incoming_vault_msg[i] = 0;
        stats_num_outgoing_vault_msg[i] = 0;
        stats_num_incoming_vault_credits[i] = 0;
        stats_num_outgoing_vault_credits[i] = 0;
        intermediate_num_incoming_vault_credits[i] = 0;
        intermediate_num_outgoing_vault_credits[i] = 0;
    }
    for (int i = 0; i < hmc_num_net_ports; i++)
    {
        stats_num_incoming_read_req_serdes_msg[i] = 0;
        stats_num_incoming_write_req_serdes_msg[i] = 0;
        stats_num_outgoing_read_resp_serdes_msg[i] = 0;

        stats_num_incoming_serdes_credits[i] = 0;
        stats_num_outgoing_serdes_credits[i] = 0;
        intermediate_num_incoming_serdes_credits[i] = 0;
        intermediate_num_outgoing_serdes_credits[i] = 0;

#ifdef HMCXBAR
        stats_num_incoming_coh_req_serdes_msg[i] = 0;
        stats_num_incoming_coh_resp_serdes_msg[i] = 0;
        stats_num_outgoing_coh_req_serdes_msg[i] = 0;
        stats_num_outgoing_coh_resp_serdes_msg[i] = 0;
#endif
    }
}

void HMC_xbar :: tick()
{
	/*
	 * Network sends requests and the Vaults respond to those requests.
	 * Since multiple requests can arrive at the same time, we process
	 * these in a deterministic manner. First we process the network
	 * requests and then process the memory responses.
	 */

    for(int i = 0; i < get_num_mc_ports(); i++) // For each vault
	{
        if ( hmc_to_mc_buffer[i].size() > 0 && upstream_credits[i] > 0) // buffer containing handled network requests
		{
            NetworkPacket* pkt = hmc_to_mc_buffer[i].front();
            hmc_to_mc_buffer[i].pop_front();

            // Send the pkt to the vault
			Send(m_ports[i], pkt);
            upstream_credits[i]--;
            assert(upstream_credits[i] >= 0);
            stats_num_outgoing_vault_msg[i]++;

//#ifdef HMCDEBUG
//    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\t(tick)HMC upstream credits[" << i << "]\t" << upstream_credits[i]+1
//            << "->" << upstream_credits[i] << "\thmc_to_mc_buffer.size\t" << hmc_to_mc_buffer[i].size() << endl;
//	manifold::mcp_cache_namespace::Mem_msg* req = (manifold::mcp_cache_namespace::Mem_msg*)pkt->data;
//	uint64_t pkt_laddr = req->get_addr(); // Local address right now
//	uint64_t pkt_gaddr = hmc_map->get_global_addr(pkt_laddr, this->get_hmc_id()); // Global address
//    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\tsending MSG pkt from MC port\t"
//			<< dec << m_ports[i] << "(" << m_ports[i] - hmc_num_net_ports << ")\tsrc_id\t" << pkt->get_src() << "\tsrc_port\t" << pkt->get_src_port()
//			<< "\tdst_id\t" << pkt->get_dst() << "\tdst_port\t" << pkt->get_dst_port() << "\tladdr\t" << hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << dec << endl;
//#endif
		}
	}

    // Now start to process memory responses (If the xbar is used as the network, this queue will also
    // contain COH msgs).
    for(int i = 0; i < get_num_net_ports(); i++) // For each SerDes link
    {
        if ( hmc_to_serdes_buffer[i].size() > 0 && downstream_credits[i] > 0) // buffer containing memory responses
        {
            NetworkPacket* pkt = hmc_to_serdes_buffer[i].front();
            hmc_to_serdes_buffer[i].pop_front();
            Send(net_ports[i],pkt);
            downstream_credits[i]--;
            assert(downstream_credits[i] >= 0);
            if (pkt->type == MEM_MSG_TYPE)
                stats_num_outgoing_read_resp_serdes_msg++;
#ifdef HMCXBAR
            else if (pkt->type == COH_MSG_TYPE)
            {
                manifold::mcp_cache_namespace::Coh_msg* msg = (manifold::mcp_cache_namespace::Coh_msg*)pkt->data;
                if (msg->type == 0) // COH REQ
                    stats_num_outgoing_coh_req_serdes_msg++;
                else                // COH RESP
                    stats_num_outgoing_coh_resp_serdes_msg++;
            }
#endif
        }
    }

}

void HMC_xbar::handle_vault_responses(manifold::uarch::NetworkPacket *pkt, int serdes_port, int vault)
{
    unsigned int credit_type = 0;
    uint64_t pkt_delay = 0;
    // MEM MSG
    /*
     *  Read response is 3 FLITs = 48B. 16 bit wide SerDes implies 24 SerDes clock ticks
    */
    pkt_delay = vault_rx_calculate_delay(vault, serdes_port, 2);
#ifdef HUTDEBUG
    manifold::uarch::Mem_msg* msg = (manifold::uarch::Mem_msg*) pkt->data;
    vault_rx_addr[vault].push_back(msg->addr);
#endif
#ifdef HMCDEBUG
    cerr << "\tqueue MEM MSG after delay";
    cerr << " READ RESPONSE\t" << dec << "\tdelay\t" << pkt_delay << endl;
#endif
    credit_type = 1;
    manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->xbar_clk, &HMC_xbar::queue_after_xbar_delay_vault, this, pkt, vault, serdes_port, credit_type);
}

void HMC_xbar::handle_serdes_mem_msg(manifold::uarch::NetworkPacket* pkt, int in_port, int vault)
{    
    // THIS FUNCTION IS SPECIFICALLY FOR MEM MSG
#ifdef HMCDEBUG
    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thandle_serdes_mem_msg";
#endif
    unsigned int credit_type = 0;
    uint64_t pkt_delay = 0;
    manifold::mcp_cache_namespace::Mem_msg* msg = (manifold::mcp_cache_namespace::Mem_msg*) pkt->data;
    if (msg->type == 0)             // MEM_REQ
    {
        if (msg->is_read())         // READ REQ
        {
            stats_num_incoming_read_req_serdes_msg[in_port]++;
            /*
             *  Read request is 1 FLIT = 16B. 16 bit wide SerDes implies 8 SerDes clock ticks
            */
            pkt_delay = serdes_rx_calculate_delay(in_port, vault, 2);
#ifdef HUTDEBUG
            serdes_rx_addr[in_port].push_back(msg->addr);
#endif
#ifdef HMCDEBUG
            cerr << " MEM READ REQ addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
            credit_type = 0;
            manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->xbar_clk, &HMC_xbar::queue_mem_after_xbar_delay_serdes, this, pkt, in_port, vault, credit_type);
        }
        else                        // WRITE REQ
        {
            stats_num_incoming_write_req_serdes_msg[in_port]++;
            /*
             *  Write request is 3 FLITs = 48B. 16 bit wide SerDes implies 24 SerDes clock ticks
            */
            pkt_delay = serdes_rx_calculate_delay(in_port, vault, 3);
#ifdef HUTDEBUG
            serdes_rx_addr[in_port].push_back(msg->addr);
#endif
#ifdef HMCDEBUG
            cerr << " MEM WRITE REQ addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
            credit_type = 1;
            manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->xbar_clk, &HMC_xbar::queue_mem_after_xbar_delay_serdes, this, pkt, in_port, vault, credit_type);
        }
    }
    else if (msg->type == 1)        // MEM_RPLY
    {
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\tHMC handle_mem_msg MEM MSG TYPE ERROR. CPU cannot send read/write response type" << endl;
        assert(0);
    }
    else
    {
        cerr << dec << "\n@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\trcvd NET MSG pkt BAD REQ TYPE ERROR!!!! " << endl;
        assert(0);
    }
}

#ifdef HMCXBAR
void HMC_xbar::handle_serdes_coh_msg(manifold::uarch::NetworkPacket* pkt, int in_port, int dst)
{
    // THIS FUNCTION IS SPECIFICALLY FOR COH MSG
#ifdef HMCDEBUG
    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thandle_serdes_coh_msg";
#endif
    unsigned int credit_type = 0;
    uint64_t pkt_delay = 0;
    manifold::mcp_cache_namespace::Coh_msg* msg = (manifold::mcp_cache_namespace::Coh_msg*) pkt->data;

    switch(msg->type)
    {
        case 0:                     // Coh Request
        {
            stats_num_incoming_coh_req_serdes_msg[in_port]++;
            /*
             * A coh request is 1 FLIT = 16B. Assuming 16 bit lines for SerDes link
             * 2B get transferred every SerDes clock tick.
            */
            pkt_delay = serdes_rx_calculate_delay(in_port, dst, 0);
#ifdef HUTDEBUG
            serdes_rx_addr[in_port].push_back(msg->addr);
#endif
    #ifdef HMCDEBUG
            cerr << " COH REQ addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
    #endif
            credit_type = 0;
            manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->xbar_clk, &HMC_xbar::queue_coh_after_xbar_delay_serdes, this, pkt, in_port, dst, credit_type);
            break;
        }
        case 1:                     // Coh Reply
        {
            stats_num_incoming_coh_resp_serdes_msg[in_port]++;
            /*
             * A coh reply is 3 FLITs (1 FLIT overhead + 2 FLITs data) = 48B.
             * Assuming 16 bit lines for SerDes link 2B get transferred every
             * SerDes clock tick.
            */
            pkt_delay = serdes_rx_calculate_delay(in_port, dst, 1);
#ifdef HUTDEBUG
            serdes_rx_addr[in_port].push_back(msg->addr);
#endif
#ifdef HMCDEBUG
            cerr << " COH REPLY addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
            credit_type = 1;
            manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->xbar_clk, &HMC_xbar::queue_coh_after_xbar_delay_serdes, this, pkt, in_port, dst, credit_type);
            break;
        }
        default:
        {
            cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\tHMC ReceivedNET BAD COH MSG" << endl;
            assert(0);
        }
    }
}

void HMC_xbar::queue_coh_after_xbar_delay_serdes(manifold::uarch::NetworkPacket *pkt, int in_port, int dst, unsigned int type)
{
    // THIS FUNCTION IS SPECIFICALLY FOR COH MSG
    assert(pkt->type == COH_MSG_TYPE);
    assert( serdes_rx_dep_time[in_port].front() == xbar_clk.NowTicks());

    hmc_to_serdes_buffer[dst].push_back(pkt);
    send_credit_upstream(in_port, type); // Send credits to SerDes (downstream_credits)

#ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\tqueue_coh_after_xbar_delay COH MSG TYPE port\t" << in_port << "\tdst\t" << dst << endl;
        cerr << "8888888888 SERDES RX BUFFER STATS " << in_port << " 8888888888888" << endl;
        cerr << "ARR_CLK\t\tTYPE\tDEP_CLK\t\tADDR" << endl;
#ifdef HUTDEBUG
        std::list<uint64_t>::iterator it1 = serdes_rx_cur_clk_tick[in_port].begin();
        std::list<uint64_t>::iterator it2 = serdes_rx_pkt_type[in_port].begin();
        std::list<uint64_t>::iterator it4 = serdes_rx_addr[in_port].begin();
#endif /* HUTDEBUG */
        for(std::list<uint64_t>::iterator it3 = serdes_rx_dep_time[in_port].begin(); it3 != serdes_rx_dep_time[in_port].end(); ++it3)
        {
#ifdef HUTDEBUG
            cerr << *it1 << "\t\t\t" << *it2 << "\t\t" << *it3 << "\t\t\t" << hex << *it4 << dec << endl;
            ++it1;
            ++it2;
            ++it4;
#else
            cerr << "  \t\t  \t\t" << *it3 << endl;
#endif /* HUTDEBUG */
        }
#endif /* HMCDEBUG */
    serdes_rx_dep_time[in_port].pop_front();

#ifdef HUTDEBUG
    serdes_rx_cur_clk_tick[in_port].pop_front();
    serdes_rx_pkt_type[in_port].pop_front();
    serdes_rx_addr[in_port].pop_front();
#endif /* HUTDEBUG */
}
#endif

// queue mem req from serdes to vault after xbar delay
void HMC_xbar::queue_mem_after_xbar_delay_serdes(manifold::uarch::NetworkPacket *pkt, int in_port, int vault, int type)
{
    // THIS FUNCTION IS SPECIFICALLY FOR MEM MSG
    assert(pkt->type == MEM_MSG_TYPE);
    assert( serdes_rx_dep_time[in_port].front() == xbar_clk.NowTicks());

    hmc_to_mc_buffer[vault].push_back(pkt);
    send_credit_upstream(in_port, type); // Send credits to SerDes (downstream_credits)

#ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\tqueue_mem_after_xbar_delay MEM MSG TYPE port\t" << in_port << "\tvault\t" << vault << endl;
        cerr << "8888888888 SERDES RX BUFFER STATS " << in_port << " 8888888888888" << endl;
        cerr << "ARR_CLK\t\tTYPE\tDEP_CLK\t\tADDR" << endl;
#ifdef HUTDEBUG
        std::list<uint64_t>::iterator it1 = serdes_rx_cur_clk_tick[in_port].begin();
        std::list<uint64_t>::iterator it2 = serdes_rx_pkt_type[in_port].begin();
        std::list<uint64_t>::iterator it4 = serdes_rx_addr[in_port].begin();
#endif /* HUTDEBUG */
        for(std::list<uint64_t>::iterator it3 = serdes_rx_dep_time[in_port].begin(); it3 != serdes_rx_dep_time[in_port].end(); ++it3)
        {
#ifdef HUTDEBUG
            cerr << *it1 << "\t\t\t" << *it2 << "\t\t" << *it3 << "\t\t\t" << hex << *it4 << dec << endl;
            ++it1;
            ++it2;
            ++it4;
#else
            cerr << "  \t\t  \t\t" << *it3 << endl;
#endif /* HUTDEBUG */
        }
#endif /* HMCDEBUG */
    serdes_rx_dep_time[in_port].pop_front();

#ifdef HUTDEBUG
    serdes_rx_cur_clk_tick[in_port].pop_front();
    serdes_rx_pkt_type[in_port].pop_front();
    serdes_rx_addr[in_port].pop_front();
#endif /* HUTDEBUG */
}

// queue mem responses from vault to serdes after xbar delay
void HMC_xbar::queue_after_xbar_delay_vault(manifold::uarch::NetworkPacket *pkt, int in_port, int serdes_port, unsigned int type)
{
    // THIS FUNCTION IS SPECIFICALLY FOR MEM MSG
    assert(pkt->type == MEM_MSG_TYPE);
    assert( vault_rx_dep_time[in_port].front() == xbar_clk.NowTicks() );

    hmc_to_serdes_buffer[serdes_port].push_back(pkt);
    send_credit_downstream(in_port); // Send credits to Vaults (upstream_credits)

 #ifdef HMCDEBUG
        cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\tqueue_after_xbar_delay_vault MEM MSG TYPE vault\t" << in_port << "\tto serdes\t" << serdes_port << endl;
        cerr << "8888888888 VAULT RX BUFFER STATS " << in_port << " 8888888888888" << endl;
        cerr << "ARR_CLK\t\tTYPE\tDEP_CLK\t\tADDR" << endl;
#ifdef HUTDEBUG
        std::list<uint64_t>::iterator it1 = vault_rx_cur_clk_tick[in_port].begin();
        std::list<uint64_t>::iterator it2 = vault_rx_pkt_type[in_port].begin();
        std::list<uint64_t>::iterator it4 = vault_rx_addr[in_port].begin();
#endif /* HUTDEBUG */
        for(std::list<uint64_t>::iterator it3 = vault_rx_dep_time[in_port].begin(); it3 != vault_rx_dep_time[in_port].end(); ++it3)
        {
#ifdef HUTDEBUG
            cerr << *it1 << "\t\t\t" << *it2 << "\t\t" << *it3 << "\t\t\t" << hex << *it4 << dec << endl;
            ++it1;
            ++it2;
            ++it4;
#else
            cerr << "  \t\t  \t\t" << *it3 << endl;
#endif /* HUTDEBUG */
        }
#endif /* HMCDEBUG */
    vault_rx_dep_time[in_port].pop_front();

#ifdef HUTDEBUG
    vault_rx_cur_clk_tick[in_port].pop_front();
    vault_rx_pkt_type[in_port].pop_front();
    vault_rx_addr[in_port].pop_front();
#endif /* HUTDEBUG */
}

void HMC_xbar :: send_credit_upstream(int port, int type)
{
    manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
    credit_pkt->type = CREDIT_MSG_TYPE;
    Send(net_ports[port], credit_pkt);
#ifdef HMCDEBUG
    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\tsending CREDIT pkt from NET port\t"
            << dec << net_ports[port] << endl;
#endif
    stats_num_outgoing_serdes_credits[port]++;
    if (type == 0)
        intermediate_num_outgoing_serdes_credits[port]++;
    else
        intermediate_num_outgoing_serdes_credits[port] += resp_flit_size;
}

void HMC_xbar :: send_credit_downstream(int port)
{
    manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
    credit_pkt->type = CREDIT_MSG_TYPE;
    Send(m_ports[port], credit_pkt);
#ifdef HMCDEBUG
    cerr << dec << "@\t" << xbar_clk.NowTicks() << " xbar_clk\thmc_id\t" << this->get_hmc_id() << "\tsending CREDIT pkt from MC port\t"
            << dec << m_ports[port] << endl;
#endif
    stats_num_outgoing_vault_credits[port]++;
    intermediate_num_outgoing_vault_credits[port]++;
}

void HMC_xbar :: clear_xbar_counters()
{
    for(int i = 0; i < hmc_num_mc_ports; i++)
    {
        intermediate_num_incoming_vault_credits[i] = 0;
        intermediate_num_outgoing_vault_credits[i] = 0;
    }
    for(int i = 0; i < hmc_num_net_ports; i++)
    {
        intermediate_num_incoming_serdes_credits[i] = 0;
        intermediate_num_outgoing_serdes_credits[i] = 0;
    }

}

void HMC_xbar :: set_hmc_map(manifold::uarch::DestMap *m)
{
#ifdef HMCDEBUG
    cout << "HMC xbar @\t" << this << "\tset_hmc_map\t" << m << endl;
#endif
    this->hmc_map = m;
}

uint64_t HMC_xbar::serdes_rx_calculate_delay(int from_port, int to_port, int type)
{
    #ifdef HMCDEBUG
    cerr << "\nserdes rx delay calculation" << endl;
    #endif
    #ifdef HUTDEBUG
    assert(serdes_rx_cur_clk_tick[from_port].size() == serdes_rx_dep_time[from_port].size());
    serdes_rx_cur_clk_tick[from_port].push_back(xbar_clk.NowTicks());
    #endif
    uint64_t delay = 0;
    int to_quadrant = 0;

    if(serdes_rx_dep_time[from_port].empty())
    {
        switch(type)
        {
        case 0:     // COH REQ
            delay = 1;
            serdes_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 1);
            break;
        case 1:     // COH REPLY
            delay = 5;
            serdes_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 5);
            break;
        case 2:     // READ REQ
        {
            /* Some quadrant logic. If read request is from a different
             * quadrant, add some extra delay.
             */
            to_quadrant = to_port / ( get_num_mc_ports() / get_num_net_ports() );
#ifdef HMCDEBUG
            cerr << "\t to_port quadrant= " << to_quadrant << " from_quadrant= " << from_port << endl;
#endif
            if (to_quadrant != from_port)
            {
                delay = 2 + 4;
                serdes_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 2 + 4);
            }
            else
            {
                delay = 2;
                serdes_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 2);
            }
            break;
        }
        case 3:     // WRITE REQ
        {
            /* Some quadrant logic. If write request is from a different
             * quadrant, add some extra delay.
             */
            to_quadrant = to_port / ( get_num_mc_ports() / get_num_net_ports() );
#ifdef HMCDEBUG
            cerr << "\t to_port quadrant= " << to_quadrant << " from_quadrant= " << from_port << endl;
#endif
            if (to_quadrant != from_port)
            {
                delay = 6 + 4;
                serdes_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 6 + 4);
            }
            else
            {
                delay = 6;
                serdes_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 6);
            }
            break;
        }
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
    #ifdef HUTDEBUG
        serdes_rx_pkt_type[from_port].push_back((uint64_t)type);
    #endif
        return delay;
    }
    else
    {
        switch(type)
        {
        case 0:     // COH REQ
            serdes_rx_dep_time[from_port].push_back(serdes_rx_dep_time[from_port].back() + 2);
            break;
        case 1:     // COH REPLY
            serdes_rx_dep_time[from_port].push_back(serdes_rx_dep_time[from_port].back() + 6);
            break;
        case 2:     // READ REQ
        {
            /* Some quadrant logic. If read request is from a different
             * quadrant, add some extra delay.
             */
            to_quadrant = to_port / ( get_num_mc_ports() / get_num_net_ports() );
#ifdef HMCDEBUG
            cerr << "\t to_port quadrant= " << to_quadrant << " from_quadrant= " << from_port << endl;
#endif
            if (to_quadrant != from_port)
                serdes_rx_dep_time[from_port].push_back(serdes_rx_dep_time[from_port].back() + 2 + 4);
            else
                serdes_rx_dep_time[from_port].push_back(serdes_rx_dep_time[from_port].back() + 2);
            break;
        }
        case 3:     // WRITE REQ
        {
            /* Some quadrant logic. If write request is from a different
             * quadrant, add some extra delay.
             */
            to_quadrant = to_port / ( get_num_mc_ports() / get_num_net_ports() );
#ifdef HMCDEBUG
            cerr << "\t to_port quadrant= " << to_quadrant << " from_quadrant= " << from_port << endl;
#endif
            if (to_quadrant != from_port)
                serdes_rx_dep_time[from_port].push_back(serdes_rx_dep_time[from_port].back() + 6 + 4);
            else
                serdes_rx_dep_time[from_port].push_back(serdes_rx_dep_time[from_port].back() + 6);
            break;
        }
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
        delay = serdes_rx_dep_time[from_port].back() - xbar_clk.NowTicks();
    #ifdef HUTDEBUG
        serdes_rx_pkt_type[from_port].push_back((uint64_t)type);
    #endif
        return delay;
    }
}

uint64_t HMC_xbar::vault_rx_calculate_delay(int from_port, int to_port, int type)
{
#ifdef HMCDEBUG
    cerr << "\nvault rx delay calculation" << endl;
#endif
#ifdef HUTDEBUG
    assert(vault_rx_cur_clk_tick[from_port].size() == vault_rx_dep_time[from_port].size());
    vault_rx_cur_clk_tick[from_port].push_back(xbar_clk.NowTicks());
#endif
    uint64_t delay = 0;
    int from_quadrant = 0;

    if(vault_rx_dep_time[from_port].empty())
    {
        switch(type)
        {
        case 0:     // COH REQ
        case 1:     // COH REPLY
            cerr << "vault cannot send COH MSG" << endl;
            assert(0);
            break;
        case 2:     // READ RESP
        {
            /* Some quadrant logic. If read request is from a different
             * quadrant, add some extra delay.
             */
            from_quadrant = from_port / ( get_num_mc_ports() / get_num_net_ports() );
#ifdef HMCDEBUG
            cerr << "\t from_quadrant= " << from_quadrant << " to_port= " << to_port << endl;
#endif
            if (from_quadrant != to_port)
            {
                delay = 4 + 4;
                vault_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 4 + 4);
            }
            else
            {
                delay = 4;
                vault_rx_dep_time[from_port].push_back(xbar_clk.NowTicks() + 4);
            }
            break;
        }
        case 3:     // WRITE RESP
            cerr << "vault cannot send WRITE RESP" << endl;
            assert(0);
            break;
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
#ifdef HUTDEBUG
        vault_rx_pkt_type[from_port].push_back((uint64_t)type);
#endif
        return delay;
    }
    else
    {
        switch(type)
        {
        case 0:     // COH REQ
        case 1:     // COH REPLY
            cerr << "vault cannot send COH MSG" << endl;
            assert(0);
            break;
        case 2:     // READ RESP
        {
            /* Some quadrant logic. If read request is from a different
             * quadrant, add some extra delay.
             */
            from_quadrant = from_port / ( get_num_mc_ports() / get_num_net_ports() );
#ifdef HMCDEBUG
            cerr << "\t from_quadrant= " << from_quadrant << " to_port= " << to_port << endl;
#endif
            if (from_quadrant != to_port)
                vault_rx_dep_time[from_port].push_back(vault_rx_dep_time[from_port].back() + 4 + 4);
            else
                vault_rx_dep_time[from_port].push_back(vault_rx_dep_time[from_port].back() + 4);
            break;
        }
        case 3:     // WRITE RESP
            cerr << "vault cannot send WRITE RESP" << endl;
            assert(0);
            break;
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
        delay = vault_rx_dep_time[from_port].back() - xbar_clk.NowTicks();
    #ifdef HUTDEBUG
        vault_rx_pkt_type[from_port].push_back((uint64_t)type);
    #endif
        return delay;
    }
}

void HMC_xbar :: print_stats(ostream& out)
{
    out << "****** HMCXBAR " << hmc_id << "********* stats:" << endl;
    for (int i = 0; i < hmc_num_mc_ports; i++)
    {
        out << "  incoming vault[" << i <<"] msg: " << stats_num_incoming_vault_msg[i]
        << "  outgoing vault[" << i <<"] msg: " << stats_num_outgoing_vault_msg[i] << endl
        << "  incoming vault[" << i <<"] credits: " << stats_num_incoming_vault_credits[i]
        << "  outgoing vault[" << i <<"] credits: " << stats_num_outgoing_vault_credits[i] << endl;
    }
    for (int i = 0; i < hmc_num_net_ports; i++)
    {
        out << "  incoming serdes[" << i <<"] read req msg: " << stats_num_incoming_read_req_serdes_msg[i] << endl
        << "  outgoing serdes[" << i <<"] read resp msg: " << stats_num_outgoing_read_resp_serdes_msg[i] << endl
        << "  incoming serdes[" << i <<"] write req msg: " << stats_num_incoming_write_req_serdes_msg[i] << endl

#ifdef HMCXBAR
        << "  incoming serdes[" << i <<"] coh req msg: " << stats_num_incoming_coh_req_serdes_msg[i] << endl
        << "  outgoing serdes[" << i <<"] coh req msg: " << stats_num_outgoing_coh_req_serdes_msg[i] << endl
        << "  incoming serdes[" << i <<"] coh resp msg: " << stats_num_incoming_coh_resp_serdes_msg[i] << endl
        << "  outgoing serdes[" << i <<"] coh resp msg: " << stats_num_outgoing_coh_resp_serdes_msg[i] << endl
#endif
        << "  incoming serdes[" << i <<"] credits: " << stats_num_incoming_serdes_credits[i] << endl
        << "  outgoing serdes[" << i <<"] credits: " << stats_num_outgoing_serdes_credits[i] << endl;
    }
}

#ifdef FORECAST_NULL
void MuxDemux :: do_output_to_net_prediction()
{

    Ticks_t now = m_clk.NowTicks();

    manifold::kernel::BorderPort* bp = this->border_ports[PORT_NET];
    assert(bp);

    if(!m_llp->m_downstream_output_buffer.empty() || !m_lls->m_downstream_output_buffer.empty()) { //if output buffer has msg
	bp->update_output_tick(now);
        return;
    }

    //remove all old values
    while(m_llp->m_msg_out_ticks.size() > 0)
        if(m_llp->m_msg_out_ticks.front() < now)
	    m_llp->m_msg_out_ticks.pop_front();
	else
	    break;
    while(m_llp->m_credit_out_ticks.size() > 0)
        if(m_llp->m_credit_out_ticks.front() < now)
	    m_llp->m_credit_out_ticks.pop_front();
	else
	    break;

    while(m_lls->m_msg_out_ticks.size() > 0)
        if(m_lls->m_msg_out_ticks.front() < now)
	    m_lls->m_msg_out_ticks.pop_front();
	else
	    break;
    while(m_lls->m_credit_out_ticks.size() > 0)
        if(m_lls->m_credit_out_ticks.front() < now)
	    m_lls->m_credit_out_ticks.pop_front();
	else
	    break;

    //The earliest tick for possible outgoing msg/credit is the minimum of the following
    //3 values:
    //(1) earliest scheduled msg or credit
    //(2) min lookup time; note, if (1) exists, it must be smaller than (2)
    //(3) credit for unprocessed incoming msg

    //Find the earliest output msg or credit
    Ticks_t out_ticks[4];
    int count = 0;
    if(m_llp->m_msg_out_ticks.size() > 0)
        out_ticks[count++] =  m_llp->m_msg_out_ticks.front();
    if(m_llp->m_credit_out_ticks.size() > 0)
        out_ticks[count++] =  m_llp->m_credit_out_ticks.front();
    if(m_lls->m_msg_out_ticks.size() > 0)
        out_ticks[count++] =  m_lls->m_msg_out_ticks.front();
    if(m_lls->m_credit_out_ticks.size() > 0)
        out_ticks[count++] =  m_lls->m_credit_out_ticks.front();

    //find the minimum
    Ticks_t min = 0;
    if(count > 0) {
        min = out_ticks[0];
	for(int i=1; i<count; i++)
	    if(out_ticks[i] < min)
	        min = out_ticks[i];
    }

    if(min == now) {
	bp->update_output_tick(now);
        return; //short circuit; no need to continue
    }


    //at this point, min==0 (count==0) OR min > now
    assert(bp->get_clk() == &m_clk);

    if(count == 0) {
        min = now + ((m_llp->my_table->get_lookup_time() < m_lls->my_table->get_lookup_time()) ?
	             m_llp->my_table->get_lookup_time() : m_lls->my_table->get_lookup_time());
    }

    //at this point min is earliest outgoing msg/credit without considering unprocessed incoming msgs.

    //check if any incoming msg scheduled for current or future cycle; credit is sent for each incoming msg.
    while(m_input_msg_ticks.size() > 0)
        if(m_input_msg_ticks.front() < now) //if earlier than now, it must have been processed already, and
	    m_input_msg_ticks.pop_front();  //if any output, than it must be in m_msg_out_ticks or m_credit_out_ticks.
	else
	    break;
    if(m_input_msg_ticks.size() > 0) {
        if(m_input_msg_ticks.front() == now) {
	    m_input_msg_ticks.pop_front();
	    bp->update_output_tick(now + 1); //asumption: credit is sent one tick after msg; the earliest to send credit is next tick
            return; //can't be smaller than now+1
        }
        else {//event scheduled for future
	    Ticks_t t = m_input_msg_ticks.front() + 1; //assumption: credit is sent one tick after msg
            min = (min < t) ? min : t; //get the smaller
        }
    }

    bp->update_output_tick(min);
    //cout << "@ " << m_clk.NowTicks() << " mux preddd " << min << "\n";
//cout.flush();

}


void MuxDemux :: remote_input_notify(Ticks_t when, void* data, int port)
{
    NetworkPacket* pkt = (NetworkPacket*)data;
    if(pkt->get_type() == CREDIT_MSG_TYPE) {
        //ignore
    }
    else {
        if(m_input_msg_ticks.size() > 0) {
            assert(m_input_msg_ticks.back() <= when);
        }
        m_input_msg_ticks.push_back(when);
    }
}

#endif //ifdef FORECAST_NULL





} // namespace xbar_namespace
} //namespace manifold
