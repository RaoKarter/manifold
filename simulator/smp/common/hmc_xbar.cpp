#include "hmc_xbar.h"
#include "kernel/component.h"
#include "kernel/manifold.h"

using namespace manifold::kernel;
using namespace manifold::uarch;


namespace manifold {
namespace hmc_xbar {

int HMC_xbar :: MEM_MSG_TYPE = -1;
int HMC_xbar :: CREDIT_MSG_TYPE = -1;
bool HMC_xbar :: Msg_type_set = false;

HMC_xbar :: HMC_xbar(int id, const HMC_xbar_settings& hmcxbar_settings, Clock& clk) :
		hmc_id(id) , hmc_clk(clk)
{
	hmc_num_mc_ports = hmcxbar_settings.num_mem_links;
	hmc_num_net_ports = hmcxbar_settings.num_hmc_net_links;
	downstream_credits = new int [hmc_num_net_ports];
	upstream_credits = new int [hmc_num_mc_ports];
	this->hmcxbar_vault_map = hmcxbar_settings.s_vault_map;

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
//	vault_node_idx_vec.resize(hmc_num_mc_ports);

	int j = 0;
	for (int i = net_ports[hmc_num_net_ports-1] + 1 ; i < hmc_num_mc_ports + hmc_num_net_ports; ++i)
	{
		m_ports[j] = i;
		j++;
//		vault_node_idx_vec.push_back(i);
	}

//	vault_map = new manifold::uarch::PageBasedMap(vault_node_idx_vec, 0);

	//register with clock
	Clock :: Register(clk, this, &HMC_xbar::tick, (void(HMC_xbar::*)(void)) 0 );

	// Create one request buffer for each connected memory vault
	hmc_net_requests = new std::list<manifold::uarch::NetworkPacket*>[hmc_num_mc_ports];

	// Create one response buffer for each connected network port
	hmc_mc_responses = new std::list<manifold::uarch::NetworkPacket*>[hmc_num_net_ports];

    //stats
	stats_num_net_incoming_msg = 0;
	stats_num_net_outgoing_msg = 0;
	stats_num_net_incoming_credits = 0;
	stats_num_net_outgoing_credits = 0;
	stats_num_mc_incoming_msg = 0;
	stats_num_mc_outgoing_msg = 0;
	stats_num_mc_incoming_credits = 0;
	stats_num_mc_outgoing_credits = 0;
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
		if ( hmc_net_requests[i].size() > 0 && upstream_credits[i] > 0) // buffer containing handled network requests
		{
			NetworkPacket* pkt = hmc_net_requests[i].front();
			hmc_net_requests[i].pop_front();

			// Send the pkt to the vault
			Send(m_ports[i], pkt);
			upstream_credits[i]--;
			stats_num_mc_outgoing_msg++;
#ifdef HMCDEBUG
	cerr << dec << "@\t" << m_clk->NowTicks() <<"\t(tick)HMCupstream credits[" << i << "]\t" << upstream_credits[i]+1
			<< "->" << upstream_credits[i] << "\thmc_net_requests.size\t" << hmc_net_requests[i].size() << endl;
	manifold::mcp_cache_namespace::Mem_msg* req = (manifold::mcp_cache_namespace::Mem_msg*)pkt->data;
	uint64_t pkt_laddr = req->get_addr(); // Local address right now
	uint64_t pkt_gaddr = hmc_map->get_global_addr(pkt_laddr, this->get_hmc_id()); // Global address
	cerr << dec << "@\t" << m_clk->NowTicks() << "\thmc_id\t" << this->get_hmc_id() << "\tsending MSG pkt from MC port\t"
			<< dec << m_ports[i] << "(" << m_ports[i] - hmc_num_net_ports << ")\tsrc_id\t" << pkt->get_src() << "\tsrc_port\t" << pkt->get_src_port()
			<< "\tdst_id\t" << pkt->get_dst() << "\tdst_port\t" << pkt->get_dst_port() << "\tladdr\t" << hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << dec << endl;
#endif
		}
	}

	// Now start to process memory responses
	for(int i = 0; i < get_num_net_ports(); i++) // For each SerDes link
	{
		if ( hmc_mc_responses[i].size() > 0 && downstream_credits[i] > 0) // buffer containing memory responses
		{
			NetworkPacket* pkt = hmc_mc_responses[i].front();
			hmc_mc_responses[i].pop_front();

			// Send the packet into the network
			Send(net_ports[i],pkt);
			downstream_credits[i]--;
			stats_num_net_outgoing_msg++;
#ifdef HMCDEBUG
		cerr << dec << "@\t" << m_clk->NowTicks() <<"\t(tick)HMCdownstream credits[" << i << "]\t" << downstream_credits[i]+1
					<< "->" << downstream_credits[i] << "\thmc_mc_responses.size\t" << hmc_mc_responses[i].size() << endl;
		manifold::uarch::Mem_msg* req = (manifold::uarch::Mem_msg*)pkt->data;
		uint64_t pkt_gaddr = req->get_addr(); // Global address right now
		uint64_t pkt_laddr = hmc_map->get_local_addr(pkt_gaddr); // Local address
		cerr << dec << "@\t" << m_clk->NowTicks() << "\thmc_id\t" << this->get_hmc_id() << "\tsending MSG pkt from NET port\t"
			<< dec << net_ports[i] << "\tsrc_id\t" << pkt->get_src() << "\tsrc_port\t" << pkt->get_src_port()
			<< "\tdst_id\t" << pkt->get_dst() << "\tdst_port\t" << pkt->get_dst_port() << "\tladdr\t" << hex << pkt_laddr << "\tgaddr\t" << pkt_gaddr << dec << endl;
#endif
		}
	}

}



void HMC_xbar :: send_credit_downstream(int port)
{
	manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
	credit_pkt->type = CREDIT_MSG_TYPE;
	Send(m_ports[port], credit_pkt);
#ifdef HMCDEBUG
	cerr << dec << "@\t" << m_clk->NowTicks() << "\thmc_id\t" << this->get_hmc_id() << "\tsending CREDIT pkt from MC port\t"
			<< dec << m_ports[port] << endl;
#endif
    stats_num_mc_outgoing_credits++;
}

void HMC_xbar :: send_credit_upstream(int port)
{
	manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
	credit_pkt->type = CREDIT_MSG_TYPE;
	Send(net_ports[port], credit_pkt);
#ifdef HMCDEBUG
	cerr << dec << "@\t" << m_clk->NowTicks() << "\thmc_id\t" << this->get_hmc_id() << "\tsending CREDIT pkt from NET port\t"
			<< dec << net_ports[port] << endl;
#endif
    stats_num_net_outgoing_credits++;
}

void HMC_xbar :: set_hmc_map(manifold::uarch::DestMap *m)
{
#ifdef HMCDEBUG
	cout << "HMC xbar @\t" << this << "\tset_hmc_map\t" << m << endl;
#endif
    this->hmc_map = m;
}


void HMC_xbar :: print_stats(ostream& out)
{
	//TODO: This needs to be printed carefully
    out << "****** HMCXBAR " << hmc_id << "********* stats:" << endl
	<< "  incoming mc msg: " << stats_num_mc_incoming_msg << endl
	<< "  outgoing mc msg: " << stats_num_mc_outgoing_msg << endl
	<< "  incoming net msg: " << stats_num_net_incoming_msg << endl
	<< "  outgoing net msg: " << stats_num_net_outgoing_msg << endl
	<< "  incoming net credits: " << stats_num_net_incoming_credits << endl
    << "  incoming mc credits: " << stats_num_mc_incoming_credits << endl
    << "  mc outgoing credits: " << stats_num_mc_outgoing_credits << endl
    << "  net outgoing credits: " << stats_num_net_outgoing_credits << endl;
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
