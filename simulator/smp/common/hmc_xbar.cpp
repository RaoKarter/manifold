#include "hmc_xbar.h"
#include "LLP_cache.h"
#include "LLS_cache.h"
#include "kernel/component.h"
#include "kernel/manifold.h"

using namespace manifold::kernel;
using namespace manifold::uarch;


namespace manifold {
namespace xbar_namespace {

hmcxbar :: hmcxbar(int nid, Clock& clk, int num_mc_ports) : xbar_clk(clk), xbar_num_mc_ports(num_mc_ports), xbar_nid(nid)
{
	// Create ports for xbar to communicate with memory vaults
//	PORT_MC = new int[xbar_num_mc_ports];

	// Create one buffer for each connected memory vaults
	xbar_net_requests = new std::list<manifold::uarch::NetworkPacket*>[xbar_num_mc_ports];

    //stats
    stats_num_llp_incoming_msg = 0;
    stats_num_lls_incoming_msg = 0;
    stats_num_llp_outgoing_msg = 0;
    stats_num_lls_outgoing_msg = 0;
    stats_num_llp_incoming_credits = 0;
    stats_num_lls_incoming_credits = 0;
    stats_num_outgoing_credits = 0;
}

void hmcxbar :: tick()
{
	/*------------- The crossbar code comes here -----------------*/
	// Requests might come from MC side as well as the network side.
	// We process them in a deterministic manner. First the network
	// requests and later the MC requests

	for(int i = 0; i < xbar_num_mc_ports; i++)
	{
		if ( xbar_net_requests[i].size() > 0 )
		{
			NetworkPacket* pkt = xbar_net_requests[i].front();
			xbar_net_requests[i].pop_front();

			// Do something with the packet
			Send(PORT_MC1,pkt);
		}
	}

    if ( xbar_mc_requests.size() > 0 )
	{
		NetworkPacket* pkt = xbar_mc_requests.front();
		xbar_mc_requests.pop_front();

		// Send the packet into the network
		Send(PORT_NET,pkt);
	}


}



void hmcxbar :: send_credit_downstream()
{
    NetworkPacket* pkt = new NetworkPacket;
    pkt->type = CREDIT_MSG_TYPE;
    Send(PORT_NET, pkt);
//cout << "OOOOOOOOOOOOOOOOOOOOOOOOOO, @ " << m_clk.NowTicks() << " mux send credit\n";
//cout.flush();
    stats_num_outgoing_credits++;
}


void hmcxbar :: print_stats(ostream& out)
{
    out << "****** XBAR " << xbar_nid << "********* stats:" << endl
	<< "  incoming msg: " << stats_num_llp_incoming_msg << " (LLP)  "
	                      << stats_num_lls_incoming_msg << " (LLS)  "
			      << stats_num_llp_incoming_msg + stats_num_lls_incoming_msg << " (total)" << endl
	<< "  outgoing msg: " << stats_num_llp_outgoing_msg << " (LLP)  "
	                      << stats_num_lls_outgoing_msg << " (LLS)  "
			      << stats_num_llp_outgoing_msg + stats_num_lls_outgoing_msg << " (total)" << endl
	<< "  incoming credits: " << stats_num_llp_incoming_credits << " (LLP)  "
	                          << stats_num_lls_incoming_credits << " (LLS)  "
			          << stats_num_llp_incoming_credits + stats_num_lls_incoming_credits << " (total)" << endl
        << "  outgoing credits: " << stats_num_outgoing_credits << endl;
   	out << "  Total Reads Received= " << stats_n_reads << endl;
	out << "  Total Writes Received= " << stats_n_writes << endl;
	out << "  Total Reads Sent Back= " << stats_n_reads_sent << endl;
//	out << "  Avg Memory Latency= " << (double)stats_totalMemLat / stats_n_reads_sent << endl;
	out << "  Reads per source:" << endl;
	for(map<int, unsigned>::iterator it=stats_n_reads_per_source.begin(); it != stats_n_reads_per_source.end(); ++it)
	{
		out << "    " << it->first << ": " << it->second << endl;
	}
	out << "  Writes per source:" << endl;
	for(map<int, unsigned>::iterator it=stats_n_writes_per_source.begin(); it != stats_n_writes_per_source.end();++it)
	{
		out << "    " << it->first << ": " << it->second << endl;
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
