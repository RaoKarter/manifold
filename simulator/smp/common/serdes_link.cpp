#include "serdes_link.h"
#include "kernel/component.h"
#include "kernel/manifold.h"
#include "mcp-cache/coh_mem_req.h"
#include "mcp-cache/cache_req.h"

#define MEAS_CYCLES 10000

using namespace manifold::kernel;
using namespace manifold::uarch;
using namespace manifold::mcp_cache_namespace;


namespace manifold {
namespace hmc_serdes {

int HMC_SerDes :: MEM_MSG_TYPE = -1;
int HMC_SerDes :: CREDIT_MSG_TYPE = -1;
int HMC_SerDes :: COH_MSG_TYPE = -1;
bool HMC_SerDes :: Msg_type_set = false;

HMC_SerDes :: HMC_SerDes(int id, const HMC_SerDes_settings& serdes_settings, Clock& clk) :
        hmc_serdes_id(id) , serdes_clk(clk)
{
    downstream_credits = serdes_settings.downstream_credits;
    MAX_DOWNSTREAM_CREDITS = downstream_credits;
    upstream_credits = serdes_settings.upstream_credits;
    MAX_UPSTREAM_CREDITS = upstream_credits;
    num_rx_tx_lines = serdes_settings.num_rx_tx_lines;
    req_flit_size = serdes_settings.req_flit_size;
    resp_flit_size = serdes_settings.resp_flit_size;

    assert(Msg_type_set);
    assert(MEM_MSG_TYPE != CREDIT_MSG_TYPE);

    //register with clock
    Clock :: Register(clk, this, &HMC_SerDes::tick, (void(HMC_SerDes::*)(void)) 0 );

    time = 0;

    //stats
    stats_num_incoming_net_read_req_msg = 0;
    stats_num_incoming_net_write_req_msg = 0;
    stats_num_incoming_xbar_read_resp_msg = 0;
    stats_num_outgoing_xbar_read_req_msg = 0;
    stats_num_outgoing_xbar_write_req_msg = 0;
    stats_num_outgoing_net_read_resp_msg = 0;

    stats_num_incoming_net_credits = 0;
    stats_num_incoming_xbar_credits = 0;
    stats_num_outgoing_net_credits = 0;
    stats_num_outgoing_xbar_credits = 0;

    intermediate_outgoing_xbar_credits = 0;
    intermediate_outgoing_net_credits = 0;

#ifdef HMCXBAR
    stats_num_incoming_coh_req_net_msg = 0;
    stats_num_incoming_coh_resp_net_msg = 0;
    stats_num_outgoing_coh_req_net_msg = 0;
    stats_num_outgoing_coh_resp_net_msg = 0;

    stats_num_incoming_coh_req_xbar_msg = 0;
    stats_num_incoming_coh_resp_xbar_msg = 0;
    stats_num_outgoing_coh_req_xbar_msg = 0;
    stats_num_outgoing_coh_resp_xbar_msg = 0;
#endif
}

void HMC_SerDes :: tick()
{
    /*
     * First handle the to_xbar buffer. If the buffer is not empty and has sufficient
     * upstream credits, send the pkt to the HMC xbar.
     */
    if ( serdes_to_xbar_buffer.size() > 0 && upstream_credits > 0) // buffer containing handled network requests
    {
        NetworkPacket* pkt = serdes_to_xbar_buffer.front();
        serdes_to_xbar_buffer.pop_front();

        // Send the pkt to the xbar
        Send(XBAR_PORT, pkt);
        if(pkt->type == MEM_MSG_TYPE)
        {
            manifold::uarch::Mem_msg* msg = (manifold::uarch::Mem_msg*) pkt->data;
            if(msg->is_read())  // MEM READ REQ
                stats_num_outgoing_xbar_read_req_msg++;
            else                // MEM WRITE REQ
                stats_num_outgoing_xbar_write_req_msg++;
        }
#ifdef HMCXBAR
        else if(pkt->type == COH_MSG_TYPE)
        {
            manifold::mcp_cache_namespace::Coh_msg* msg = (manifold::mcp_cache_namespace::Coh_msg*) pkt->data;
            if(msg->type == 0)  // COH REQ
                stats_num_outgoing_coh_req_xbar_msg++;
            else                // COH RESP
                stats_num_outgoing_coh_resp_xbar_msg++;
        }
#endif

        upstream_credits--;
        assert(upstream_credits >= 0);
    }

    /*
     * Next handle the to_net buffer. If the buffer is not empty and has sufficient
     * downstream credits, send the pkt to the network or cache (in the case where
     * the HMC xbar is the network).
     */
    if ( serdes_to_net_buffer.size() > 0 && downstream_credits > 0) // buffer containing handled hmcxbar responses
    {
        NetworkPacket* pkt = serdes_to_net_buffer.front();
        serdes_to_net_buffer.pop_front();

        // Send the pkt to the NET
        Send(NET_PORT, pkt);
        if (pkt->type == MEM_MSG_TYPE)
            stats_num_outgoing_net_read_resp_msg++;
#ifdef HMCXBAR
        else if(pkt->type == COH_MSG_TYPE)
        {
            manifold::mcp_cache_namespace::Coh_msg* msg = (manifold::mcp_cache_namespace::Coh_msg*) pkt->data;
            if(msg->type == 0)  // COH REQ
                stats_num_outgoing_coh_req_net_msg++;
            else                // COH RESP
                stats_num_outgoing_coh_resp_net_msg++;
        }
#endif

        downstream_credits--;
        assert(downstream_credits >= 0);
    }
}

void HMC_SerDes::handle_xbar_mem_msg(manifold::uarch::NetworkPacket *pkt)
{    
#ifdef HMCDEBUG
    cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk " << "\thandle_xbar_mem_msg";
#endif
    unsigned int credit_type = 0;
    uint64_t pkt_delay = 0;

    if (pkt->type == MEM_MSG_TYPE)
    {
        manifold::uarch::Mem_msg* msg = (manifold::uarch::Mem_msg*) pkt->data;
        if (msg->type == 0)             // MEM_REQ
        {
            // Vaults should only send responses!!!
            cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\thandle_xbar_msg MEM MSG TYPE ERROR. Vault cannot send read/write request type" << endl;
            assert(0);
        }
        else if (msg->type == 1)        // MEM_RPLY
        {
            stats_num_incoming_xbar_read_resp_msg++;
            /*
             *  Read response is 3 FLITs = 48B. 16 bit wide SerDes implies 24 SerDes clock ticks
            */
            pkt_delay = xbar_rx_calculate_delay(2);
#ifdef HUTDEBUG
                xbar_rx_addr.push_back(msg->addr);
#endif
#ifdef HMCDEBUG
            cerr << " MEM READ RESP addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
            credit_type = 1;
            manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->serdes_clk, &HMC_SerDes::queue_after_serdes_time_to_net, this, pkt, credit_type);
        }
        else
        {
            cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tserdes_id\t" << this->get_serdes_id() << "\trcvd NET MSG pkt BAD REQ TYPE ERROR!!!! " << endl;
            assert(0);
        }
    }
    else
    {
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tSerDes Received BAD MSG TYPE from xbar" << endl;
        assert(0);
    }
}

#ifdef HMCXBAR
void HMC_SerDes::handle_xbar_coh_msg(manifold::uarch::NetworkPacket *pkt)
{
#ifdef HMCDEBUG
    cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk" << "\thandle_xbar_coh_msg";
#endif
    unsigned int credit_type = 0;
    uint64_t pkt_delay = 0;
    if(pkt->type == COH_MSG_TYPE)
    {
        manifold::mcp_cache_namespace::Coh_msg* msg = (manifold::mcp_cache_namespace::Coh_msg*) pkt->data;
        switch(msg->type)
        {
            case 0:                     // Coh Request
            {
                stats_num_incoming_coh_req_xbar_msg++;
                /*
                 * A coh request is 1 FLIT = 16B. Assuming 16 bit lines for SerDes link
                 * 2B get transferred every SerDes clock tick.
                */
                pkt_delay = xbar_rx_calculate_delay(0);
#ifdef HUTDEBUG
                xbar_rx_addr.push_back(msg->addr);
#endif
#ifdef HMCDEBUG
                cerr << " COH REQ addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
                credit_type = 0;
                manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->serdes_clk, &HMC_SerDes::queue_after_serdes_time_to_net, this, pkt, credit_type);
                break;
            }
            case 1:                     // Coh Reply
            {
                stats_num_incoming_coh_resp_xbar_msg++;
                /*
                 * A coh reply is 3 FLITs (1 FLIT overhead + 2 FLITs data) = 48B.
                 * Assuming 16 bit lines for SerDes link 2B get transferred every
                 * SerDes clock tick.
                */
                pkt_delay = xbar_rx_calculate_delay(1);
#ifdef HUTDEBUG
                xbar_rx_addr.push_back(msg->addr);
#endif
#ifdef HMCDEBUG
                cerr << " COH REPLY addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
                credit_type = 1;
                manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->serdes_clk, &HMC_SerDes::queue_after_serdes_time_to_net, this, pkt, credit_type);
                break;
            }
            default:
            {
                cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tSerDes Received BAD COH MSG from xbar" << endl;
                assert(0);
            }
        }
    }

}
#endif


void HMC_SerDes::handle_net_mem_msg(manifold::uarch::NetworkPacket* pkt)
{    
#ifdef HMCDEBUG
    cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\thandle_net_mem_msg";
#endif
    unsigned int credit_type = 0;
    uint64_t pkt_delay = 0;
    if (pkt->type == MEM_MSG_TYPE)
    {
        manifold::mcp_cache_namespace::Mem_msg* msg = (manifold::mcp_cache_namespace::Mem_msg*) pkt->data;
        if (msg->type == 0)             // MEM_REQ
        {
            if (msg->is_read())         // READ REQ
            {
                stats_num_incoming_net_read_req_msg++;
                /*
                 *  Read request is 1 FLIT = 16B. 16 bit wide SerDes implies 8 SerDes clock ticks
                */
                pkt_delay = net_rx_calculate_delay(2);
#ifdef HUTDEBUG
                net_rx_addr.push_back(msg->addr);
#endif
#ifdef HMCDEBUG
                cerr << " MEM READ REQ addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
                credit_type = 0;
                manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->serdes_clk, &HMC_SerDes::queue_after_serdes_time_to_xbar, this, pkt, credit_type);
            }
            else                        // WRITE REQ
            {
                stats_num_incoming_net_write_req_msg++;
                /*
                 *  Write request is 3 FLITs = 48B. 16 bit wide SerDes implies 24 SerDes clock ticks
                */
                pkt_delay = net_rx_calculate_delay(3);
#ifdef HUTDEBUG
                net_rx_addr.push_back(msg->addr);
#endif
#ifdef HMCDEBUG
                cerr << " MEM WRITE REQ addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
                credit_type = 1;
                manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->serdes_clk, &HMC_SerDes::queue_after_serdes_time_to_xbar, this, pkt, credit_type);
            }
        }
        else if (msg->type == 1)        // MEM_RPLY
        {
            cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tSerDes handle_net_msg MEM MSG TYPE ERROR. CPU cannot send read/write response type" << endl;
            assert(0);
        }
        else
        {
            cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tserdes_id\t" << this->get_serdes_id() << "\trcvd NET MSG pkt BAD REQ TYPE ERROR!!!! " << endl;
            assert(0);
        }
    }

    else
    {
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tSerDes Received BAD MSG TYPE from NET" << endl;
        assert(0);
    }
}

#ifdef HMCXBAR
void HMC_SerDes::handle_net_coh_msg(manifold::uarch::NetworkPacket* pkt)
{
#ifdef HMCDEBUG
    cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\thandle_net_coh_msg";
#endif
    unsigned int credit_type = 0;
    uint64_t pkt_delay = 0;
    if(pkt->type == COH_MSG_TYPE)
    {
        manifold::mcp_cache_namespace::Coh_msg* msg = (manifold::mcp_cache_namespace::Coh_msg*) pkt->data;
        switch(msg->type)
        {
            case 0:                     // Coh Request
            {
                stats_num_incoming_coh_req_net_msg++;
                /*
                 * A coh request is 1 FLIT = 16B. Assuming 16 bit lines for SerDes link
                 * 2B get transferred every SerDes clock tick.
                */
                pkt_delay = net_rx_calculate_delay(0);
#ifdef HUTDEBUG
                net_rx_addr.push_back(msg->addr);
#endif
#ifdef HMCDEBUG
                cerr << " COH REQ addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
                credit_type = 0;
                manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->serdes_clk, &HMC_SerDes::queue_after_serdes_time_to_xbar, this, pkt, credit_type);
                break;
            }
            case 1:                     // Coh Reply
            {
                stats_num_incoming_coh_resp_net_msg++;
                /*
                 * A coh reply is 3 FLITs (1 FLIT overhead + 2 FLITs data) = 48B.
                 * Assuming 16 bit lines for SerDes link 2B get transferred every
                 * SerDes clock tick.
                */
                pkt_delay = net_rx_calculate_delay(1);
#ifdef HUTDEBUG
                net_rx_addr.push_back(msg->addr);
#endif
#ifdef HMCDEBUG
                cerr << " COH REPLY addr\t" << hex << msg->addr << dec << "\tdelay\t" << pkt_delay << endl;
#endif
                credit_type = 1;
                manifold::kernel::Manifold::ScheduleClock(pkt_delay, this->serdes_clk, &HMC_SerDes::queue_after_serdes_time_to_xbar, this, pkt, credit_type);
                break;
            }
            default:
            {
                cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tSerDes Received NET BAD COH MSG" << endl;
                assert(0);
            }
        }
    }
    else
    {
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tSerDes Received BAD MSG TYPE from NET" << endl;
        assert(0);
    }
}
#endif

void HMC_SerDes :: queue_after_serdes_time_to_xbar(manifold::uarch::NetworkPacket *pkt, unsigned int type)
{
    // THIS FUNCTION IS SPECIFICALLY FOR NET RX MSGS
    assert(net_rx_dep_time.front() == serdes_clk.NowTicks());
    serdes_to_xbar_buffer.push_back(pkt);
#ifdef HMCXBAR
    int src_port = pkt->get_src_port();
    send_credit_upstream(src_port,type);
#else
    send_credit_upstream(type); // Send credits to Network (using downstream_credits)
#endif

#ifdef HMCDEBUG
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tqueue_after_serdes_time_to_xbar";
        cerr << "\tdst\t" << pkt->dst << endl;
        cerr << "8888888888 NET RX BUFFER STATS SerDes" << this->get_serdes_id() << " 8888888888888" << endl;
        cerr << "ARR_CLK\t\tTYPE\tDEP_CLK\t\tADDR" << endl;
#ifdef HUTDEBUG
        std::list<uint64_t>::iterator it1 = net_rx_cur_clk_tick.begin();
        std::list<uint64_t>::iterator it2 = net_rx_pkt_type.begin();
        std::list<uint64_t>::iterator it4 = net_rx_addr.begin();
#endif /* HUTDEBUG */
        for(std::list<uint64_t>::iterator it3 = net_rx_dep_time.begin(); it3!=net_rx_dep_time.end(); ++it3)
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
    net_rx_dep_time.pop_front();

#ifdef HUTDEBUG
    net_rx_cur_clk_tick.pop_front();
    net_rx_pkt_type.pop_front();
    net_rx_addr.pop_front();
#endif /* HUTDEBUG */
}

void HMC_SerDes :: queue_after_serdes_time_to_net(manifold::uarch::NetworkPacket *pkt, unsigned int type)
{
    // THIS FUNCTION IS SPECIFICALLY FOR XBAR RX MSGS
    assert( xbar_rx_dep_time.front() == serdes_clk.NowTicks() );
    serdes_to_net_buffer.push_back(pkt);
    send_credit_downstream(type); // Send credits to hmcxbar (upstream_credits)

#ifdef HMCDEBUG
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tqueue_after_serdes_time_to_net";
        cerr << "\tdst\t" << pkt->dst << endl;
        cerr << "8888888888 XBAR RX BUFFER STATS SerDes" << this->get_serdes_id() << " 8888888888888" << endl;
        cerr << "ARR_CLK\t\tTYPE\tDEP_CLK\t\tADDR" << endl;
#ifdef HUTDEBUG
        std::list<uint64_t>::iterator it1 = xbar_rx_cur_clk_tick.begin();
        std::list<uint64_t>::iterator it2 = xbar_rx_pkt_type.begin();
        std::list<uint64_t>::iterator it4 = xbar_rx_addr.begin();
#endif /* HUTDEBUG */
        for(std::list<uint64_t>::iterator it3 = xbar_rx_dep_time.begin(); it3 != xbar_rx_dep_time.end(); ++it3)
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
    xbar_rx_dep_time.pop_front();

#ifdef HUTDEBUG
    xbar_rx_cur_clk_tick.pop_front();
    xbar_rx_pkt_type.pop_front();
    xbar_rx_addr.pop_front();
#endif /* HUTDEBUG */
}

void HMC_SerDes :: send_credit_upstream(int llp_lls_port, int type)
{
//    cerr << " Code should not be reaching here right now!!!" << endl;
//    exit(0);
    manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
    credit_pkt->type = CREDIT_MSG_TYPE;
    credit_pkt->set_dst_port(llp_lls_port);
    Send(NET_PORT, credit_pkt);
#ifdef HMCDEBUG
    cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tserdes_id\t" << this->get_serdes_id() << "\tsending CREDIT pkt from NET port\t"
            << dec << NET_PORT << "\tdst_port\t" << llp_lls_port << endl;
#endif
    stats_num_outgoing_net_credits++;
    if (type == 0)
        intermediate_outgoing_net_credits++;
    else
        intermediate_outgoing_net_credits += resp_flit_size;
}

void HMC_SerDes :: send_credit_upstream(int type)
{
    manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
    credit_pkt->type = CREDIT_MSG_TYPE;
    Send(NET_PORT, credit_pkt);
#ifdef HMCDEBUG
    cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tserdes_id\t" << this->get_serdes_id() << "\tsending CREDIT pkt from NET port\t"
            << dec << NET_PORT << endl;
#endif
    stats_num_outgoing_net_credits++;
    if (type == 0)
        intermediate_outgoing_net_credits++;
    else
        intermediate_outgoing_net_credits += resp_flit_size;
}

void HMC_SerDes :: send_credit_downstream(int type)
{
    manifold::uarch::NetworkPacket *credit_pkt = new manifold::uarch::NetworkPacket();
    credit_pkt->type = CREDIT_MSG_TYPE;
    // Use this in hmcxbar to determine pkt size
    credit_pkt->set_dst_port(type);
    Send(XBAR_PORT, credit_pkt);
#ifdef HMCDEBUG
    cerr << dec << "@\t" << serdes_clk.NowTicks() << " SerDes clk\tserdes_id\t" << this->get_serdes_id() << "\tsending CREDIT pkt from MC port\t"
            << dec << XBAR_PORT << endl;
#endif
    stats_num_outgoing_xbar_credits++;
    if(type == 0)
        intermediate_outgoing_xbar_credits++;
    else
        intermediate_outgoing_xbar_credits += resp_flit_size;
}

uint64_t HMC_SerDes::net_rx_calculate_delay(int type)
{
#ifdef HMCDEBUG
    cerr << "\nSerDes" << this->get_serdes_id() << " net rx delay calculation" << endl;
#endif
#ifdef HUTDEBUG
    assert(net_rx_cur_clk_tick.size() == net_rx_dep_time.size());
    net_rx_cur_clk_tick.push_back(serdes_clk.NowTicks());
#endif
    uint64_t delay = 0;

    if(net_rx_dep_time.empty())
    {
        switch(type)
        {
        case 0:     // COH REQ
            delay = 7;
            net_rx_dep_time.push_back(serdes_clk.NowTicks() + 7);
            break;
        case 1:     // COH REPLY
            delay = 23;
            net_rx_dep_time.push_back(serdes_clk.NowTicks() + 23);
            break;
        case 2:     // READ REQ
            delay = 7;
            net_rx_dep_time.push_back(serdes_clk.NowTicks() + 7);
            break;
        case 3:     // WRITE REQ
            delay = 23;
            net_rx_dep_time.push_back(serdes_clk.NowTicks() + 23);
            break;
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
    #ifdef HUTDEBUG
        net_rx_pkt_type.push_back((uint64_t)type);
    #endif
        return delay;
    }
    else
    {
        switch(type)
        {
        case 0:     // COH REQ
            net_rx_dep_time.push_back(net_rx_dep_time.back() + 7);
            break;
        case 1:     // COH REPLY
            net_rx_dep_time.push_back(net_rx_dep_time.back() + 23);
            break;
        case 2:     // READ REQ
            net_rx_dep_time.push_back(net_rx_dep_time.back() + 7);
            break;
        case 3:     // WRITE REQ
            net_rx_dep_time.push_back(net_rx_dep_time.back() + 23);
            break;
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
        delay = net_rx_dep_time.back() - serdes_clk.NowTicks();
    #ifdef HUTDEBUG
        net_rx_pkt_type.push_back((uint64_t)type);
    #endif
        return delay;
    }
}

uint64_t HMC_SerDes::xbar_rx_calculate_delay(int type)
{
#ifdef HMCDEBUG
    cerr << "\nSerDes" << this->get_serdes_id() <<  " xbar rx delay calculation" << endl;
#endif

#ifdef HUTDEBUG
    assert(xbar_rx_cur_clk_tick.size() == xbar_rx_dep_time.size());
    xbar_rx_cur_clk_tick.push_back(serdes_clk.NowTicks());
#endif
    uint64_t delay = 0;

    if(xbar_rx_dep_time.empty())
    {
        switch(type)
        {
        case 0:     // COH REQ
            xbar_rx_dep_time.push_back(serdes_clk.NowTicks() + 7);
            delay = 7;
            break;
        case 1:     // COH REPLY
            xbar_rx_dep_time.push_back(serdes_clk.NowTicks() + 23);
            delay = 23;
            break;
        case 2:     // READ REPLY
            xbar_rx_dep_time.push_back(serdes_clk.NowTicks() + 23);
            delay = 23;
            break;
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
#ifdef HUTDEBUG
        xbar_rx_pkt_type.push_back((uint64_t)type);
#endif
        return delay;
    }
    else
    {
        switch(type)
        {
        case 0:     // COH REQ
            xbar_rx_dep_time.push_back(xbar_rx_dep_time.back() + 7);
            break;
        case 1:     // COH REPLY
            xbar_rx_dep_time.push_back(xbar_rx_dep_time.back() + 23);
            break;
        case 2:     // READ REPLY
            xbar_rx_dep_time.push_back(xbar_rx_dep_time.back() + 23);
            break;
        default:
            cerr << "\ndelay calculation failed" << endl;
            assert(0);
        }
#ifdef HUTDEBUG
        xbar_rx_pkt_type.push_back((uint64_t)type);
#endif
        delay = xbar_rx_dep_time.back() - serdes_clk.NowTicks();
        return delay;
    }
}

void HMC_SerDes::clear_bw_counters()
{
    intermediate_outgoing_xbar_credits = 0;
    intermediate_outgoing_net_credits = 0;
}

void HMC_SerDes::print_stats(ostream& out)
{
    out << "****** SERDES " << hmc_serdes_id << "********* stats:" << endl
    << "  incoming from net read req msg: " << stats_num_incoming_net_read_req_msg << endl
    << "  outgoing to xbar read req msg: " << stats_num_outgoing_xbar_read_req_msg << endl
    << "  incoming from net write req msg: " << stats_num_incoming_net_write_req_msg << endl
    << "  outgoing to xbar write req msg: " << stats_num_outgoing_xbar_write_req_msg << endl
    << "  incoming from xbar read resp msg: " << stats_num_incoming_xbar_read_resp_msg << endl
    << "  outgoing to net read resp msg: " << stats_num_outgoing_net_read_resp_msg << endl

#ifdef HMCXBAR
    << "  incoming from cache coh req msg: " << stats_num_incoming_coh_req_net_msg << endl
    << "  outgoing to xbar coh req msg: " << stats_num_outgoing_coh_req_xbar_msg << endl
    << "  incoming from cache coh resp msg: " << stats_num_incoming_coh_resp_net_msg << endl
    << "  outgoing to xbar coh resp msg: " << stats_num_outgoing_coh_resp_xbar_msg << endl

    << "  incoming from xbar coh req msg: " << stats_num_incoming_coh_req_xbar_msg << endl
    << "  outgoing to cache coh req msg: " << stats_num_outgoing_coh_req_net_msg << endl
    << "  incoming from xbar coh resp msg: " << stats_num_incoming_coh_resp_xbar_msg << endl
    << "  outgoing to cache coh resp msg: " << stats_num_outgoing_coh_resp_net_msg << endl
#endif

    << "  incoming net credits: " << stats_num_incoming_net_credits << endl
    << "  incoming xbar credits: " << stats_num_incoming_xbar_credits << endl
    << "  outgoing net credits: " << stats_num_outgoing_net_credits << endl
    << "  outgoing xbar credits: " << stats_num_outgoing_xbar_credits << endl
    << "  Average BW: " << (stats_num_outgoing_net_credits + stats_num_outgoing_xbar_credits) * req_flit_size / (serdes_clk.NowTicks() / serdes_clk.freq) << " GB/s" << endl;
}

} // namespace hmc_serdes
} //namespace manifold

