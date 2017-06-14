#ifndef SERDES_LINK_H
#define SERDES_LINK_H

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
namespace hmc_serdes{

//! The HMC needs a network interface. This class is a SerDes Link that connects
//! HMC xbar to the network via a SerDes link

struct HMC_SerDes_settings {
    HMC_SerDes_settings(int num_rx_tx_lines, int req_flit_size, int resp_flit_size, int d_credits, int u_credits) :
        num_rx_tx_lines(num_rx_tx_lines), req_flit_size(req_flit_size), resp_flit_size(resp_flit_size), downstream_credits(d_credits), upstream_credits(u_credits)
    {}

    int num_rx_tx_lines; // These are the number of data lines in a serdes link
    int req_flit_size; // Size of a mem or coh msg request FLIT
    int resp_flit_size; // Size of a mem or coh msg response FLIT
    int downstream_credits;
    int upstream_credits;
};

class HMC_SerDes : public manifold::kernel::Component {
public:
    enum {NET_PORT=0, XBAR_PORT, PORT_KITFOX};
    HMC_SerDes(int id, const HMC_SerDes_settings& hmc_serdes_settings, manifold::kernel::Clock&);
    ~HMC_SerDes() {}

    int get_serdes_id() { return hmc_serdes_id; }

    void tick();
    // Handler for incoming requests from the network
    template<typename T> void handle_net(int, manifold::uarch::NetworkPacket* pkt);
    // Handler for incoming responses from the hmcxbar
    template<typename T> void handle_hmcxbar_incoming(int, manifold::uarch::NetworkPacket* pkt);

    // These functions handle coherent messages and memory messages received from the cores
    // Internally they call the queue_coh_after_serdes_time and queue_mem_after_serdes_time.
    void handle_net_mem_msg(manifold::uarch::NetworkPacket* pkt);
    void handle_xbar_mem_msg(manifold::uarch::NetworkPacket* pkt);

#ifdef HMCXBAR
    void handle_net_coh_msg(manifold::uarch::NetworkPacket* pkt);
    void handle_xbar_coh_msg(manifold::uarch::NetworkPacket* pkt);
#endif

    // These functions queue the incoming network packets. A coh msg pkt goes back
    // towards the queues cores whereas mem msg pkts get send towards the vaults
    // based on the pkt->type and pkt->dst
    void queue_after_serdes_time_to_xbar(manifold::uarch::NetworkPacket* pkt, unsigned int type);
    void queue_after_serdes_time_to_net(manifold::uarch::NetworkPacket* pkt, unsigned int type);

    uint64_t net_rx_calculate_delay(int type);
    uint64_t xbar_rx_calculate_delay(int type);

    void send_credit_upstream(int,int);
    void send_credit_upstream(int);
    void send_credit_downstream(int);
    void clear_bw_counters();

    static void Set_msg_types(int mem, int credit, int coh) // Set some interface parameters
    {
        assert(Msg_type_set == false);
        MEM_MSG_TYPE = mem;
        CREDIT_MSG_TYPE = credit;
        COH_MSG_TYPE = coh;
        Msg_type_set = true;
    }

    void print_stats(ostream&);

#ifdef LIBKITFOX
    template<typename T>
    void handle_kitfox_proxy_request(int temp, T *kitfox_proxy_request);
#endif
protected:

    std::list<uint64_t> net_rx_dep_time;
    std::list<uint64_t> xbar_rx_dep_time;

#ifdef HUTDEBUG
    std::list<uint64_t> net_rx_cur_clk_tick;
    std::list<uint64_t> xbar_rx_cur_clk_tick;
    std::list<uint64_t> net_rx_pkt_type;
    std::list<uint64_t> xbar_rx_pkt_type;
    std::list<uint64_t> net_rx_addr;
    std::list<uint64_t> xbar_rx_addr;
#endif /* HUTDEBUG */

    // NEED BUFFERS FOR EACH DIRECTION
    std::list<manifold::uarch::NetworkPacket*> serdes_to_net_buffer; //store to_serdes (to_net) messages
    std::list<manifold::uarch::NetworkPacket*> serdes_to_xbar_buffer; //store to_xbar messages
    std::list<unsigned int> serdes_to_xbar_buffer_msg_type; //store to_xbar message type
//    std::list<unsigned int> serdes_to_net_buffer_msg_type; //store to_serdes (to_net) message type
private:
    static int MEM_MSG_TYPE;
    static int CREDIT_MSG_TYPE;
    static int COH_MSG_TYPE;
    static bool Msg_type_set;

    manifold::kernel::Clock& serdes_clk;
    int hmc_serdes_id;
    int downstream_credits;     // NI credits
    int upstream_credits;		// Vault side credits
    int num_rx_tx_lines;
    int req_flit_size;
    int resp_flit_size;
    int MAX_UPSTREAM_CREDITS;
    int MAX_DOWNSTREAM_CREDITS;

    double time;

    // stats
    // incoming from
    // outgoing to

#ifdef HMCXBAR
    unsigned stats_num_incoming_coh_req_net_msg;
    unsigned stats_num_incoming_coh_resp_net_msg;
    unsigned stats_num_outgoing_coh_req_net_msg;
    unsigned stats_num_outgoing_coh_resp_net_msg;

    unsigned stats_num_incoming_coh_req_xbar_msg;
    unsigned stats_num_incoming_coh_resp_xbar_msg;
    unsigned stats_num_outgoing_coh_req_xbar_msg;
    unsigned stats_num_outgoing_coh_resp_xbar_msg;
#endif
    unsigned stats_num_incoming_net_read_req_msg;
    unsigned stats_num_incoming_net_write_req_msg;
    unsigned stats_num_incoming_xbar_read_resp_msg;
    unsigned stats_num_outgoing_xbar_read_req_msg;
    unsigned stats_num_outgoing_xbar_write_req_msg;
    unsigned stats_num_outgoing_net_read_resp_msg;

    unsigned stats_num_incoming_net_credits;
    unsigned stats_num_incoming_xbar_credits;
    unsigned stats_num_outgoing_net_credits;
    unsigned stats_num_outgoing_xbar_credits;

    unsigned intermediate_outgoing_xbar_credits;
    unsigned intermediate_outgoing_net_credits;

#ifdef LIBKITFOX
    manifold::uarch::hmcserdes_counter_t counter;
#endif
};

//! handle memory requests incoming from network; type T is uarch::Mem_msg type
template<typename T>
void HMC_SerDes :: handle_net(int in_port, manifold::uarch::NetworkPacket* pkt)
{
    assert(in_port == NET_PORT);

    if (pkt->type == CREDIT_MSG_TYPE)
    {
        downstream_credits++;
        stats_num_incoming_net_credits++;
#ifdef HMCDEBUG
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " Serdes clk\tserdes_id\t" << this->get_serdes_id() << "\trcvd NET CREDIT pkt @ port\t" << dec << in_port
                <<"\tSerDes downstream credits\t" << downstream_credits-1 << "->" << downstream_credits << endl;
#endif
        assert(downstream_credits <= MAX_DOWNSTREAM_CREDITS);
        delete pkt;
        return;
    }

    if (pkt->type == MEM_MSG_TYPE)
    {
#ifdef HMCDEBUG
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " Serdes clk\tserdes_id\t" << this->get_serdes_id() << "\tSerDes Received PKT TYPE is MEM MSG" << endl;
#endif
        // MEM_MSG_TYPE
        /*
         * This function will calculate the SerDes Rx delay and send the packet to the
         * HMC xbar switch. Credits are sent upstream internally.
         */
        handle_net_mem_msg(pkt);
    }
#ifdef HMCXBAR
    else if (pkt->type == COH_MSG_TYPE)
    {
#ifdef HMCDEBUG
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " Serdes clk\tserdes_id\t" << this->get_serdes_id() << "\tSerDes Received PKT TYPE is COH MSG" << endl;
#endif
        // COH_MSG_TYPE
        /*
         * This function will calculate the SerDes Rx delay and send the packet to the
         * HMC xbar switch. Credits are sent upstream internally.
         */
        handle_net_coh_msg(pkt);
    }
#endif
    else
    {
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " Serdes clk\tserdes_id\t" << this->get_serdes_id() << "\tSerDes rcvd NET MSG pkt @ port\t"
             << dec << in_port << "\tBAD MSG ERROR\t" << endl;
        assert(0);
    }
}

//! handle memory responses incoming from hmcxbar; type T is Mem_msg type
template<typename T>
void HMC_SerDes :: handle_hmcxbar_incoming(int in_port, manifold::uarch::NetworkPacket* pkt)
{
    assert(in_port == XBAR_PORT);

    if (pkt->type == CREDIT_MSG_TYPE)
    {
        upstream_credits++;
        stats_num_incoming_xbar_credits++;
        assert(upstream_credits <= MAX_UPSTREAM_CREDITS);

#ifdef HMCDEBUG
        cerr << dec << "@\t" << serdes_clk.NowTicks() << " Serdes clk\tserdes_id\t" << this->get_serdes_id() << "\trcvd XBAR CREDIT pkt @ port\t" << dec << in_port
                <<"\tSerDes upstream credits\t" << upstream_credits-1 << "->" << upstream_credits << endl;
#endif
        delete pkt;
        return;
    }

    if (pkt->type == MEM_MSG_TYPE)
    {
        /*
         * This function will calculate the SerDes Tx delay and send the packet back to the
         * network or cache. Credits are sent downstream internally.
         */
        handle_xbar_mem_msg(pkt);
    }
#ifdef HMCXBAR
    else if (pkt->type == COH_MSG_TYPE)
    {
        /*
         * This function will calculate the SerDes Tx delay and send the packet back to the
         * network or cache. Credits are sent downstream internally.
         */
        handle_xbar_coh_msg(pkt);
    }
#endif

}


#ifdef LIBKITFOX
template <typename T>
void HMC_SerDes :: handle_kitfox_proxy_request(int temp, T *kitfox_proxy_request)
{
    assert(kitfox_proxy_request->get_type() == manifold::uarch::KitFoxType::hmcserdes_type);
    assert(kitfox_proxy_request->get_id() == hmc_serdes_id);

    double meas_time_period = 0;
    meas_time_period = (serdes_clk.NowTicks() * serdes_clk.period) - time;

     // Power = Num of FLITS * Bytes-per-FLIT * Bits-per-Byte * nJ-per-bit
    counter.serdes_power = (double)( ( (intermediate_outgoing_xbar_credits + intermediate_outgoing_net_credits) * 16 * 8 * 1.356e-9 ) / meas_time_period );
    cerr << "serdes_power= " << counter.serdes_power << endl;
    kitfox_proxy_request->set_serdes_power(counter.serdes_power);

    cerr << "int_out_credits_xbar= " << intermediate_outgoing_xbar_credits << " int_out_credits_net= " << intermediate_outgoing_net_credits << endl;
    cerr << "@\t" << serdes_clk.NowTicks() << " Serdes_clk\tserdes" << this->get_serdes_id() << "\tmeas_time_period\t" << meas_time_period;
    cerr << "\tBW\t" << (double) ( (intermediate_outgoing_xbar_credits + intermediate_outgoing_net_credits ) * 16 / meas_time_period) / (1024 * 1024 * 1024) << " GB/s" << endl;

    counter.clear();
    clear_bw_counters();

    Send(PORT_KITFOX, kitfox_proxy_request);
    time = serdes_clk.NowTicks() * serdes_clk.period;
}
#endif // LIBKITFOX

}   // hmc_serdes namespace
}   // manifold namespace
#endif // SERDES_LINK_H
