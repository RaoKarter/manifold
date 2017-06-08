#ifndef MC_BUILDER_H
#define MC_BUILDER_H

#include <map>
#include <libconfig.h++>
#include "CaffDRAM/Controller.h"
#include "DRAMSim2/dram_sim.h"
#include "uarch/DestMap.h"
#include "hmc_xbar.h"
#include "serdes_link.h"
#ifdef HUTDEBUG
#include "pkt_generator.h"
#endif

#ifdef LIBKITFOX
#include "kitfox_builder.h"
using namespace manifold::kitfox_proxy;
#endif

#ifdef HUTDEBUG
class SysBuilder_llp_HUT;
#else
class SysBuilder_llp;
#endif
class NetworkBuilder;
class CacheBuilder;

class MemControllerBuilder {
public:
    enum {CAFFDRAM, DRAMSIM, HMC}; //mc types

#ifdef HUTDEBUG
    MemControllerBuilder(SysBuilder_llp_HUT* b) : m_sysBuilder(b) {}
#else
    MemControllerBuilder(SysBuilder_llp* b) : m_sysBuilder(b) {}
#endif /* HUTDEBUG */

    virtual ~MemControllerBuilder() {}

    virtual int get_type() = 0;

    virtual void read_config(libconfig::Config&) = 0;
    virtual void create_mcs(std::map<int, int>& id_lp) = 0;
#ifdef HUTDEBUG
    virtual void connect_mc_network() = 0;
#else
    virtual void connect_mc_network(NetworkBuilder*) = 0;
#endif /* HUTDEBUG */
    virtual void print_config(std::ostream&);
    virtual void print_stats(std::ostream&) = 0;
    virtual void set_mc_map_obj(manifold::uarch::DestMap *mc_map) = 0;

protected:
#ifdef HUTDEBUG
    SysBuilder_llp_HUT* m_sysBuilder;
#else
    SysBuilder_llp* m_sysBuilder;
#endif /* HUTDEBUG */

    unsigned m_NUM_MC;
    std::vector<double> m_CLOCK_FREQ;
    std::vector<manifold::kernel::Clock*> m_clocks;
    bool m_use_default_clock;
    std::map<int, int> m_mc_id_cid_map; //node id to cid map

};


//builder for CaffDRAM
class CaffDRAM_builder : public MemControllerBuilder {
public:
#ifdef HUTDEBUG
    CaffDRAM_builder(SysBuilder_llp_HUT* b) : MemControllerBuilder(b) {}
#else
    CaffDRAM_builder(SysBuilder_llp* b) : MemControllerBuilder(b) {}
#endif

    int get_type() { return CAFFDRAM; }

    void read_config(libconfig::Config&);
    void create_mcs(std::map<int, int>& id_lp);
    void connect_mc_network(NetworkBuilder*);
    void set_mc_map_obj(manifold::uarch::DestMap *mc_map);

    manifold::caffdram::Dsettings& get_settings() { return m_dram_settings; }

    void print_config(std::ostream&);
    void print_stats(std::ostream&);

private:
    manifold::caffdram::Dsettings m_dram_settings;
    int m_MC_DOWNSTREAM_CREDITS; //credits for sending down to network
    int m_MEM_MSG_TYPE;
    int m_CREDIT_MSG_TYPE;
};



//builder for DramSim
class DramSim_builder : public MemControllerBuilder {
public:
#ifdef HUTDEBUG
    DramSim_builder(SysBuilder_llp_HUT* b) : MemControllerBuilder(b) {}
#else
    DramSim_builder(SysBuilder_llp* b) : MemControllerBuilder(b) {}
#endif /* HUTDEBUG */

    int get_type() { return DRAMSIM; }

    void read_config(libconfig::Config&);
    void create_mcs(std::map<int, int>& id_lp);
    void connect_mc_network(NetworkBuilder*);
    void set_mc_map_obj(manifold::uarch::DestMap *mc_map);

    void print_config(std::ostream&);
    void print_stats(std::ostream&);

private:
    int m_MC_DOWNSTREAM_CREDITS; //credits for sending down to network
    int m_MEM_MSG_TYPE;
    int m_CREDIT_MSG_TYPE;
    std::string m_DEV_FILE; //device file name
    std::string m_SYS_FILE; //system file name
    unsigned m_MEM_SIZE; //mem size;
};


//builder for HMC
class HMC_builder : public MemControllerBuilder
{
public:
#ifdef HUTDEBUG
    HMC_builder(SysBuilder_llp_HUT* b) : MemControllerBuilder(b) {}
#else
    HMC_builder(SysBuilder_llp* b) : MemControllerBuilder(b) {}
#endif /* HUTDEBUG */

    int get_type() { return HMC; }

    void read_config(libconfig::Config&);
    void create_mcs(std::map<int, int>& id_lp);
#ifdef HUTDEBUG
    void connect_mc_network();
    static int HMCLog2(unsigned num)
    {
        assert(num > 0);

        int bits = 0;
        while(((unsigned)0x1 << bits) < num)
        {
            bits++;
        }
        return bits;
    }
#else
    void connect_mc_network(NetworkBuilder*);
#endif /* HUTDEBUG */
    unsigned get_vault_size() { return m_VAULT_SIZE; }
    unsigned get_mem_size() { return m_MEM_SIZE; }
    unsigned get_num_vaults() { return m_NUM_VAULTS; }
    unsigned get_num_layers() { return m_NUM_LAYERS; }
    unsigned get_num_serdes() { return m_NUM_SERDES; }
    unsigned get_num_hmcs() { return m_NUM_HMCs; }
    int get_trans_size() { return m_TRANS_SIZE; }

    void set_mc_map_obj(manifold::uarch::DestMap *hmc_map);
    void print_config(std::ostream&);
    void print_stats(std::ostream&);

private:
    int m_XBAR_DOWNSTREAM_CREDITS; //credits for sending up to serdes
    int m_XBAR_UPSTREAM_CREDITS; //credits for sending down to vault
    int m_SERDES_DOWNSTREAM_CREDITS; //credits for sending up to network
    int m_SERDES_UPSTREAM_CREDITS; //credits for sending down to xbar
    int m_MC_DOWNSTREAM_CREDITS; //credits for sending up to xbar
    int m_SERDES_NUM_RX_TX_LINES;
    int m_SERDES_REQ_FLIT_SIZE;
    int m_SERDES_RESP_FLIT_SIZE;
    int m_COH_MSG_TYPE;
    int m_MEM_MSG_TYPE;
    int m_CREDIT_MSG_TYPE;
    std::string m_DEV_FILE; //device file name
    std::string m_SYS_FILE; //system file name
    unsigned m_VAULT_SIZE; //size of each vault
    unsigned m_MEM_SIZE; //mem size = m_NUM_VAULTS x m_VAULT_SIZE
    unsigned m_NUM_VAULTS; //number of vaults in each HMC
    unsigned m_NUM_LAYERS; //number of layers in each vault
    unsigned m_NUM_SERDES; //number of SerDes links to each HMC
    unsigned m_NUM_HMCs; // number of HMCs
    int m_TRANS_SIZE; // Cache line size
    std::vector<int> vault_node_idx_vec;
    std::set<int> vault_node_idx_set; //set is used to ensure each index is unique

    void set_dram_clocks(libconfig::Config&);
    void set_serdes_clocks(libconfig::Config&);
    void set_xbar_clocks(libconfig::Config&);
    //function to create SerDes links from within create_mcs();
    void create_serdes(int lp, int* hmc_cid);
    //function to create drams from within create_mcs();
    void create_drams(int lp, int* hmc_cid, manifold::uarch::PageBasedMap *mc_map);

#ifdef HUTDEBUG
    void create_pkt_generator(int lp);
#endif

protected:
    bool xbar_use_default_clock;
    bool serdes_use_default_clock;
    std::vector<double> hmc_xbar_CLOCK_FREQ;
    std::vector<double> hmc_SerDes_CLOCK_FREQ;
    std::vector<manifold::kernel::Clock*> xbar_clocks; //clock for the HMC xbar
    std::vector<manifold::kernel::Clock*> serdes_clocks; //clock for the HMC serdes
    std::map<int, int> m_hmc_id_cid_map; //hmc id to cid map
    std::map<int,int>* m_vault_id_cid_map; //vault id to cid map
    std::map<int,int>* m_serdes_id_cid_map; //vault id to cid map
    std::vector<unsigned> serdes_nw_ids;
#ifdef HUTDEBUG
    std::map<int,int> m_pkt_gen_id_cid_map;
    unsigned total_pkts;
    unsigned pkt_gen_upstream_credits;
#endif /* HUTDEBUG */

};


#endif // #ifndef NETWORK_BUILDER_H
